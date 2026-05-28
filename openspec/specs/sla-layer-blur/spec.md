# sla-layer-blur Specification

## Purpose

定義 SL1 SLA 切片輸出的層級模糊（layer blur）後處理行為，使其視覺結果對齊 Chitubox 的模糊半徑與邊緣銳度，並控制其效能與記憶體開銷。

## Requirements

### Requirement: 模糊半徑映射對齊 Chitubox

SL1 切片輸出的層級模糊後處理，其傳入 `agg::stack_blur_gray8` 的半徑 SHALL 等於使用者設定值 `blur`（即 `radius = blur`），不得再額外加上 `+1` 偏移。此映射搭配次像素銳度補強（見下），使 DS-online 的 blur 1~3 在視覺上近似 Chitubox 的 blur 2~4。

對應的整數半徑與三角核標準差：

| blur 設定 | radius | 三角核 σ_r |
|---|---|---|
| 1 | 1 | ≈ 0.71 |
| 2 | 2 | ≈ 1.15 |
| 3 | 3 | ≈ 1.58 |

#### Scenario: blur=1 的轉場帶對齊到 r=1

- **WHEN** `blur = 1`，對一個黑白硬邊（step edge）影像套用模糊後處理
- **THEN** 套用的 stack blur 半徑 SHALL 為 `1`（三角核 [1,2,1]/4）
- **AND** 不再有 `radius = blur + 1` 所造成的 5px 過寬轉場帶

#### Scenario: blur=2 與 blur=3 的逐階半徑

- **WHEN** `blur = 2` 或 `blur = 3`
- **THEN** 套用的 stack blur 半徑 SHALL 分別為 `2` 與 `3`

### Requirement: 次像素銳度補強（動態 Alpha Blending）

由於最小整數半徑 `r=1` 仍略寬於 Chitubox，系統 SHALL 在 `stack_blur_gray8` 輸出之上，將「未模糊原圖」與「模糊圖」依 `out = (1-α)·original + α·blurred` 線性混合，以在固定 footprint 內提高中心權重、達成次像素銳度。

混合核可拆解為「硬尖峰 `(1-α)·δ` + 模糊暈 `α·K_r`」。硬尖峰在小半徑時被模糊暈遮蔽（觀感平滑），但在大半徑（r≥2）時外露，造成「硬核心 + 分離淡暈」的硬邊。故 α SHALL 隨 `blur` 等級遞增，使尖峰強度 `(1-α)` 逐步淡出。

α 與定點係數 `k = round(α·256)` 的對照（公式 `α(b) = min(1.0, 0.6 + 0.2·(b−1))`）：

| blur | α | k | 尖峰 (1-α) | 有效 σ_eff = √α·σ_r |
|------|-----|-----|-----------|---------------------|
| 1 | 0.6 | 154 | 0.40 | ≈ 0.55 |
| 2 | 0.8 | 205 | 0.20 | ≈ 1.03 |
| ≥3 | 1.0 | 256 | 0.00 | = σ_r（≈ 1.58…） |

混合 SHALL 以整數定點運算 `buf[p] = (buf[p]*(256−k) + temp_buf[p]*k) >> 8` 實作，避免 per-pixel 浮點。

#### Scenario: blur=1 套用 α=0.6 銳化

- **WHEN** `blur = 1`
- **THEN** 系統 SHALL 以 `k = 154`（α=0.6）混合原圖與模糊圖
- **AND** 結果轉場帶 SHALL 比未混合的純 r=1 模糊更銳（暗側鄰點亮度由約 25% 降至約 15%）

#### Scenario: blur=2 套用 α=0.8 柔化尖峰

- **WHEN** `blur = 2`
- **THEN** 系統 SHALL 以 `k = 205`（α=0.8）混合
- **AND** 殘留硬尖峰強度 SHALL 降至 0.20，消除明顯硬核心

#### Scenario: blur≥3 全模糊無尖峰

- **WHEN** `blur >= 3`（含設定範圍上限 4~10）
- **THEN** α SHALL 為 `1.0`（k=256），不保留任何硬尖峰
- **AND** 系統 SHALL 走純模糊路徑、不進行混合，輸出與 Chitubox 大半徑柔邊一致

#### Scenario: 有效模糊強度單調遞增

- **WHEN** 比較 blur=1、2、3 的輸出
- **THEN** 有效 σ_eff SHALL 單調遞增（≈ 0.55 < 1.03 < 1.58），不得出現「低階反而更糊」的反轉

### Requirement: blur 關閉時不套用模糊

當 `blur = 0` 時，系統 SHALL 不執行任何模糊或混合運算，且 SHALL NOT 將半徑 `0` 傳入 `stack_blur_gray8`，亦 SHALL NOT 配置混合用暫存緩衝。

#### Scenario: blur=0 維持原始邊緣

- **WHEN** `blur = 0`
- **THEN** 模糊/混合後處理區塊 SHALL 被完整跳過（受 `blur > 0` 守門）
- **AND** 輸出影像的邊緣 SHALL 與未套用模糊時一致

### Requirement: 切片效能與記憶體開銷

模糊後處理 SHALL NOT 顯著增加切片時間。`stack_blur_gray8` 為 O(N) 滑動窗口、複雜度與半徑無關。

- 當 α=1.0（`blur >= 3`）時，系統 SHALL 對原始 `buf` 就地模糊、不配置暫存緩衝、不執行混合迴圈，使其運算量與「未加入混合機制前的純模糊」完全等價（零冗餘複製）。
- 當 α<1.0（`blur = 1` 或 `2`）時，系統需配置一塊 `w×h` 暫存緩衝以保留未模糊原圖，並執行一輪 O(N) 定點混合；此額外開銷（一次全幅複製 + 一輪混合）相對 stack blur 與 PNG 編碼為小量，且暫存緩衝為每層獨立配置、用完即釋放，不跨層累積。

#### Scenario: blur≥3 效能與修改前等價

- **WHEN** 以 `blur >= 3` 切片同一模型
- **THEN** 不得配置 `temp_buf`，模糊 SHALL 就地作用於 `buf`
- **AND** 後處理耗時 SHALL 與未加入混合機制前的純模糊相當

#### Scenario: blur=1/2 額外開銷可忽略

- **WHEN** 以 `blur = 1` 或 `blur = 2` 切片
- **THEN** 額外的暫存複製與定點混合耗時 SHALL 相對 blur + PNG 編碼可忽略
- **AND** 暫存緩衝 SHALL 於該層編碼後釋放，峰值記憶體不持續累積
