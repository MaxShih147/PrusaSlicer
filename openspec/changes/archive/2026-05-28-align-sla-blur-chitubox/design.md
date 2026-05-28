## Context

SLA 切片影像的層級模糊後處理位於 `SL1Archive::get_encoder` 回傳的 lambda 內（[src/libslic3r/Format/SL1.cpp](../../../src/libslic3r/Format/SL1.cpp)，約第 299–371 行）。處理鏈順序為：AA 量化 → gray level 提亮 → blur。blur 為最後一步，僅當 `blur > 0` 時執行。

`stack_blur_gray8`（[bundled_deps/agg/agg/agg_blur.h](../../../bundled_deps/agg/agg/agg_blur.h)）是半寬為 `radius` 的三角權重核（Bartlett window）。在 `r=1` 時權重為 `[1,2,1]/4`（中心 0.50），其分度即等效 σ ≈ √(((r+1)²−1)/6)。

**演進脈絡：** 第一步先移除舊有多餘的 `+1` 偏移（`radius = blur_config + 1` → `radius = blur_config`，已於階段 1 完成）。但事後以截圖（同解析度、同放大倍率）目視比對發現：即使 `r=1`，DS 的轉場帶仍**略寬於** Chitubox 對應階。由於 `r=1` 已是最小整數半徑（3px footprint），無法再用整數半徑收窄，需要一個「比 r=1 更銳、又比無模糊平滑」的次像素機制。

關鍵數學洞察：在固定 3px footprint 下，轉場帶寬度由 footprint（半徑）決定，**銳利度由中心權重決定**。提高中心權重等價於把原圖與模糊圖做線性混合：

```
out = (1-α)·original + α·blurred
等效核 = [α/4, 1-α/2, α/4]   （以 r=1 三角核為例）
α=1.0 → [0.25, 0.50, 0.25]（現況，略寬）
α=0.6 → [0.15, 0.70, 0.15]（更銳、仍平滑）
```

此混合即「隱藏的次半徑旋鈕」，且**不需雙次捲積、不增加 stack_blur 的演算成本**。

## Goals / Non-Goals

**Goals:**
- 在 `radius = blur`（已修正）之上，疊加固定 α 混合，達成「比 r=1 更銳、比無模糊平滑」的次像素銳度。
- 使 DS blur 1~3 視覺上近似 Chitubox blur 2~4。
- 混合對**所有 `blur > 0` 等級**一致套用（系統性偏移，非僅 blur=1）。
- 維持切片效能與可控的記憶體峰值。

**Non-Goals:**
- 不追求像素級精確對齊，「效果近似即可」。
- 不新增 UI 參數：α 為內部校準常數，不暴露給使用者（不動 preset/invalidation/Tab 接線）。
- 不更動 `blur` 參數的範圍（min=0 / max=10）、UI 語意。
- 不更動 `stack_blur_gray8` 演算法本身，僅在其輸出之上做混合。

## Decisions

**決策 1：半徑映射採 Offset 修正（`radius = blur`）。**
移除舊有 `+1`。`if (blur_config > 0)` 守門維持不變，確保 `blur=0` 跳過、永不傳入 radius=0。（已於階段 1 完成。）

**決策 2：以雙緩衝 α 混合達成次像素銳度。**
不走「更小整數半徑」（已無空間）或「自寫變寬核」，而以 `out = (1-α)·original + α·blurred` 在既有 `stack_blur_gray8` 輸出之上混合。
- 替代方案：僅銳化 blur=1（自寫 3-tap 就地捲積，可省第二塊 buffer）—— 否決。兩條程式路徑增加維護與出錯風險，違背「零意外」，且不利於後續動態 α。

**決策 3：硬尖峰（Spike）分析 —— α 須隨半徑動態調整。**
混合核可拆解為「硬尖峰 + 模糊暈」：

```
K_eff = (1-α)·δ   +   α·K_r
        └─硬尖峰─┘     └─模糊暈(寬度=r)─┘
```

`(1-α)·δ` 保留原始未模糊的銳利邊。當 r=1 時模糊暈僅 ±1px，尖峰與暈重疊成單一平滑 3px 漸層（觀感理想）；但 r≥2 時暈擴散開來，**固定強度的硬尖峰外露**，形成「硬核心 + 分離淡暈」=使用者回報的 blur=2/3「邊緣太硬」視覺斷層。
- 故 α 不可全等級固定：必須隨 blur 遞增、令尖峰強度 `(1-α)` 逐步淡出。
- 替代方案（否決）：固定 α=0.6 全套 —— blur=2/3 殘留 0.40 尖峰，硬邊。
- 替代方案（否決）：二元「blur==1→0.6, else→1.0」—— 雖無寬度反轉，但邊緣質感在 1→2 之間由「尖峰 0.40」驟跳到「0.00」，質感不連續，仍有斷層感。

**決策 4：動態 α 斜坡 `α(b) = min(1.0, 0.6 + 0.2·(b−1))`。**

| blur | α | k=round(α·256) | 256−k | 尖峰(1-α) | 有效 σ_eff=√α·σ_r |
|------|-----|------|-------|-----------|------|
| 1 | 0.6 | 154 | 102 | 0.40 | 0.55 |
| 2 | 0.8 | 205 | 51 | 0.20 | 1.03 |
| ≥3 | 1.0 | 256 | 0 | 0.00 | =σ_r（1.58…） |

（σ_r：三角核標準差，blur 1/2/3 = 0.707 / 1.155 / 1.581。）此斜坡保證：(a) σ_eff 單調遞增（0.55→1.03→1.58，無「低階更糊」反轉）；(b) 尖峰強度單調淡出（0.40→0.20→0.00），blur≥3 完全無硬核心，徹底消除大半徑硬邊。blur=4~10 維持 α=1.0（純模糊）。

**決策 5：整數定點混合 + k=256 優化路徑。**
`buf[p] = (buf[p] * (256 - k) + temp_buf[p] * k) >> 8`，`k = round(α(b)·256)`。符合「低延遲」、結果可重現。當 `k == 256`（α=1.0，blur≥3）時混合退化為「直接取 temp_buf」，**跳過 per-pixel 混合迴圈**（直接以模糊結果編碼，等同純 stack_blur），零額外成本。

## Double-Buffered Alpha Blending（雙緩衝混合同步策略）

**資料流（`blur > 0` 時）：**

```
k = round(alpha(blur_config) * 256)     ← 動態：blur 1→154, 2→205, ≥3→256
buf      = 量化後原圖（既有，1× w×h）  ← 保持「原始/銳利」
temp_buf = std::vector<uint8_t>(buf)    ← 每層獨立複製，1× w×h
stack_blur_gray8(temp_buf, radius, radius)   ← 就地模糊「副本」
if k >= 256:                             ← α=1.0 優化路徑
    encode(temp_buf)                     ← 直接取模糊圖，跳過混合迴圈
else:
    for p in [0, bufsize):
        buf[p] = (buf[p] * (256-k) + temp_buf[p] * k) >> 8   ← 定點混合回 buf
    encode(buf)
```

**為何必須第二塊全幅 buffer：** `stack_blur_gray8` 為就地運算，混合需同時握有「原始」與「模糊」兩份影像。複用既有 stack_blur ⇒ 必須保留一份未模糊副本。無法用單行歷史規避（混合需逐像素的原始值與模糊值並存）。

**記憶體決策：每層獨立 `std::vector`，不用 thread-local。**
- `get_encoder()` 在 [SLAArchiveWriter.hpp:58](../../../src/libslic3r/Format/SLAArchiveWriter.hpp#L58) 的平行 `for_each` 內**每層重建**，僅捕獲三個 `int`，天生 thread-safe。
- temp_buf 採與既有 `buf` 相同模式：lambda 內 `std::vector`，RAII、用完即釋放。
- 否決 `static thread_local`：雖省 malloc churn，卻會整個 session 滯留 `N × w×h`，並引入類共享狀態的理解負擔，違背「零意外」。一次 memcpy（8K ≈ 33MB，約 3–5ms）相對 blur + PNG 編碼可忽略。
- 峰值記憶體：post-processing transient 由 `N × w×h` 升為 `N × 2×w×h`（N = 平行度）。8K mono × 16 執行緒約 +0.5GB transient，用完即釋放——故需階段 3 的記憶體壓力測試確認大尺寸場景。

**失敗安全性：fail-loud，不做降級。**
- 編碼發生在 `draw_layers` 平行區，**不在** [SL1.cpp:409](../../../src/libslic3r/Format/SL1.cpp#L409) 的 try/catch 內；既有 `buf` 配置本就有「bad_alloc 往外拋、終止切片」的曝險。
- temp_buf 沿用相同行為：若單塊配置失敗，raster/PNG/全流程早已不可行；「跳過 blend」式降級只會產出**靜默錯誤**卻宣稱成功，比直接失敗更危險。讓 bad_alloc 往外拋（fail-loud）才穩健且與既有一致。

## Risks / Trade-offs

- [既有專案沿用同一 `blur` 值，輸出將更銳] → 為本變更之預期效果；於提案/釋出說明告知語意已調整。
- [α=0.6 為目視校準起點，可能需微調] → 以對照圖事後檢核；提供 0.5~0.7 調整區間，僅需改一個常數。
- [峰值記憶體上升 N×w×h] → 以階段 3 記憶體壓力測試（8K/多執行緒）確認；transient、用完即釋放，風險可控。
- [blur≥2 混合為振幅弱化而非變窄] → 仍朝 Chitubox 收斂；若高階目視明顯不符，再評估是否分級套不同 α，不在本次預設範圍。
- [效能] → 多一次 memcpy + 一輪定點混合（O(w×h)），相對 blur + PNG 可忽略；stack blur 工作量不變。
