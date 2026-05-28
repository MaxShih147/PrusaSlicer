## Why

DS-online 目前的 SLA 切片模糊（blur）效果偏強：在相同解析度與放大倍率下，DS blur 1~3 的漸層過渡帶都比 Chitubox 對應階別寬，導致邊緣過度糊化。使用者希望「效果近似即可」，將 DS blur 1~3 重新對齊到 Chitubox blur 2~4 的觀感，同時嚴禁顯著增加切片時間。

第一步先移除舊有多餘的 `+1` 半徑偏移（`radius = blur + 1` → `radius = blur`）。但事後目視比對發現：即使縮到最小整數半徑 `r=1`，DS 的轉場帶仍**略寬於** Chitubox。由於 `r=1` 已是 3px 的最小 footprint、無法再用整數半徑收窄，需要一個「比 r=1 更銳、又比無模糊平滑」的次像素機制。

## What Changes

- 修正 SL1 切片輸出後處理中的模糊半徑映射：由 `radius = blur_config + 1` 改為 **`radius = blur_config`**（移除多餘的 `+1` 偏移）。
- **採用 Alpha Blending 補強**：在固定整數半徑 `r=1` 仍略微過糊的限制下，於 `stack_blur_gray8` 輸出之上，將「未模糊原圖」與「模糊圖」依 `out = (1-α)·original + α·blurred` 線性混合。等價於在固定 footprint 內提高中心權重，達成次像素銳度。
- **動態 α 斜坡（依 blur 等級調整）**：混合核本質為「硬尖峰 `(1-α)·δ` + 模糊暈 `α·K_r`」。尖峰在小半徑時被暈遮蔽（看起來平滑），但在大半徑（r≥2）時暈擴散開來、硬尖峰外露，造成「硬核心 + 分離淡暈」的視覺斷層感（使用者回報 blur=2/3「邊緣太硬」）。故 α 須隨等級遞增，使尖峰逐步淡出：

  | blur | α | k | 說明 |
  |------|-----|-----|------|
  | 1 | 0.6 | 154 | 銳化（已驗證為理想觀感） |
  | 2 | 0.8 | 205 | 尖峰減半、柔化 |
  | ≥3 | 1.0 | 256 | 純模糊、零尖峰 |

  公式：`α(b) = min(1.0, 0.6 + 0.2·(b−1))`。此斜坡使有效 σ 單調遞增（不會發生「低階反而更糊」的反轉），且尖峰強度 `(1-α)` 由 0.40→0.20→0.00 逐步淡出，消除大半徑硬邊。
- 採整數定點運算 `buf[p] = (buf[p]*(256-k) + temp_buf[p]*k) >> 8`（`k = round(α*256)`），避免 per-pixel float，維持低延遲且結果可重現。
- **k=256 優化路徑**：當 α=1.0（blur≥3）時混合退化為「直接取模糊圖」，可跳過 per-pixel 混合迴圈（等同純 stack_blur），零額外成本。
- 行為保持：`blur=0` 仍維持「不套用模糊」（受 `if (blur_config > 0)` 守門，永不把 radius=0 餵給 stack blur，亦不進行混合）。`blur` 參數範圍 min=0 / max=10 不變。
- α 為內部校準常數，**不新增 UI 參數**，不動 preset / invalidation / Tab 接線。

## Capabilities

### New Capabilities
- `sla-layer-blur`: SLA 切片影像於 SL1 輸出時套用的層級模糊後處理，涵蓋 `blur` 參數值到 `stack_blur_gray8` 半徑的映射規則、alpha blend 銳度補強與邊界行為。

### Modified Capabilities
<!-- 無既有 spec 需修改 -->

## Impact

- **程式碼**：[src/libslic3r/Format/SL1.cpp](../../../src/libslic3r/Format/SL1.cpp) 中 `get_encoder` lambda 的模糊區塊（約第 360–368 行）—— 半徑計算、新增 `temp_buf` 雙緩衝、stack blur 改作用於副本、以及定點混合迴圈。
- **演算法依賴**：`agg::stack_blur_gray8`（[bundled_deps/agg/agg/agg_blur.h](../../../bundled_deps/agg/agg/agg_blur.h)）—— 僅在其輸出之上混合，演算法本身不動。
- **效能**：stack blur 為 O(N) 滑動窗口、與半徑無關；新增一次 memcpy（複製 `temp_buf`）與一輪 O(w×h) 定點混合，相對 blur + PNG 編碼可忽略，符合「嚴禁顯著增加切片時間」。
- **記憶體（預期略微增加）**：alpha blend 需保留一份未模糊副本，故每層 post-processing 的暫存記憶體由 `1×(w×h)` 升為 `2×(w×h)`。因編碼於 `draw_layers` 平行區執行，峰值約為 `N × 2×(w×h)`（N = 平行度）。8K mono × 多執行緒約增加數百 MB transient，**用完即釋放、不持續累積**（採每層獨立 `std::vector`，非 thread-local）。
- **設定/相容性**：`blur` 參數定義與範圍不變；既有專案沿用同一參數值，但輸出將更銳（為本變更之預期效果）。
