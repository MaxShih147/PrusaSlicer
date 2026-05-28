## 1. 修正 radius 映射邏輯（Offset −1）

- [x] 1.1 在 [src/libslic3r/Format/SL1.cpp](../../../src/libslic3r/Format/SL1.cpp) 的 `get_encoder` lambda（約第 362 行）將 `const unsigned radius = static_cast<unsigned>(blur_config + 1);` 改為 `static_cast<unsigned>(blur_config);`
- [x] 1.2 確認 `if (blur_config > 0)` 守門條件維持不變，且區塊其餘程式碼（rendering_buffer / pixfmt_gray8 / stack_blur_gray8 呼叫）不動
- [x] 1.3 **驗證**：靜態檢查映射數值 —— blur=0→跳過、blur=1→radius=1、blur=2→radius=2、blur=3→radius=3（確認不再有 +1，且永不把 radius=0 傳入 stack_blur）

## 2. 實作 alpha blend 混合邏輯（雙緩衝、動態 α 斜坡）

- [x] 2.1 在 `if (blur_config > 0)` 區塊內，於 stack_blur 之前建立 `std::vector<uint8_t> temp_buf(buf);`（複製量化後原圖；每層獨立配置，沿用既有 `buf` 模式）
- [x] 2.2 將 `agg::rendering_buffer` / `pixfmt_gray8` / `stack_blur_gray8` 改為作用於 `temp_buf`（就地模糊「副本」，保留 `buf` 為未模糊原圖）
- [x] 2.4 確認 `blur=0` 路徑完全不受影響（不配置 temp_buf、不混合）
- [x] 2.3 **改為動態 α 映射**：依 `α(b) = min(1.0, 0.6 + 0.2·(blur_config−1))` 計算 `k = round(α·256)`（blur=1→154、blur=2→205、blur≥3→256）。取代原本寫死的 `k=154`
- [x] 2.6 **k=256 優化路徑（就地模糊）**：當 `k >= 256`（α=1.0，blur≥3）時**不配置 temp_buf**，直接以 `rbuf` 指向 `buf.data()` 就地模糊、跳過混合迴圈（與 pre-mod 純模糊零冗餘複製等價）；其餘（blur=1/2）才配置 temp_buf 並走 `buf[p] = (buf[p]*(256-k) + temp_buf[p]*k) >> 8`
- [x] 2.5 **驗證（數值）**：靜態確認三組 k（154/205/256）對應的有效 σ 單調遞增（0.55/1.03/1.58）、尖峰 `(1-α)` 單調淡出（0.40/0.20/0.00）；混合結果在 [0,255] 不溢位（max 255·256>>8=255）；blur=4~10 維持 k=256

## 3. 記憶體壓力測試與視覺/效能回歸

- [x] 3.1 **記憶體壓力測試**：8K mono 多執行緒切片實機驗證，post-processing 峰值記憶體正常、用完即釋放、無持續累積（使用者實機確認）
- [x] 3.2 以 SLA 設定 `blur=1` 切片含 45° 斜邊的測試模型，輸出 SL1 並取出單層 PNG（使用者實機完成）
- [x] 3.3 **驗證（視覺）**：斜邊灰階轉場帶較純模糊更銳、blur=2/3 同步收窄（使用者實機確認）
- [x] 3.4 **驗證（關閉行為）**：靜態確認 `blur=0` 時整段 `if (blur_config > 0)` 跳過——不配置 temp_buf、不模糊、不混合，輸出與未套用模糊一致
- [x] 3.5 **驗證（效能）**：相同 `blur` 計時切片無顯著增加耗時（使用者實機確認）；blur≥3 已改就地模糊、與 pre-mod 純模糊零冗餘複製等價
- [x] 3.6 **驗證（對齊）**：blur=1/2/3 輸出與 Chitubox blur=2/3/4 並排比對——依使用者 v3 對照圖確認觀感高度近似、blur=2/3 硬邊消除
