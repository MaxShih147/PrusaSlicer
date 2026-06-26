# sla-support-binary-rasterization Specification

## Purpose
TBD - created by archiving change sla-support-binary-rasterization. Update Purpose after archive.
## Requirements
### Requirement: 支撐幾何以獨立 STL 跨服務傳入

支撐幾何 SHALL 以**獨立的 STL 檔**跨服務傳遞，而非與主體合併成單一 STL。前端 `DS-online` 切片時 SHALL 上傳兩個獨立 blob（主體 `model.stl` 與支撐 `support.stl`）；後端 `web_slicer_core` SHALL 落地第二 blob 為 `input/support.stl`，並於偵測到該檔時於切片器命令動態加入 `--import-support-stl <path>`。本流程 SHALL 維持 `supports_enable=False`（切片器不自生支撐）與 `pad_enable=False`（底筏已含於匯入的 `support.stl`）。本範圍 SHALL 僅支援單一 `SLAPrintObject`。

#### Scenario: 前端雙 blob 獨立上傳
- **GIVEN** 使用者在 DS-online 完成主體與支撐的擺放（兩者為可獨立操作的分離物件）
- **WHEN** 使用者觸發切片
- **THEN** 前端上傳主體與支撐兩個獨立 STL blob，不在前端合併
- **AND** 後端將支撐 blob 落地為 `input/support.stl`

#### Scenario: 後端動態組裝 CLI 參數
- **GIVEN** 後端 job 目錄存在 `input/support.stl`
- **WHEN** `run_slicing` 組裝 PrusaSlicer 命令
- **THEN** 命令包含 `--import-support-stl <input/support.stl 路徑>`
- **AND** `supports_enable` 與 `pad_enable` 維持 False（不自生支撐與底筏）

#### Scenario: 無支撐檔時向後相容
- **GIVEN** 後端 job 目錄不存在 `input/support.stl`
- **WHEN** `run_slicing` 組裝命令並切片
- **THEN** 命令不含 `--import-support-stl`，切片器行為與現況一致（無回歸）

### Requirement: 切片器載入匯入支撐並復原 soSupport 軌

切片器 SHALL 提供 CLI 選項 `--import-support-stl <path>`，讀入獨立支撐 STL 為 `TriangleMesh`，掛載至對應 `SLAPrintObject` 的 `m_supportdata->tree_mesh`，並令 `support_points` 與 `support_tree` 步驟跳過自生、`slice_supports` 跨越 `supports_enable` 閘門，將匯入網格切為 `support_slices` → `soSupport`。匯入支撐 MUST 與主體掛在同一 `SLAPrintObject` 並套用相同物件 `trafo`（合約 A：共用世界座標），使兩者在切片平面自然對齊、無需額外變換矩陣。

#### Scenario: 匯入支撐被切為 soSupport
- **GIVEN** 切片器以 `--import-support-stl support.stl` 啟動、`supports_enable=False`
- **WHEN** 執行 SLA 管線
- **THEN** `support_points` 與 `support_tree` 不生成自生支撐，匯入網格成為 `tree_mesh`
- **AND** `slice_supports` 將匯入網格切片，`soSupport` 非空、`merged_input_to_slices()` 的支撐軌被填充

#### Scenario: 合約 A 座標自然對齊
- **GIVEN** 前端以共用世界座標匯出 `model.stl` 與 `support.stl`
- **WHEN** 切片器把支撐掛在同一 `SLAPrintObject` 並套用相同 `trafo`
- **THEN** 支撐與主體在每一切片平面上正確貼合，無錯位、無需傳遞變換矩陣

### Requirement: 模型與支撐雙軌差異化光柵

切片器在 SLA 光柵化時 SHALL 將 `soModel` 與 `soSupport` 保留為兩條獨立軌道，不得以 `union_ex` 合併為單軌。模型軌 SHALL 套用 AA gamma 與後處理（量化＋blur）；支撐軌 SHALL 以 threshold 二值繪製、豁免 AA 與 blur，且 MUST 於模型後處理「之後」就地合成於同一像素緩衝（`m_buf`），不開闢額外全解析度緩衝。

#### Scenario: 兩軌分別以不同光柵模式處理
- **GIVEN** 一個切片層同時含有模型與（匯入而來的）支撐幾何
- **WHEN** 系統執行該層光柵化
- **THEN** 模型像素以 AA 灰階呈現邊緣、支撐像素呈純白（255）二值
- **AND** 支撐的繪製發生在 blur／量化之後，使支撐區不受任何柔化影響

### Requirement: 獨立支撐的 RLE 優化邊緣

當支撐與模型無接觸（獨立支撐）時，系統 SHALL 確保支撐像素周圍的背景維持純黑（值 0），不得存在因 blur 擴散而產生的灰階光暈（halo）。此行為 SHALL 降低後端 row-major RLE 的 run 數，使最終 prz 檔案體積顯著小於現況。

#### Scenario: DS-online 切獨立支撐物件後 prz 體積下降
- **GIVEN** 使用者在 DS-online 載入含獨立支撐（與模型無接觸）的物件並以雙 blob 切片
- **WHEN** 使用者下載 prz
- **THEN** 支撐像素的緊鄰背景為純黑 0、無灰階漸層光暈
- **AND** 該 prz 體積顯著小於同物件於現況（main）所切出的 prz

#### Scenario: 支撐邊界為單一二值跳變
- **GIVEN** 含獨立支撐的切片層
- **WHEN** 沿一條掃描線跨越支撐邊界
- **THEN** 像素由 0 直接跳變為 255（或反向），中間不存在灰階過渡像素

### Requirement: 主體模型的 AA 柔化平滑

系統 SHALL 對模型主體維持既有的 AA 灰階量化（`anti_aliasing_level`／`gray_level`）與 blur（`blur`）效果，使列印外觀品質相對現況不退化。模型的後處理行為 MUST 與現況一致（僅作用於模型軌，不受支撐像素污染）。

#### Scenario: 2D 預覽放大可見模型邊緣平滑
- **GIVEN** 使用者在 DS-online 前端開啟某切片層的 2D 預覽
- **WHEN** 使用者放大觀測模型主體輪廓
- **THEN** 模型邊緣呈現正常的 AA 灰階漸層與 blur 柔化
- **AND** 同一層的支撐輪廓呈現銳利的二值邊緣（無灰階漸層）

### Requirement: 接觸支撐處的邊界交織正確性

對於模型與支撐有接觸的切片層，系統 SHALL 保證支撐二值繪製只覆蓋支撐自身足跡，不得啃蝕（抹除或破壞）模型幾何。模型邊緣 MAY 因 blur 鄰域運算出現微弱染亮，但模型的整體幾何形狀 MUST 維持正確。

#### Scenario: 接觸層的模型幾何不被二值化破壞
- **GIVEN** 使用者載入支撐與模型接觸的物件並以雙 blob 切片
- **WHEN** 檢視接觸發生的切片層
- **THEN** 模型主體的輪廓與面積維持正確、未被支撐二值化啃蝕
- **AND** 支撐區域呈銳利二值、僅覆蓋其自身足跡
- **AND** 交界處允許微弱染亮，但不構成幾何錯誤

### Requirement: 通用光柵層介面擴充不破抽象

切片器通用光柵層 SHALL 以「帶預設值的非純虛擬」擴充 `RasterBase`（`draw_binary()` 預設等同 `draw()`、`apply_postprocess()` 預設 no-op），使 `SVGRaster` 等向量子類無需修改即繼承正確行為。SL1 專屬的 blur／量化公式 MUST 經 `RasterPostProcessor` callback 隔離於 `SL1.cpp`，通用 `AGGRaster` MUST NOT 內含任何機型特化公式。

#### Scenario: SVG 路徑零回歸
- **GIVEN** 使用者觸發 SL1_SVG 匯出
- **WHEN** 系統輸出向量切片
- **THEN** 輸出與現況一致（`SVGRaster` 繼承介面預設值），無回歸

#### Scenario: 後端與檔案格式不受影響
- **GIVEN** 本變更已套用
- **WHEN** 切片器輸出 SL1 並由後端轉為 prz
- **THEN** 後端 `prz_encoder.py` 維持無損 RLE 轉碼、未做任何修改
- **AND** SL1 與 prz 的檔案格式定義保持不變

### Requirement: 記憶體零增量

系統在雙軌差異化光柵下 SHALL NOT 引入額外的全解析度影像緩衝，亦不得開闢 ROI 暫存緩衝；支撐 MUST 由 AGG 就地二值繪製於模型所在的同一 `m_buf`。切片期間峰值記憶體 SHALL NOT 高於現況。

#### Scenario: 切大型支撐密集模型不發生記憶體炸裂
- **GIVEN** 使用者切一個支撐密集的大型模型（雙 blob）
- **WHEN** 切片並行進行
- **THEN** 切片期間峰值 RAM 不高於現況
- **AND** 切片正常完成，無因記憶體不足而失敗

