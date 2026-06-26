## Why

支撐被 SLA 影像管線的 blur/AA 柔化後，會在背景留下一圈灰階光暈（halo），破壞後端 `web_slicer_core/agent/prz_encoder.py` 的 row-major RLE 連續性，使 prz 檔案無法有效壓縮；同時支撐尖端與接觸點固化面積縮小、貼床附著力下降，造成支撐脫落與列印失敗。支撐需要的是**實心、銳利、滿曝光**，而非外觀平滑。

**關鍵架構事實（調查後修正）**：原始假設「切片器內可從 `soModel`/`soSupport` 取得雙軌幾何」**已破產**。實測確認：前端 `DS-online` 在上傳前已用 Three.js `STLExporter` 把「主體 + 支撐」**幾何串接成單一 STL**，後端 `jobs.py:run_slicing` 以 `supports_enable=False` 餵給切片器 → 切片器讀到的 `soModel` 含全部幾何、`soSupport` 為空。純切片器內部重構無法分流支撐。

因此本變更從「純切片器內部重構」**升級為跨儲存庫的全棧分軌合約**：前端不再合併、改傳兩個獨立幾何；支撐 STL 跨服務穿透至切片器，於切片器內復原 `soSupport` 軌，再施以「支撐豁免 blur/AA」的差異化光柵。

## What Changes

- **前端（DS-online）契約變更**：切片時不再把主體與支撐合併成單一 STL，改為**上傳兩個 blob**（`model.stl` + `support.stl`），兩者共用同一世界座標原點（合約 A，`STLExporter` 既有特性）。
- **後端（web_slicer_core）調用鏈擴充**：`api_v2.py` 接收並落地第二個 blob 為 `input/support.stl`；`jobs.py:run_slicing` 偵測到支撐檔時，動態組裝新 CLI 參數 `--import-support-stl <path>`。**後端不做任何影像處理**（`prz_encoder.py` 維持無損 RLE 轉碼，零修改）。
- **切片器（prusaslicer_fork）新增 CLI `--import-support-stl <path>`**：讀入獨立支撐 STL → `TriangleMesh`，掛在對應 `SLAPrintObject` 的 `m_supportdata->tree_mesh`，套用與 model 相同的物件 `trafo` 自然對齊（合約 A）。新增旗標令 `support_points`/`support_tree` 步驟跳過自生、`slice_supports` 跨越 `supports_enable` 閘門，將匯入網格切成 `support_slices` → `soSupport`。
- **切片器雙軌差異化光柵（沿用既有成果）**：`PrintLayer` 維持 model/support 雙軌；`merged_input_to_slices()` 取消 `union_ex`、各歸各軌（**階段一介面、階段二雙軌結構已落地，消費端不變，原樣保留**）。`rasterize()` 三段式：model 走 AA + 後處理 → support 走 `draw_binary` 二值（豁免 AA/blur）。
- **通用光柵層介面擴充（已落地）**：`RasterBase` 新增非純虛擬 `draw_binary()` / `apply_postprocess()`（帶預設值，SVG 零波及）；SL1 blur/量化配方經 `RasterPostProcessor` callback 隔離於 `SL1.cpp`。
- **功能定性**：Always-on（支撐區恆豁免 blur/AA）；背景零光暈使 prz 顯著縮小；記憶體零增量（support 經 AGG 就地二值畫入同一 `m_buf`）。

## Capabilities

### New Capabilities
- `sla-support-binary-rasterization`: 跨服務分軌——支撐幾何以獨立 STL 經前端雙 blob → 後端 `--import-support-stl` → 切片器 `tree_mesh` → `slice_supports` 復原 `soSupport`；切片器雙軌差異化光柵讓支撐豁免 AA/blur、以二值就地合成，根除背景灰階光暈以縮小 prz；定義通用光柵層 `draw_binary()`/`apply_postprocess()` 非純虛擬介面與後處理 callback 注入契約，及 `rasterize()` 三段式渲染序列。

### Modified Capabilities
<!-- 無既有 openspec/specs 規範變更。 -->

## Impact

**切片器（prusaslicer_fork）**
- [src/libslic3r/PrintConfig.cpp](../../../src/libslic3r/PrintConfig.cpp)：CLI config def 區新增 `import_support_stl`（coString，路徑）。
- [src/CLI/ProcessActions.cpp](../../../src/CLI/ProcessActions.cpp)：`print->process()` 前載入支撐 STL → `TriangleMesh`、套物件 trafo、掛上 `SLAPrintObject::m_supportdata->tree_mesh` 並設匯入旗標。
- [src/libslic3r/SLAPrint.hpp](../../../src/libslic3r/SLAPrint.hpp)：`SLAPrintObject` 新增「匯入支撐」旗標存取子；`PrintLayer` 雙軌（**已落地**）。
- [src/libslic3r/SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp)：`support_points`/`support_tree` 於匯入時跳過自生；`slice_supports` 行 999 閘門改為 `(supports_enable || imported)`；`merged_input_to_slices()` 雙軌（**已落地**）；`rasterize()` 三段式（待階段三）。
- [src/libslic3r/SLA/RasterBase.hpp](../../../src/libslic3r/SLA/RasterBase.hpp) / [AGGRaster.hpp](../../../src/libslic3r/SLA/AGGRaster.hpp) / [RasterBase.cpp](../../../src/libslic3r/SLA/RasterBase.cpp)：`RasterPostProcessor` + `draw_binary()`/`apply_postprocess()`（**已落地**）。
- [src/libslic3r/Format/SL1.cpp](../../../src/libslic3r/Format/SL1.cpp)：blur/量化 body 包成 `RasterPostProcessor` 注入；`get_encoder()` 退化為 `PNGRasterEncoder{}`（待階段三）。

**後端（web_slicer_core）**
- `agent/api_v2.py`：slice 任務接收第二 blob，落地為 `input/support.stl`。
- `agent/jobs.py`：`run_slicing` 偵測 `input/support.stl` 存在時，`cmd += ["--import-support-stl", <path>]`；`supports_enable` 維持 False（不自生）。
- **不影響**：`agent/prz_encoder.py`（無損 RLE 轉碼不變）、SL1/prz 檔案格式。

**前端（DS-online）**
- 切片上傳流程：改為送主體與支撐兩個 blob（不在前端合併）。

**白賺效益**：Preview（`PNGPreviewEncoder`）於後處理後 encode，自動繼承最終合成（主體平滑、支撐銳利）。

## Validation（核心驗證紀律：以 DS-online 前端手動驗證為主）

1. **prz 體積驗證**：DS-online 載入含獨立支撐物件 → 切片（雙 blob）→ 下載 prz，對照現況應顯著縮小。
2. **Preview 觀感**：2D Preview 放大，主體邊緣平滑、支撐銳利二值無灰階。
3. **跨服務分軌生效**：確認切片器 `soSupport` 非空（匯入支撐成功切片），支撐區呈二值。
4. **座標對齊（合約 A）**：支撐與主體在切片平面正確貼合、無錯位。
5. **記憶體不退化**：切支撐密集大模型，峰值 RAM 不高於現況。
6. **接觸支撐回歸**：模型邊緣未被支撐二值化啃蝕。
7. **後端/格式/ SVG 零回歸**：`prz_encoder.py` 未改、SL1/prz 格式不變、SVG 匯出無回歸。