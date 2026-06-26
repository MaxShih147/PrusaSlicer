## 1. 通用光柵介面擴充與基礎類別修改（已完成）

- [x] 1.1 於 `src/libslic3r/SLA/RasterBase.hpp` 新增 `using RasterPostProcessor = std::function<void(void* ptr, size_t w, size_t h, size_t num_components)>;`（緊鄰既有 `RasterEncoder` 定義，保持對稱）
- [x] 1.2 於 `RasterBase` 類別新增兩個「非純虛擬（帶預設值）」：`virtual void draw_binary(const ExPolygon& poly) { draw(poly); }` 與 `virtual void apply_postprocess() {}`
- [x] 1.3 **即時驗證**：局部編譯確認 `SVGRaster` 未實作新方法仍可編譯通過（預設值未污染抽象、SVG 零波及）— Visual Studio 2022 編譯通過
- [x] 1.4 於 `AGGRaster`（`src/libslic3r/SLA/AGGRaster.hpp`）新增成員 `RasterPostProcessor m_postproc;` 與保存原始 gamma 的成員（`std::function<double(double)> m_gammafn`），於建構子初始化
- [x] 1.5 於 `AGGRaster` override `apply_postprocess()`：若 `m_postproc` 有效則就地呼叫於 `m_buf`（非 const、免複本）
- [x] 1.6 於 `AGGRaster` override `draw_binary()`：`m_rasterizer.gamma(agg::gamma_threshold(0.5))` → `_draw(poly)` → 還原 `m_rasterizer.gamma(m_gammafn)`（防禦性還原）
- [x] 1.7 於 `create_raster_grayscale_aa()`（`RasterBase.hpp` 宣告 + `RasterBase.cpp` 定義）新增尾端可選參數 `RasterPostProcessor pp = {}`，向下傳入建構鏈存入 `m_postproc`
- [x] 1.8 **即時驗證**：局部編譯確認新成員/override/簽章變更通過、既有不傳 `pp` 的呼叫端相容 — Visual Studio 2022 編譯通過

## 1.5 切片器 CLI：`--import-support-stl` 載入、tree_mesh 掛載與步驟閘門（新增）

> 範圍：僅支援單一 `SLAPrintObject`；`supports_enable` 與 `pad_enable` 維持 False（底筏已含於匯入的 support.stl）。

- [x] 1.5.1 於 `src/libslic3r/PrintConfig.cpp` 的 `CLIMiscConfigDef`（output 旁）新增 `import_support_stl`（coString，cli=`import-support-stl`）；`Setup.cpp`/`PrintHelp.cpp` 自動納入
- [x] 1.5.2 〔返工·null-deref 修正〕於 `src/libslic3r/SLAPrint.hpp` 的 `SLAPrintObject` 新增持久成員 `indexed_triangle_set m_imported_support_its` + `m_imported_support` 旗標 + `has_imported_support()`；`set_imported_support_mesh()`（SLAPrint.cpp）改為「套 `m_trafo` 後 `m_imported_support_its = std::move(m.its)`、設旗標」（**不再建立 `m_supportdata`**）；`SLAPrint::attach_imported_support()` 保留（單物件）
- [x] 1.5.3 〔返工·掛載點後移〕`src/CLI/ProcessActions.cpp` 維持「讀 `import_support_stl` → `TriangleMesh::ReadSTLFile`（已含 admesh 修復 + 共享頂點，無需另加網格修復）→ `attach_imported_support(its)`」；過早建立 `m_supportdata` 的程式碼原在 setter，已於 1.5.2 移除 → ProcessActions 無需改動
- [x] 1.5.4 〔返工·四閘門 + 延後建立 + 防禦〕`src/libslic3r/SLAPrintSteps.cpp`：`support_points()`/`generate_pad()` 於 `has_imported_support()` early-return（不變）；**`support_tree()` 於 `has_imported_support()` 時建立 `m_supportdata`（若 null）並 `tree_mesh = TriangleMesh{m_imported_support_its}` 後 return**（此步在三次 reset 之後 → 存活到 `slice_supports`）；`slice_supports()` 行 999 閘門含 `|| has_imported_support()`，**1036 行 `for` 迴圈已移入 `if(sd)` 內防 null-deref**
- [x] 1.5.5 失效觸發：本架構每次切片為**全新 CLI 程序**（SLAPrint 全新建、步驟皆 Fresh 必跑），換支撐檔自然重切 → CLI 單發路徑無 stale 問題，無需額外 invalidation 程式碼（僅 GUI 持久化才需，不在範圍）
- [x] 1.5.6 **即時驗證（Log 埋設）**：於 `slice_supports` 切片後加暫時性 `BOOST_LOG_TRIVIAL(info) << "[import-support] imported=… support_slices=… tree_tris=…"`（驗證完於 5.10 移除）
- [x] 1.5.7 **即時驗證**：編譯通過、CLI 實測切片完整跑完 100%（衝過 46% support tree / 55% slicing supports，無 silent crash）；不傳 `--import-support-stl` 行為與現況一致 — 使用者 VS2022 + CLI 驗證通過

## 2. PrintLayer 雙軌幾何分流（已完成；soSupport 來源改為匯入支撐）

> 注入點修正後，本階段消費端不變：soSupport 由 1.5 的匯入支撐切片填充，自動流入支撐軌。

- [x] 2.1 於 `src/libslic3r/SLAPrint.hpp` 的 `PrintLayer` 新增 `m_transformed_support_slices` + friend setter `transformed_support_slices()` + const getter（`m_transformed_slices` 改作 model 軌語義）
- [x] 2.2 於 `merged_input_to_slices()`：移除 1427 行 `union_ex(trslices)` 合併；model 軌存 `model_polygons`、support 軌存 `supports_polygons`，分別寫入兩 setter
- [x] 2.3 **即時驗證（Log 檢查軌道狀態）**：setter 寫入後加暫時性 Debug Log 印每層兩軌 `size()`（與 1.5.6 的 soSupport Log 串連核對）
- [x] 2.4 全域搜尋確認 `transformed_slices()` 唯一消費者為 `rasterize()` lvlfn；移除 union 後無其他消費者被破壞
- [x] 2.5 **即時驗證**：與 1.5.7 一併整體編譯通過，雙軌存取子與資料流重構落地（CLI 切片完整完成）

## 3. SL1 後處理 Lambda 封裝與 rasterize() 三段式渲染管線

- [x] 3.1 於 `src/libslic3r/Format/SL1.cpp` 將 AA 量化＋`stack_blur` body 抽成 `RasterPostProcessor` lambda：就地改 `ptr`、不再 copy `buf`、結尾不呼叫 `PNGRasterEncoder`；by-value 捕獲 `anti_aliasing_level/gray_level/blur_config`（量化以 `quant` 子 lambda 去重，算式不變）
- [x] 3.2 於 `SL1Archive::create_raster()` 依 `m_cfg` 建構 `pp` 並以新尾端參數注入 `create_raster_grayscale_aa(res,pxdim,gamma,tr,std::move(pp))`；AA-off 情境注入空 `pp`（gamma=0 即二值）
- [x] 3.3 將 `SL1Archive::get_encoder()` 退化為 `return sla::PNGRasterEncoder{};`
- [x] 3.4 **即時驗證（封裝隔離）**：grep 確認 `AGGRaster.hpp`/`RasterBase.*` 不含 `anti_aliasing_level`/`gray_level`/`stack_blur`/`init_val` 等機型特化字樣（僅餘通用 `gamma_threshold` 與描述性註解）
- [x] 3.5 於 `rasterize()` lvlfn 改為三段式：`for(model) raster.draw(p)` → `raster.apply_postprocess()` → `for(support) raster.draw_binary(p)`
- [x] 3.6 **即時驗證（整體編譯）**：建置 SLA 目標，三段式管線、雙軌、注入式後處理串接通過 — 使用者 VS2022 編譯通過
- [x] 3.7 **即時驗證（輸出對齊）**：CLI `--import-support-stl` 切片輸出 SL1，目視確認「模型邊緣有 AA、支撐純白二值無灰階」、無支撐輸出與現況一致 — 使用者目視確認正確
- [x] 3.8 `draw_binary` 只切 gamma LUT、未動 renderer 前景色（建構時 `White(255)`）→ 二值輸出 255；每層獨立 `create_raster()` 確保 gamma LUT 切換無跨緒污染（設計核驗）

## 4. 後端 web_slicer_core 雙 blob 接收與 CLI 動態組裝（新增）

- [x] 4.1 於 `web_slicer_core/agent/api_v2.py` 新增 `POST /slices/{job_id}/upload-support` 端點（存 `pending["support_stl"]`）；`execute_slice_job` 偵測到該 blob 時落地為 `job_dir/input/support.stl`（與 `model.stl` 同層）
- [x] 4.2 於 `web_slicer_core/agent/jobs.py` `run_slicing`：偵測 `input/support.stl` 存在時，先強制 `config.supports_enable=False` 與 `config.pad_enable=False`（寫進 INI），再 `cmd += ["--import-support-stl", str(import_support_file)]`（置於 `--load` 之後、`input_file` 之前）
- [x] 4.3 確認 `web_slicer_core/agent/prz_encoder.py` 維持零修改（無損 RLE 轉碼）— 未觸碰
- [x] 4.4 **即時驗證（後端）**：`py_compile` 通過；獨立腳本複刻 cmd 組裝三情境並斷言——(A) 帶支撐檔→強制 supports/pad=False、含 `--import-support-stl`、無 `--export-support-stl`；(B) 不帶+supports 開→無 import、有 `--export-support-stl`（向後相容）；(C) 不帶+supports 關→純模型同現況。ALL ASSERTIONS PASSED
- [x] 4.5 前端 `DS-online` 契約：切片上傳改送主體與支撐兩個獨立 blob（不在前端合併）— `MeshManager.exportModelsSplitSTL()` 拆成「模型（排除支撐）＋支撐（世界座標烘焙，合約 A）」兩 blob；`backendService.uploadSupport()` 對齊 `POST /slices/{job_id}/upload-support`；`slicingService.runBackendSlice/ensureSlicingJob` 上傳 model.stl 後若有支撐再上傳 support.stl；`sceneCoordinator` 導出 `exportModelsSplitSTL`。ESLint 四檔零新增告警（既有問題不動）

## 5. DS-online 前端端到端場景驗收（1:1 對齊 spec.md）

- [x] 5.1a 〔修正·Z 軸腰斬〕`src/libslic3r/SLAPrintSteps.cpp` `slice_model()`：於 `minZ = bb3d.min(Z) - get_elevation()` 之後、`minZf/minZs` 衍生之前，當 `has_imported_support() && !m_imported_support_its.empty()` 時 `minZ = std::min(minZ, bounding_box(m_imported_support_its).min(Z))`，把 Z 網格下緣鉗位到匯入支撐 bbox Z 底（見 design.md D8）。只下降不抬升、模型切片不受影響、不傳支撐檔逐位元一致；`slice_supports` 無須改動。已由使用者 VS2022 編譯 + CLI/前端實測（5.1）確認最底數層出現純支撐幾何、腰斬消除
- [x] 5.1 **雙 blob 傳遞 + 座標對齊（合約 A）**：DS-online 雙 blob 切片，確認切片器 `soSupport` 非空，且支撐與主體在切片平面正確貼合、無錯位 — 使用者實測正確
- [x] 5.2 **場景一・獨立支撐 RLE 優化邊緣**：支撐緊鄰背景為純黑 0、無灰階光暈，邊界為單一 0↔255 跳變 — 使用者實測正確
- [x] 5.3 **場景一・prz 體積**：對照現況（main）所切 prz，確認新版 prz 體積顯著縮小 — 使用者確認 prz 檔案變小
- [x] 5.4 **場景二・主體 AA 平滑**：2D 預覽放大，模型邊緣維持 AA 灰階＋blur、同層支撐銳利二值 — 使用者實測正確
- [x] 5.5 **場景三・接觸支撐邊界交織**：接觸層模型輪廓/面積未被支撐二值化啃蝕、支撐僅覆蓋自身足跡、交界僅微弱染亮 — 使用者確認通過
- [x] 5.6 **記憶體峰值零增量**：切大型支撐密集模型，峰值 RAM 不高於現況、無 OOM — 使用者確認記憶體未明顯增加
- [x] 5.7 **SVG 零回歸**：觸發 SL1_SVG 匯出，輸出與現況一致 — 使用者確認 sl1 正確
- [x] 5.8 **後端/格式不變**：`prz_encoder.py` 未改、SL1/prz 格式不變 — 使用者確認 prz 參數內容未被改動
- [x] 5.9 **向後相容**：不送支撐 blob 時，整條管線行為與現況一致 — 使用者確認一致
- [x] 5.10 全部通過後，移除 1.5.6 / 2.3 遺留的暫時性 Debug Log，整理收尾，準備 `/opsx:archive` — 已移除 `[import-support]`（ProcessActions.cpp、SLAPrintSteps.cpp:1051）與 `[dual-track]`（SLAPrintSteps.cpp:1469）三處暫時日誌；保留合法 warning 與永久 dual-track 註解
- [x] 5.11 〔SUGGESTION·收尾〕補強非阻擋性改善：(1) 新增 Catch2 回歸測試 `tests/sla_print/sla_import_support_tests.cpp`（鎖定 `attach_imported_support` 單物件契約＋空 print 守衛＋`has_imported_support()` 旗標翻轉，API 層級、無切片幾何 flakiness）並登錄於 `tests/sla_print/CMakeLists.txt`；(2) `web_slicer_core/agent/jobs.py` 補註 `--export-support-stl` 與 `--import-support-stl` 互斥關係（匯入時 `supports_enable` 已強制 False）；(3) 本任務 5.1a 措辭收尾為已驗證。待使用者編譯執行 `sla_print_tests "[sla_import_support]"` 綠燈