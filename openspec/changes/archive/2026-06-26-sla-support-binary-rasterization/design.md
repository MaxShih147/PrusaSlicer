## Context

SLA 影像管線在 `merged_input_to_slices()`（[SLAPrintSteps.cpp:1427](../../../src/libslic3r/SLAPrintSteps.cpp#L1427)）原以 `union_ex` 把模型與支撐合併成單軌，AA/blur 一律施加於整張影像，支撐 blur 後的背景灰階光暈破壞後端 RLE、無法縮小 prz。後端 `prz_encoder.py` 為無損轉碼器（PNG → 灰階 → 純 RLE），所有影像處理都在切片器 `get_encoder()`。

**核心架構事實（調查後修正）**：前端 `DS-online` 以 Three.js `STLExporter` 把「主體 + 支撐」串接成**單一 STL** 上傳，後端 `jobs.py:run_slicing`（[jobs.py:130](../../../../../web_slicer_core/agent/jobs.py)）以 `supports_enable=False` 餵單檔給切片器 → `soModel` 含全部幾何、`soSupport` 為空。原「切片器內從 origin 分流」假設破產。

**已落地成果（保留沿用，消費端不變）**：
- 階段一：`RasterBase` 新增 `RasterPostProcessor` 型別與 `draw_binary()`/`apply_postprocess()` 非純虛擬（帶預設值）；`AGGRaster` 儲存並就地執行注入的後處理 functor、`draw_binary` 以 `agg::gamma_threshold` 二值繪製；`create_raster_grayscale_aa` 新增尾端可選 `RasterPostProcessor`。（VS2022 編譯通過）
- 階段二：`PrintLayer` 雙軌（`m_transformed_slices` = model、`m_transformed_support_slices` = support）；`merged_input_to_slices()` 取消 union、各歸各軌，從 `soModel`/`soSupport` 分流。

soSupport 既有產生鏈（注入點關鍵）：
```
slaposSupportTree → support_tree(po): create_support_tree() 產 m_supportdata->tree_mesh
slaposSliceSupports → slice_supports(po): sla::slice(tree_mesh.its, pad_mesh.its,…) → support_slices [SLAPrintSteps.cpp:1011]
get_all_polygons(record, soSupport) 讀 support_slices [SLAPrintSteps.cpp:209]
merged_input_to_slices()（階段二）讀 soSupport → m_transformed_support_slices [SLAPrintSteps.cpp:1431]
```
SLA CLI 選項定義於 `cli_*_config_def`（[PrintConfig.cpp:6010 區](../../../src/libslic3r/PrintConfig.cpp#L6010)，如既有 `export_support_stl`），由 [Setup.cpp](../../../src/CLI/Setup.cpp) 自動納入、[ProcessActions.cpp](../../../src/CLI/ProcessActions.cpp) 消費。

## Goals / Non-Goals

**Goals:**
- 支撐幾何以**獨立 STL** 跨服務（前端雙 blob → 後端 `--import-support-stl` → 切片器）傳入，於切片器復原 `soSupport` 軌。
- 切片器雙軌差異化光柵：model 走 AA + 後處理，support 走二值（豁免 AA/blur），背景零光暈。
- 通用光柵層介面擴充不破抽象（SVG 零波及）、SL1 配方隔離於 `SL1.cpp`。
- 記憶體零增量（support 經 AGG 就地二值畫入同一 `m_buf`）。
- 後端 `prz_encoder.py` 與 SL1/prz 格式零修改。

**Non-Goals:**
- 切片器自生支撐樹（改為匯入；`supports_enable` 在此流程維持 False）。
- 後端影像處理、SL1 PNG 之外的格式、preset/profile、cache。
- SVG 路徑差異化（靠介面預設值自動退化為正確向量行為）。
- 合約 B（model 與 support 各自獨立位移需傳變換矩陣）——已定調合約 A，不需要。

## Decisions

### D1. 跨服務幾何傳遞合約（合約 A：共用世界座標）

前端 `DS-online` 切片時上傳兩個 blob：`model.stl` 與 `support.stl`，兩者**共用同一世界座標原點**（`STLExporter` 既有特性，不需額外傳變換矩陣）。後端落地為 `input/model.stl` 與 `input/support.stl`，切片器把支撐掛在**同一 `SLAPrintObject`**、套用相同物件 `trafo` → 自然對齊。

**理由**：消除座標複雜度；DS-online 本就以 model 與 support 為共座標的獨立 mesh（`generate_supports` 回傳的 `model_support.stl` 即與 model 共座標）。
**替代（捨棄）**：合約 B（攜帶 4×4 變換矩陣）→ 跨服務序列化複雜、易錯，無必要。

### D2. 切片器 CLI：`--import-support-stl <path>`

- **選項定義**：於 `cli_*_config_def`（PrintConfig.cpp）新增 `import_support_stl`（`coString`，路徑）。`Setup.cpp` 自動納入、`PrintHelp.cpp` 自動列出，無需改 parser。
- **載入與「持久化暫存」**（ProcessActions.cpp，`print->process()` **之前**）：讀 STL → `TriangleMesh::ReadSTLFile`（預設 `repair=true`，已含 admesh 修復 + 共享頂點，無需另加網格修復）→ 呼叫 `SLAPrint::attach_imported_support(its)`；後者把幾何存進 `SLAPrintObject` 的**持久成員 `m_imported_support_its`**（套 `m_trafo` 對齊，合約 A）並設 `m_imported_support` 旗標。**此處不建立 `m_supportdata`**。
- **延後掛載**（`support_tree()`，見 D3）：真正把 `tree_mesh` 灌入 `m_supportdata` 的動作延到 `support_tree()` 內執行。

**理由**：`--import-support-stl` 為字串輸入選項，與既有 `export_support_stl` 對稱。**`m_imported_support_its` 必須是獨立持久成員、而非塞進 `m_supportdata`**，因為 `m_supportdata` 會被 upstream 步驟清空（見 D3 的 reset 病灶）。

### D3. 延後掛載至 `support_tree()`（修正：原「ProcessActions 掛 `m_supportdata->tree_mesh`」會被 reset 清空 → null-deref）

匯入的是 **3D 網格**，必須先被切片成每層 ExPolygons 才成為 `soSupport`；注入目標仍是 `m_supportdata->tree_mesh`，由既有 `slice_supports` 切成 `support_slices` → `soSupport` → **自動流進階段二的 `m_transformed_support_slices`**（消費端零改動）。

**CLI 實測病灶（null-dereference）**：原設計在 `ProcessActions`（`process()` 前）就建 `m_supportdata` 並掛 `tree_mesh`，但 `process()` 早期的三個步驟都會 `po.m_supportdata.reset()`：`mesh_assembly()`（0%，[SLAPrintSteps.cpp:467](../../../src/libslic3r/SLAPrintSteps.cpp#L467)）、`hollow_model()`（9%，[480](../../../src/libslic3r/SLAPrintSteps.cpp#L480)）、`drill_holes()`（18%，[532](../../../src/libslic3r/SLAPrintSteps.cpp#L532)）。於是到 `slice_supports()`（55%）時 `m_supportdata` 為 null，但 `m_imported_support` 旗標放行閘門，衝進 [SLAPrintSteps.cpp:1036](../../../src/libslic3r/SLAPrintSteps.cpp#L1036) 的 `sd->support_slices` → **解參考 null `unique_ptr` → silent segfault**（`if(sd)` 區塊連同 debug log 一起被跳過，故無任何輸出）。

**修正（掛載時機後移到 reset 之後）**：
```cpp
// support_tree(po)：此步在 0/9/18% 三次 reset 之後（46%）才執行，建立的 m_supportdata 能存活到 slice_supports
if (po.has_imported_support()) {
    if (!po.m_supportdata)
        po.m_supportdata = std::make_unique<SupportData>(TriangleMesh{po.m_imported_support_its});
    po.m_supportdata->tree_mesh = TriangleMesh{po.m_imported_support_its};
    return;                      // 仍跳過 create_support_tree
}
```
- `m_imported_support_its` 為 `SLAPrintObject` 持久成員（`indexed_triangle_set`，已套 `m_trafo`），**不受三次 reset 影響**。
- **防禦**：`slice_supports()` 第 1036 行的 `for` 迴圈移入 `if(sd)` 區塊內，杜絕未來任何「sd 為 null 卻進迴圈」的空指標。

**理由**：`m_transformed_support_slices` 是末端 2D 結果不能塞 3D 網格；在 reset 之後（`support_tree`）才建 `m_supportdata` 是讓匯入網格存活到切片的唯一正確時機。
**替代（捨棄）**：把支撐當第二個 `ModelObject` 整體切片 → 與單物件 archive 假設衝突、侵入大；在 ProcessActions 掛載 → 被 reset 清空（即上述病灶）。

### D4. 支撐管線步驟閘門調整（匯入旗標驅動）

| 步驟 | 既有行為 | 匯入時行為 |
|---|---|---|
| `slice_model()` | `minZ = bb3d.min(Z) - get_elevation()`（`bb3d` 僅含模型正體；本流程 elevation=0 → 下緣卡模型底） | **把 Z 網格下緣鉗位到匯入支撐 bbox Z 底**：`minZ = std::min(minZ, bounding_box(m_imported_support_its).min(Z))`（見 D8）。否則模型底以下的支撐（底筏/貼床面）無 `PrintLayer` 被腰斬 |
| `support_points()` | 偵測支撐點 | **跳過**（不需點） |
| `support_tree()` | `create_support_tree()` 生成樹覆寫 `tree_mesh` | **建立 `m_supportdata` 並把 `m_imported_support_its` 灌入 `tree_mesh`，再 return**（見 D3；此步在三次 reset 之後，故能存活）；**不**呼叫 `create_support_tree()` |
| `generate_pad()` | `pad_enable` 為真時對 `tree_mesh` 生 pad，失敗丟 `No pad can be generated` | **跳過**（底筏已含於匯入 support.stl；否則狀態機因 `slice_supports` 閘門逆向激活此步而報錯） |
| `slice_supports()` | 行 999 `if(!supports_enable && !pad_enable) return;` | 閘門改為 `(supports_enable \|\| imported \|\| pad_enable)` → 切 `tree_mesh`；**1036 行 `for` 迴圈移入 `if(sd)` 內（防 null-deref）** |

**理由**：以單一「匯入支撐」旗標驅動**五處** `if`，不發明新管線；`supports_enable`/`pad_enable` 維持 False 確保切片器不自生支撐與底筏。
**CLI 實測教訓（state machine 依賴鏈）**：`slice_supports` 閘門放行後，PrusaSlicer 狀態機會逆向把 upstream 的 `support_points→support_tree→generate_pad` 全部激活；**四步皆須 early-return**，漏掉 `generate_pad` 會在 55% 因原生 pad 驗證失敗中斷。
**風險**：步驟相依與 PrintBase 狀態機的失效觸發須確認旗標納入 invalidation（實作期驗證）。

### D5. 切片器雙軌差異化光柵（沿用階段一/二，補階段三）

- `merged_input_to_slices()`：取消 union、model→model 軌、support→support 軌（**已落地**）。
- `rasterize()` lvlfn 三段式（階段三待落地）：
  ```
  for (model)   raster.draw(p);            // AA gamma
  raster.apply_postprocess();              // SL1 blur+量化（注入 functor）；SVG no-op
  for (support) raster.draw_binary(p);     // threshold 二值；SVG 退回 draw
  ```
- SL1 配方：現 `get_encoder()` 的 AA 量化＋`stack_blur` body 包成 `RasterPostProcessor`（就地、免複本、不 PNG）於 `SL1Archive::create_raster()` 注入；`get_encoder()` 退化為 `PNGRasterEncoder{}`。

**理由**：support 經 AGG 就地二值畫入同一 `m_buf` → 背景零光暈、零額外全幀 buffer；通用 AGG 不含 SL1 公式。

### D6. 後端調用鏈（web_slicer_core）

```
api_v2 execute_slice_job：models[0] → input/model.stl；若有 support blob → input/support.stl
jobs.py run_slicing：
   cmd = prusa --export-sla … input/model.stl
   if (input/support.stl 存在): cmd += ["--import-support-stl", <path>]
   supports_enable 維持 False（不自生）
```
`prz_encoder.py` 零修改。

### D7. 執行緒安全（沿用階段一/二結論）

`draw_layers` 對每層各自 `create_raster()`，每個 `AGGRaster` 實例（含 `m_buf`/`m_rasterizer` LUT/`m_postproc`）僅被單緒單層存取，`draw_binary` 的 gamma LUT 切換結構上不可能 race，不需 `thread_local`。匯入支撐的載入發生在 `process()` 之前（單緒），不涉並行。

### D8. Z 軸切片網格下緣鉗位至匯入支撐（修正 5.1 幾何腰斬）

**病灶**：`slice_model()`（[SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp) slaposObjectSlice，步驟 4）以 `bb3d = csgmesh_positive_bb(po.m_mesh_to_slice)` 建立 Z 網格 `m_slice_index`，`bb3d` **僅含模型正體**、不含匯入支撐。下緣 `minZ = bb3d.min(Z) - get_elevation()`；本流程 `supports_enable=False`/`pad_enable=False` → `get_elevation()=0` → 下緣卡在模型底。`slice_supports()` 消費整條 `m_slice_index` 高度切 `tree_mesh`，故**模型底以下的支撐（底筏/貼床面）無對應 `PrintLayer` 被腰斬**（端到端 5.1 回報）。

**修正**：於 `minZ` 計算後、`minZf/minZs` 衍生前插入鉗位：
```cpp
double minZ = bb3d.min(Z) - po.get_elevation();
if (po.has_imported_support() && !po.m_imported_support_its.empty()) {
    const BoundingBoxf3 sbb = bounding_box(po.m_imported_support_its); // TriangleMesh.hpp:395
    minZ = std::min(minZ, sbb.min(Z));
}
```

**正確性**：
- **同座標系**：`m_mesh_to_slice` 由 `model_to_csgmesh(*model_object(), po.trafo(), …)` 建立；`m_imported_support_its` 由 `set_imported_support_mesh()` 套 `m.transform(m_trafo)`；`trafo()==m_trafo` → 直接取 Z 最小值幾何正確。
- **只下降不抬升**：`std::min` 確保支撐完全落在模型高度內時 `minZ` 不變 → 零回歸；不傳 `--import-support-stl` 時 `has_imported_support()=false`，分支不進入 → 與現況逐位元一致。
- **模型切片不受影響**：`m_model_height_levels` 由 `closest_slice_record(m_slice_index, bb3d.min(Z))` 從模型底錨定；本修正僅在 `m_slice_index` 前面補上更低層記錄，這些低層只被支撐切片消費。
- **符合原始設計意圖**：[SLAPrintSteps.cpp:618-624](../../../src/libslic3r/SLAPrintSteps.cpp#L618) 註解明言「較厚的第一層通常屬於支撐而非模型」；鉗位後第一層落在支撐底（貼床面），正是正常有支撐列印的應有行為。

**替代（捨棄）**：硬寫 `minZ = std::min(minZ, 0.0)`（強制貼床 Z=0）→ 假設 trafo 後貼床面恰在 Z=0，較脆弱；採支撐 bbox 最小 Z 忠於匯入幾何、不依賴該假設。
**範圍**：`slice_supports` 無須改動（本就消費整條 `m_slice_index`，下緣下降後支撐高度自動補齊）。

## Risks / Trade-offs

- **座標未對齊** → 合約 A + 掛同一 `SLAPrintObject` 套同 trafo；以「支撐與主體切片平面貼合」手動驗證。
- **`support_tree` 覆寫匯入網格** → D4 旗標跳過 `create_support_tree`；回歸測試確認 `tree_mesh` 為匯入內容。
- **PrintBase 失效觸發未納入匯入路徑** → 將匯入旗標/檔路徑納入 `apply()` invalidation，避免改檔不重切。
- **階段間中間態**：階段二後、階段三前，rasterize 暫只畫 model 軌（支撐未繪）；屬分階段正常中間態，編譯通過、輸出待階段三補齊。
- **模型自身 blur 光暈仍在** → 屬預期；prz 縮小來自支撐不再貢獻光暈。
- **接觸支撐邊界微弱染亮** → 可接受（proposal 定調）；模型幾何不被二值化啃蝕，納入回歸。

## Migration Plan

- **跨服務部署順序**：① 切片器先上（新增 `--import-support-stl`，向後相容：不傳則行為不變）→ ② 後端後上（偵測雙 blob 才加參數）→ ③ 前端最後切換（改送兩 blob）。任一階段未到位皆向後相容（切片器無支撐檔即等同舊行為）。
- 無 SL1/prz 檔案格式、preset、cache 變更。
- Rollback：前端恢復合併上傳即回舊行為；切片器/後端新增路徑在無支撐檔時不啟用。

### 與 master 待合併 commit 的相容性（rebase 備忘）

**實證結論（`git merge-tree` 乾跑，base=`6ec3f15f2`）**：對 `origin/master` 領先的 4 個 commit 做不碰工作區的三方乾跑合併，**衝突標記 `<<<<<<<` = 0，Git 全自動合併**。原本「ProcessActions.cpp 可能文字衝突需人工合併」的警告**已被推翻**。

master 領先 4 commit 與本變更的逐檔關係：

- `2e8d715`（支撐面數 45→16，`SupportTreeBuilder.hpp`/`SupportTreeMesher.hpp`）：本變更未碰，零交集；且只影響自生支撐生成，與匯入路徑正交。
- `7380ea1`（support-only fast path，`ProcessActions.cpp`）：與本變更同檔但 **hunk 不重疊**——本變更插入點在 base 行 441–446（`--import-support-stl` 載入），master 在 base 行 460+（fast-path），**中間隔 13 行未動 → Git 各自落地、零衝突**。語意亦正交：fast-path 條件需 `export_support_stl` 且**無** `export_sla/slice/preview`，而本流程跑完整切片（有 export_sla/preview）→ fast-path 永不觸發匯入路徑。
- `87d0b422a`（pinhead 優化器，`DefaultSupportTree.cpp`）：本變更未碰；HEAD 既有角度功能與其 `merge-tree` 乾淨（0 標記）。只影響自生支撐。
- `0b95ae76f`（drill-holes 預覽重用，`SLAPrintSteps.cpp`）：與本變更同檔但落在 base 行 260，恰在本變更 hunk（214–219 與 584+）之間的空隙 → 零重疊，`merge-tree` 乾淨。與匯入掛載／`m_supportdata` reset 鏈正交。

**唯一實務前置**：合併前工作區須乾淨（本功能須先 commit/stash），否則 `git merge` 會因 `ProcessActions.cpp`/`SLAPrintSteps.cpp` 本機未提交變更而拒絕。合併後建議重編譯 + 重跑 `--import-support-stl` 煙霧測試（master 改了相鄰 SLA 程式碼，雖無衝突仍宜端到端複驗一次）。

## Open Questions

- 多物件情境：`--import-support-stl` 對多個 `SLAPrintObject` 的對應規則（單物件先行；多物件需定義 support 檔與 object 的配對，或限定單物件）。
- `support_tree` 跳過後，pad（`create_pad` 依賴 `tree_mesh`）是否仍正確以匯入網格為基底——確認 pad_enable 與匯入併用的行為。
- 匯入旗標納入 PrintBase invalidation 的精確觸發點。