> **Repo 標示**：未特別註明者皆屬本 repo（`D:\repos\web_slicer_core\third_party\prusaslicer_fork`）。
> 標記 **【父 repo】** 者屬 `D:\repos\web_slicer_core`。
>
> **驗證紀律**：每個小階段結束前都必須跑完該階段的驗證任務並看到預期結果，才可進入下一階段。任何驗證失敗必須就地修正，不得累積到後面一次處理。

## 1. 基準鎖定（動任何程式碼之前）

- [x] 1.1 在乾淨工作區建置目前版本的 CLI，記錄二進位路徑與 commit hash，作為 before 基準
- [x] 1.2 以 3DBenchy（Y 軸 -40°）+ Critical Angle 90° + `pad_enable=1` 執行 `--export-support-stl`，保存 stdout / stderr / exit code，確認重現 `No pad can be generated for this model with the current configuration`
- [x] 1.3 同模型改 `pad_enable=0` 重跑，保存輸出，確認得到 `No support/pad mesh generated`（這是降級後要收斂到的目標輸出）
- [x] 1.4 以同一模型分別用 Critical Angle 30° / 45° / 90°（`pad_enable=0`）產生 support STL，保存檔案並記錄**三角形總數、檔案大小與 canonical digest**（raw SHA-256 僅存查、不作判準），作為「支撐密度不得改變」的回歸基準。判準為分散式容差：重複 ≥ 5 次，基準值須落在觀測集合內，或觀測極差 < 5%
- [x] 1.5 **驗證**：確認 1.2 的 stderr 與 1.3 的 stdout 內容與 spec `sla-pad-generation` 所述一致；1.4 的三份 hash 已存檔備查

> 基準資料：`evidence/baseline-20260730T063825Z/`（`BASELINE.md` 為索引）。
> **調查結果：引擎輸出非決定性**——以 30° 取樣時三角形數 5/5 穩定，raw SHA-256 每次不同，排序後 canonical digest 4/5 相同（差異 128/47,936 = 0.267%）。故 SHA-256 逐一比對判準不成立。
>
> ⚠️ 本節當初推測「三角形數 + 檔案大小完全相同」可作主判準，**已於 Task 7 被推翻**：45° 的三角形數本身就會變動，且固定 `SLA_GA_MAX_ITER` 亦無法消除。最終判準見 spec `sla-overhang-threshold-semantics`「既有支撐密度不得改變」——**分散式容差**（重複 ≥ 5 次，基準值須落在觀測集合內或極差 < 5%）。

## 2. C++ 降級邏輯（單一行為變更點）

- [x] 2.1 在 `src/libslic3r/SLAPrintSteps.cpp` 的 `SLAPrint::Steps::generate_pad()` 中，於 `validate_pad()` 失敗分支前取得判斷所需狀態：pad mesh 是否為空、`po.m_config.supports_enable.getBool()`、支撐 mesh 是否為空
- [x] 2.2 實作三條件合取的降級判準（design 決策 2）：三者同時成立時清空 pad mesh 並讓步驟正常完成，不拋例外；其餘情況維持 `throw Slic3r::SlicingError(...)`
- [x] 2.3 降級路徑加上 `BOOST_LOG_TRIVIAL(warning)`，說明因無支撐柱而未生成 pad（spec：降級留下可追溯的診斷紀錄）
- [x] 2.4 確認 `has_imported_support()` 早退仍位於本判斷之前，未被改動
- [x] 2.5 **驗證**：`cmake --build <build-dir> --target libslic3r` 編譯通過，無新增警告（使用者於 Visual Studio 2022 確認；Task 5.6 另以 `cmake --build build --target libslic3r --config Release` 複驗通過）

> 2.1 實作說明：改用直接讀取 `po.m_supportdata->tree_mesh.empty()`，未採用 `po.support_mesh()`。後者內含 `is_step_done(slaposSupportTree) && supports_enable` 守門條件，語意與本處判準耦合；直接讀取成員與本檔案既有寫法（1023/1028/1061 行）一致，意圖也更明確。

## 3. Catch2 單元測試（覆蓋分流兩支）

- [x] 3.1 新增 `tests/sla_print/sla_pad_degradation_tests.cpp`，並在 `tests/sla_print/CMakeLists.txt` 的來源清單中註冊（比照既有的 `sla_import_support_tests.cpp`）
- [x] 3.2 **驗證**：`cmake --build <build-dir> --target sla_print_tests` 建置成功，空檔案能被連結
- [x] 3.3 撰寫降級案例：以 `20mm_cube.obj` **繞 Y 軸旋轉 40°**（平底原型會生支撐，見下方修正）設定 `supports_enable=1` / `pad_enable=1` 跑完 `process()`，斷言不拋例外、pad mesh 為空、`slaposPad` 標記為 done
- [x] 3.4 **驗證**：`sla_print_tests "[sla_pad_degradation]"` 該案例通過
- [~] 3.5 撰寫 fail-closed 案例（一）：support mesh 非空但 pad 判定無效時仍拋 `SlicingError` —— **無法建構，改以 `validate_pad()` 述詞測試覆蓋入口條件**（見下方說明）
- [x] 3.6 撰寫 fail-closed 案例（二）：`supports_enable=0` / `pad_enable=1` 且 pad 無效時仍拋 `SlicingError`（另補一支平底模型變體）
- [x] 3.7 **驗證**：`sla_print_tests "[sla_pad_degradation]"` 全數通過；`sla_print_tests` 全量執行無回歸

> **3.2 / 3.4 / 3.7 執行結果**（2026-07-30，於 `build/tests/sla_print/Release/`，以 `-DSLIC3R_BUILD_TESTS=ON` 重新配置後建置）：
> - `sla_print_tests "[sla_pad_degradation]"` → **All tests passed (19 assertions in 6 test cases)**
> - `sla_print_tests.exe`（全量）→ **All tests passed (10851 assertions in 51 test cases)**，無回歸
>
> **3.3 前提修正（已用 CLI 實測）**：`20mm_cube.obj` **不是**零支撐模型。其底面法線為 (0,0,-1)、`polar == π`，即使閾值拉到 90° 仍通過過濾而放置支撐頭（實測 `(includes supports and pad)`）。繞 Y 軸旋轉 40° 後不存在完全水平朝下的面，支撐樹才為空（實測 `No support/pad mesh generated`），與 3DBenchy -40° 的成因相同。測試檔已改用旋轉後的方塊，平底原型則轉為「正常路徑未被吞掉」的 fixture。
>
> **3.5 無法建構的理由**：pad 的 footprint 來自支撐樹，支撐樹非空時 `sup_contours` 必然非空、pad 必然非空，`validate_pad()` 不會失敗；而會讓 pad config 本身無效的參數，`SLAPrint::validate()` 會在 `process()` 之前先擋下。故「支撐非空但 pad 無效」在現有 fixture 下不可達。改以直接測試 `validate_pad()` 述詞（空 pad → false、非空 pad → true、embed_object 模式 → true）鎖住降級分支的**入口條件**，確保「pad 為空」這個條件由述詞結構性保證。理論上可構造的途徑（所有支撐柱都落在模型上、無一觸及 pad 取樣帶）留待另案。
>
> **已完成的測試案例**（tag `[sla_pad_degradation]`，共 6 支）：
> 1. 零支撐 + pad ON → 不拋例外、兩個 step done、support/pad mesh 皆空
> 2. 零支撐 + pad OFF → 不拋例外（降級要收斂到的對照組）
> 3. supports OFF + pad ON（旋轉）→ 拋 `SlicingError`
> 4. supports OFF + pad ON（平底）→ 拋 `SlicingError`
> 5. 平底 + supports ON + pad ON → support/pad mesh 皆非空（正常路徑未被吞掉）
> 6. `validate_pad()` 述詞三種輸入

## 4. CLI 實機驗證（降級路徑端到端）

- [x] 4.1 重建 CLI 執行檔
- [x] 4.2 以 1.2 的完全相同參數重跑（小船 -40° + 90° + `pad_enable=1`）
- [x] 4.3 **驗證**：exit code 為 0；stdout 含 `No support/pad mesh generated`；stderr **不含** `No pad can be generated`；未產生 `*_support.stl`
- [x] 4.4 以 1.3 的參數重跑（`pad_enable=0`），**驗證**輸出與 1.3 的基準逐字相同（無回歸）
- [x] 4.5 以會長出支撐的模型 + `pad_enable=1` 執行，**驗證** stdout 含 `(includes supports and pad)` 且 `*_support.stl` 落地（正常路徑未被吞掉）
- [x] 4.6 重跑 1.4 的三組 Critical Angle，比對支撐幾何。**分散式容差判準**：每組重複執行 ≥ 5 次，基準值須落在觀測集合內，或觀測極差 < 5%；次判準為 `stl_diff.py` 差異三角形比例 < 5%

> 證據：`evidence/after-20260730T073638Z/`（`AFTER.md` 為索引）。受測二進位 `38a4b100…`（基準為 `f794ea7e…`）。
>
> | Run | pad | angle | exit 前→後 | 三角形數 前→後 | 判定 |
> |---|---|---|---|---|---|
> | A | 1 | 90° | **1 → 0** | — | ✅ 主目標：改印 `No support/pad mesh generated`、stderr 空 |
> | B | 0 | 90° | 0 → 0 | — | ✅ stdout 逐字相同 |
> | C | 0 | 30° | 0 → 0 | 47,936 → 47,936 | ✅ 相同 |
> | D | 0 | 45° | 0 → 0 | 46,528 → 44,992 | ✅ 歸因為 GA 變異，見下 |
> | E | 1 | 45° | 0 → 0 | 44,028 → 44,028 | ✅ 相同，`(includes supports and pad)` |
>
> **D 組差異歸因（三項證據排除本次修改）**：① `pad_enable=0` 時控制流走 `else if` 分支，新增程式碼一行都不執行，支撐網格由未修改的 `support_tree` 步驟產生；② 以**修改後**的二進位在 `SLA_GA_MAX_ITER=20` 下**能夠產出**基準值 46,528，即該值落在修改後的輸出分布內；③ 同一二進位同一參數重複執行本就會變動（GA=20 重複 8 次得 44,992×6、46,528×1、45,184×1）。
>
> **判準修訂（最終版）**：非決定性成因鎖定為 `add_pinheads()` 的**遺傳演算法優化器**（早停條件 + 平行排程），變異幅度隨角度而異（30° 穩定、45° 可達 3.3%），且**固定 `SLA_GA_MAX_ITER` 無法消除**。spec `sla-overhang-threshold-semantics` 與 tasks `1.4`/`4.6` 已改為**分散式容差判準**：重複 ≥ 5 次，基準值須落在觀測集合內或極差 < 5%；次判準為 `stl_diff.py` 差異比例 < 5%。明文禁止 raw SHA-256 判準，亦禁止把固定 GA 迭代當成可重現單一輸出的依據。
>
> 依最終判準複驗：D 組基準值 46,528 落在觀測集合 {44,992、45,184、46,528} 內 → **通過**；C 組 47,936 前後皆穩定 → **通過**；90° 前後皆無 STL → **通過**。

## 5. 說明文字校正（不改行為）

- [x] 5.1 修正 `src/libslic3r/PrintConfig.cpp` 中 `support_critical_angle` 的 tooltip 方向敘述；同步檢查 `branchingsupport_critical_angle`
- [x] 5.2 修正 `src/libslic3r/SLA/DefaultSupportTree.cpp` 過濾判準處的註解
- [x] 5.3 於 `src/libslic3r/SLA/SupportTreeUtils.hpp`（Branching tree 副本）補上與 5.2 一致的說明
- [x] 5.4 修正 `src/libslic3r/SLA/SupportTree.hpp` 中 `overhang_angle_threshold` 的欄位註解
- [x] 5.5 修正 `doc/sla_support_angle_refactoring.md` 對照表：使公式與每一列「效果」描述皆可由 `polar < π/2 + threshold → 跳過` 推導；並補記「`π − threshold` 為另一種可能語意、與現行式僅在 45° 等值」供未來決策參考
- [x] 5.6 **驗證**：全文檢索相反敘述已無殘留；`cmake --build build --target libslic3r --config Release` 通過
- [x] 5.7 **驗證**：確認 `resources/profiles/*.ini` 的 `critical_angle` 值未被更動（spec：內建 profile 設定值不變）

> 5.1 補充：`support_critical_angle` 與 `branchingsupport_critical_angle` 共用同一份 `def` 定義（`prefix + "support_critical_angle"`），一處修改即同時涵蓋兩者。
>
> 校正後的統一表述：**支撐頭只放在「坡度 ≤ 90° − 設定值」的朝下面**，故數值越小支撐越多；0° 支撐所有懸空面，90° 只支撐完全水平朝下的面。此式由 `polar >= π/2 + threshold` 搭配「坡度 = π − polar」直接推導。
>
> 5.5 另補：`doc/` 內兩處引用舊註解的程式碼片段（`SupportTree.hpp` 與 `DefaultSupportTree.cpp` 節錄）已同步更新，並加註「`π − threshold` 屬破壞性變更」的警語。
>
> 5.7 結果：`git diff resources/profiles/` 為空，`AnycubicSLA.ini` 與 `PrusaResearchSLA.ini` 的 `support_critical_angle = 90` 維持不變。

## 6.【父 repo】Python E2E 測試補強

> 檔案位於父 repo：`D:\repos\web_slicer_core\agent\tests\test_support_e2e.py`

- [x] 6.1 確認 Agent 所使用的 CLI 已含本次變更（**未動 submodule 指標**，見下）
- [x] 6.2 **驗證**：`pytest agent/tests/test_support_e2e.py` 現況全綠（5 passed），確認基礎環境可用
- [x] 6.3 新增零支撐 fixture `_export_tilted_box()`：**繞 Y 軸傾斜 40° 的方塊**（球體與軸對齊方塊都會生支撐，見下方修正）
- [x] 6.4 **驗證**：以 CLI 直接跑該 fixture，90° 得 `No support/pad mesh generated`（pad 開關皆然）、45° 得 `(supports only)`
- [x] 6.5 改寫 Scenario 2：由 stub CLI 餵 `"(pad only)"` 改為真引擎案例，移入 `TestRealEngine`（掛 `_needs_engine`），並另補兩支真引擎案例
- [x] 6.6 **驗證**：`completed` / `SUPPORT_NOT_NEEDED` / `has_support_mesh is False` / `error_code is None`，且 stdout 含 `No support/pad mesh generated`、stderr **不含** `No pad can be generated`
- [x] 6.7 在 `agent/support_classifier.py` 的模組 docstring 註明 `"(pad only)"` 僅在 zero-elevation（`pad_around_object`）下可達
- [x] 6.8 **驗證**：三個測試檔合計 **42 passed**；`git status` 確認父 repo 僅 `support_classifier.py`（純 docstring）與 `test_support_e2e.py` 變更

> **6.1 說明**：Agent 由 `agent/config.py` 解析到 `third_party/prusaslicer_build/src/Release/slicer-engine.exe`，該檔為 in-tree 建置產物、非透過 submodule 指標取得，故本階段無需移動 submodule 指標（正式指標更新見 9.3）。其 SHA-256 為 `38a4b100…`，即 Task 4 已驗證通過的同一份二進位。
>
> ⚠️ **環境注意（已於 Task 7 釐清並同步）**：`third_party/prusaslicer_fork/build/` 是唯一可建置的 CMake build tree；`third_party/prusaslicer_build/` 只是 **staging 目錄**（無 `.sln`／`.vcxproj`，其 CMakeCache 為殘留），由開發者手動複製 `src/Release/` 覆蓋而來，並為 Agent 的讀取路徑。Task 7 已確認兩處 `src/Release/` 下所有檔案位元組全數一致。
>
> **6.3 前提修正（已用 CLI 實測）**：原稿假設「平底自支撐模型」可產生零支撐，實測為誤。軸對齊實體的底面法線為 (0,0,-1)、`polar == π`，即使閾值 90° 仍通過過濾而放置支撐頭。必須**傾斜**模型才會零支撐。此修正同時推翻了 `test_support_e2e.py` 原 docstring 中「零支撐無法由真引擎產生」的說法——該誤解正是此路徑從未被真實測到、缺陷得以出貨的原因。
>
> **本階段新增的真引擎案例（3 支，全部實際執行未 skip）**：
> 1. `test_zero_support_with_pad_reports_not_needed` —— 主回歸守衛，含 stdout/stderr marker 斷言與「STL 不得落地」檢查，並經 `TestClient` 驗證 API 回傳中性成功
> 2. `test_zero_support_without_pad_matches_the_pad_enabled_result` —— 把 pad 開/關兩條路徑釘在一起（兩者分歧正是本次缺陷）
> 3. `test_tilted_box_still_gets_supports_at_a_lower_threshold` —— **守衛 fixture 本身**：同一模型在 45° 必須長出支撐，避免未來「支撐功能整個壞掉」時前兩支測試仍靜默通過

## 7. 跨平台建置與封裝

- [x] 7.1 Windows：於 `third_party/prusaslicer_fork/build/` 重建，並依開發慣例手動複製覆蓋至 `third_party/prusaslicer_build/`（Agent 讀取路徑）
- [x] 7.2 **驗證**：Windows 產出的 CLI 重跑 4.3 / 4.5 通過；4.6 見下方判準說明
- [N/A] 7.3 macOS：以父 repo 的 `scripts/build_prusaslicer_fork_macos.sh` 重建 —— **N/A：Windows 本機開發環境，macOS 留待 CI 處理**
- [N/A] 7.4 **驗證**：macOS 產出的 CLI 重跑 4.3 / 4.5 —— **N/A：Windows 本機開發環境，macOS 留待 CI 處理**
- [~] 7.5 更新 Agent bundle 內嵌引擎 —— **二進位已落地並驗證、legal gate 已修復通過；仍卡在匯出表 gate（532 個匯出，要求 1 個），bundle 不可出貨**
- [x] 7.6 **驗證**：以 bundle 內的引擎路徑（`slicer-engine/bin/slicer-engine.exe`）重跑 4.3 通過，行為與開發建置路徑一致

> 證據：`evidence/win-repack-20260730T084942Z/PACKAGING.md`
>
> **Build tree 規則（已釐清）**：於 `prusaslicer_fork/build/` 編譯 → 手動複製 `slicer_core.dll` 與二進位覆蓋至 `prusaslicer_build/`。兩者為獨立 CMake build tree，非 symlink。本次已確認兩處 `src/Release/` 下**所有檔案位元組全數一致**，且無任何原始碼新於 `slicer_core.dll`（`2c59148f…`，含第 5 章文字校正）。`slicer-engine.exe` 時間戳較舊屬正常——它是薄啟動器，邏輯全在 DLL。
>
> **7.2 結果**：A（pad ON, 90°）→ `exit=0`、`No support/pad mesh generated`、stderr 空、無 STL；E（pad ON, 45°）→ `(includes supports and pad)`、44,028 tri（與基準相同）；B（pad OFF, 90°）→ 無 STL。
>
> **7.5 現況**：三個關鍵二進位（`slicer-engine.exe` / `slicer_core.dll` / `OCCTWrapper.dll`）已落地且與 build tree 位元組一致，共 1,012 檔。
>
> - **legal pack（已修復）**：`legal/slicer-engine/`（Windows／release 正式範本，[agpl-boundary.md:92](../../../../../docs/single-node-cloud/agpl-boundary.md#L92)）從未進版控、於本工作副本缺檔，腳本路徑本身並無錯誤。已修改【父 repo】`scripts/package_slicer_engine_windows.ps1`：正式路徑仍為第一優先，缺檔時回退到 `legal/slicer-engine-agpl/` 並印警告；來源檔名容錯（`NOTICE`／`NOTICE.md`、`SOURCE-OFFER.md`／`SOURCE_OFFER.md`），目的地維持正式命名；一併複製 `MODIFICATIONS.md`；新增 `REPLACE_WITH_` 佔位符掃描並警告 TEST-ONLY。此 gate 現已通過。
> - **⚠️ 仍阻塞：匯出表 gate**。`slicer_core.dll` 匯出 532 個符號，去識別化要求恰為 1 個（`slicer_run_cli`）。`src/slicer_core.def` 只宣告一個，洩漏來源需由 `slicer-engine/EXPORTS.txt` 診斷（[CMakeLists.txt:506-513](../../../CMakeLists.txt#L506-L513) 指出 cereal `StaticObject` 須靠全域 `CEREAL_FORCE_STATIC` 中和）。此 gate 屬合規要求，**未予弱化**。
> - 另有兩則警告：腳本自 `deps/build/destdir/usr/local/bin/` 找不到 `libgmp-10.dll` / `libmpfr-4.dll`（該兩檔實際位於 `src/Release/`），屬另一條路徑問題。
>
> ⚠️ **副作用**：`slicer-engine/` 產生後，`agent/config.py` 的引擎解析第一順位命中該路徑，Agent 已由開發建置路徑切換至此 bundle。已驗證 `pytest agent/tests/test_support_e2e.py` 經 bundle 路徑仍為 **7 passed**。此 bundle 缺 legal pack，**可供測試但不可出貨**。
>
> **4.6 判準修訂（已完成）**：以 D 組在 `SLA_GA_MAX_ITER=20` 重複 8 次得 44,992×6、46,528×1、45,184×1 —— **固定迭代數不會消除隨機性**，「完全相同」在任何 GA 設定下皆不可用（已觀測極差 1,536 tri = 3.30%）。已改為**分散式容差判準**並同步套用至 spec `sla-overhang-threshold-semantics`、tasks `1.4`/`4.6` 與 `AFTER.md` 的「證據二」。
>
> 本判準問題不影響修正本身的結論：D 組差異與本變更無關的決定性理由仍是「`pad_enable=0` 時新增程式碼一行都不執行」，該論證不依賴任何幾何比對。

## 8. 產品端到端驗收

- [x] 8.1 啟動本機 Agent 與 DS-online，載入 3DBenchy 並繞 Y 軸旋轉 -40°
- [x] 8.2 **驗證（主目標）**：Critical Angle 90° + 勾選 Pad Generation + Generate → 右上顯示中性「No support needed」，**不再出現 Request failed**
- [x] 8.3 **驗證（回歸）**：同模型取消 Pad Generation + Generate → 結果與 8.2 一致
- [x] 8.4 **驗證（回歸）**：平底自支撐模型 + 勾選 Pad Generation → 中性結果，非紅色錯誤
- [x] 8.5 **驗證（回歸）**：需要支撐的模型 + Critical Angle 45° + 勾選 Pad Generation → 正常長出支撐，`hasSupportMesh` 為 true，3D 場景可見支撐與底座
- [x] 8.6 **驗證（回歸）**：接續 8.5 執行切片，確認層數與預覽正常
- [x] 8.7 檢查 `agent/jobs/<新 job id>/` 的 `status.json`、`stdout_support.log`、`stderr_support.log`，確認欄位與 log 內容符合 spec `sla-pad-generation` 的中性契約

> 由使用者於 DS-online + 本機 Agent 手動完成：主目標 Bug 1 已由 `Request failed` 轉為中性「No support needed」，切片與正常支撐路徑皆正常。
>
> ⚠️ **8.4 的措辭與實測不符**：「平底自支撐模型」在本引擎並非零支撐模型——軸對齊實體的底面法線為 (0,0,-1)、`polar == π`，即使 Critical Angle 90° 仍通過過濾而放置支撐頭（Task 3/6 以 CLI 實測為 `(includes supports and pad)`）。故此項實際驗到的是「非紅色錯誤」，而非「中性 SUPPORT_NOT_NEEDED」。要取得零支撐必須**傾斜**模型。spec `sla-pad-generation` 的同名 scenario **已於後續修訂**為「繞 Y 軸傾斜 40° + Critical Angle 90°」並附上不得再引用平底模型的警語；proposal.md 與 design.md 的同源敘述亦已同步更正。

## 9. 收尾

- [x] 9.1 **驗證**：`openspec validate fix-empty-pad-slicing-error` 通過
- [x] 9.2 本 repo 提交變更，commit message 說明降級判準的三條件與「行為不變」的角度決策
- [x] 9.3 【父 repo】更新 submodule 指標至正式 commit，連同 `agent/tests/test_support_e2e.py` 一併提交
- [x] 9.4 **驗證**：於父 repo 全新 clone（含 submodule）重建並重跑 6.8 與 8.2，確認在乾淨環境可重現
- [x] 9.5 於 release note 記錄已知取捨：GUI 在「支撐啟用但零支撐柱 + pad 開啟」時改為靜默不生 pad；「有支撐但 pad 失敗」仍會連坐丟棄支撐（另案處理）

> **封存時的實際狀態（據實記錄，勿與驗證通過混為一談）**
>
> 本章依使用者指示於封存時統一勾選。其中僅 **9.1 為實際執行並通過**（`openspec validate` 一般與 `--strict` 皆回報 valid）。以下三項在封存當下**尚未實際發生**，屬交付後由使用者於版本流程中自行完成：
>
> | 項目 | 封存時實況 |
> |---|---|
> | 9.2 | fork 工作區仍為未提交狀態（`SLAPrintSteps.cpp` 等 10 個已修改檔 + `sla_pad_degradation_tests.cpp`、本 change 目錄兩個未追蹤項）。工作區另含與本變更無關的既有改動（`.gitignore`、`src/libslic3r/CMakeLists.txt`、`src/slic3r/CMakeLists.txt`），提交時須分離。 |
> | 9.3 | 父 repo submodule 指標未前進；`agent/tests/test_support_e2e.py`、`agent/support_classifier.py`、`scripts/package_slicer_engine_windows.ps1` 均未提交。 |
> | 9.4 | 未於乾淨 clone 重建重驗。已驗證的是本機環境：pytest 42 passed（6.8）、DS-online + 本機 Agent 手動驗收（8.2）。 |
>
> 9.5 的取捨內容已完整記載於 proposal.md「已知限制」與「相容性 / 行為風險」兩節，發版時可直接引用。
>
> **另有兩項在封存時未結案**：7.5（Windows 消費者 bundle 卡在匯出表 gate，532 匯出 vs 要求 1）與 3.5（「有支撐但 pad 無效」案例不可建構，改以 `validate_pad()` 述詞測試間接覆蓋）。兩者皆已於各章就地記錄理由。