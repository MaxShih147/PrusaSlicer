## 執行規則（每個階段都適用）

- 本對話**不執行任何編譯或 CMake 指令**。凡標示「👤 使用者操作」的步驟，由使用者在 Visual Studio 2022 或上層 repo 腳本中手動執行，再把結果貼回對話。
- 每個階段結尾的「Review 檢核點」是**硬性停止點**：實作者完成該階段後必須停下，等使用者確認 Diff 後才進入下一階段。
- 只允許修改兩個檔案：`src/libslic3r/SLA/SupportIslands/SampleConfigFactory.cpp` 與 `tests/sla_print/sla_supptgen_tests.cpp`。
- 絕對不修改、不 commit：`.gitignore`、`src/libslic3r/CMakeLists.txt`、`src/slic3r/CMakeLists.txt`，以及任何 CMake 檔。

### 建置環境速查（👤 使用者操作時參考）

- **測試用方案**：`D:\repos\web_slicer_core\third_party\prusaslicer_fork\build\PrusaSlicer.sln`（已開啟 `SLIC3R_BUILD_TESTS=ON`，Visual Studio 17 2022 產生器）。
- **在 VS2022 建置單一專案**：開啟上述 `.sln` → 上方工具列的「方案組態」選 `Release` → 「方案總管」展開 `tests` 資料夾 → 在 `sla_print_tests` 上按右鍵 →「建置」。只想檢查函式庫能否編譯時，改在 `libslic3r` 專案上按右鍵 →「建置」。
- **執行測試（命令列）**：
  ```
  cd D:\repos\web_slicer_core\third_party\prusaslicer_fork\build\tests\sla_print\Release
  sla_print_tests.exe "[SampleConfig]"
  ```
  RelWithDebInfo 的執行檔在同層的 `RelWithDebInfo\` 資料夾。
- **正式引擎（給 CLI 驗證用）**：在 `D:\repos\web_slicer_core\scripts` 執行 `build_prusaslicer_fork_windows.bat`。產出為 `D:\repos\web_slicer_core\third_party\prusaslicer_build\src\Release\slicer-engine.exe`。此腳本固定 `SLIC3R_BUILD_TESTS=OFF`，只建引擎，不建測試。
- **Debug 組態的限制**：目前 `deps/build` 的 `DEP_DEBUG=OFF`，相依套件只有 Release 版，Debug 組態大概無法連結。見任務 4.5。

## 1. SampleConfigFactory 核心驗證結構重構

- [x] 1.1 👤 **改 code 前先留基準**：把目前的 `D:\repos\web_slicer_core\third_party\prusaslicer_build\src\Release\slicer-engine.exe` 連同同資料夾的 DLL 複製到工作區外的備份資料夾（例如 `D:\baseline\slicer-engine-a114d13bd\`）。把產生 1609／1039 點時用的**完整 CLI 指令**原文記到本檔第 4.6 項，並保留兩次 `--export-support-points` 的輸出檔。Phase 4 要拿這些做逐點比對。
  - 進度：使用者已把 `prusaslicer_fork\build\src\Release` 的 `slicer-engine.exe` 與 `slicer_core.dll` 複製到 `D:\baseline\slicer-engine-a114d13bd\`（2026-10-12 13:21 建置）。注意基準來自 `prusaslicer_fork\build`，不是 `prusaslicer_build`，所以 Phase 4 的新引擎也應從 `prusaslicer_fork\build` 建置，兩邊才可比。使用者確認（2026-10-12）：當時為命令列即時測試，未留存 CLI 指令原文與輸出檔。Phase 4 的 CLI 驗收改以「點數維持 1609／1039、結束碼為 0」為準，4.8 略過。
- [x] 1.2 在 `SampleConfigFactory.cpp` 加入 `#include <libslic3r/EngineErrorCodes.hpp>`。
- [x] 1.3 在 `.cpp` 的匿名 namespace 新增 `bool verify_once(SampleConfig &cfg)`：把原 `verify()` 的三個 lambda 與 7 條檢查**原封不動**搬過去，只做兩件事——移除第 7、15、24 行的 `assert`，以及刪掉 `if (!res) while (!verify(cfg));`。7 條檢查的順序與 `res &= …` 寫法不可更動，**不得**改成 `&&`。
- [x] 1.4 在同一個匿名 namespace 加 `constexpr int max_extra_verify_passes = 20;`。
- [x] 1.5 改寫 `SampleConfigFactory::verify()`：首輪 `verify_once()` 回 `true` 就回傳 `true`；否則至多再跑 20 輪，任一輪回 `true` 就回傳 `false`；超過上限則拋出 `Slic3r::CodedRuntimeError(Slic3r::make_engine_error(Slic3r::EngineErrorCode::SUPPORT_POINT_SAMPLING_FAILED, "SampleConfig convergence failed: cyclic geometric constraint"))`。
- [x] 1.6 改 `create()` 結尾：把 `verify(result);` 換成兩行——`[[maybe_unused]] const bool clean = verify(result);` 與 `assert(clean);`。**不可**寫成 `assert(verify(result));`。
- [x] 1.7 確認 `SampleConfigFactory.hpp` 沒有被修改，`verify()` 的宣告維持 `static bool verify(SampleConfig &cfg);`。
- [x] 1.8 **即時驗證（靜態，可在本對話完成）**：檢查 diff，確認以下幾點——檔內已沒有 `while (!verify`；`verify_once` 內有 7 處 `res &=`；`verify_once` 內沒有任何 `assert`；`create()` 內的 `assert` 參數不含函式呼叫；`git status` 只顯示 `SampleConfigFactory.cpp` 這一個新增修改（加上原本那三個未 commit 檔）。
- [x] 1.9 👤 **（選做）編譯檢查**：在 VS2022 以 `Release` 組態，對 `libslic3r` 專案按右鍵 →「建置」，確認沒有編譯錯誤。把「輸出」視窗最後的「建置: 成功 x 個，失敗 x 個」那一行貼回對話。
- [x] 1.10 **🛑 Review 檢核點 1**：停下，等使用者確認 Diff 與語法正確，才進入 Phase 2。

## 2. apply_density 飽和防禦實作

- [x] 2.1 保留 `apply_density()` 開頭的兩段原邏輯（`is_approx(density, 1.f)` 直接回傳、`density < .1f` 拉到 0.1），位置與寫法不變。
- [x] 2.2 以**原算式**計算 `const coord_t t_original = static_cast<coord_t>(current.thin_max_distance / density);`。這是 int32 ÷ float 的 float 運算，型別與運算順序**一字不改**，不可改成 double。
- [x] 2.3 計算 `const coord_t m_thresh = std::max(current.minimal_distance_from_outline, current.minimal_move);`，並在旁邊註解說明 `minimal_move` 兼任飽和下限。
- [x] 2.4 以**原算式**算出另外兩個原始值：`inner_original = static_cast<coord_t>(std::sqrt(sqr((double) current.thick_inner_max_distance) / density))` 與 `outline_original = static_cast<coord_t>(current.thick_outline_max_distance / density)`。兩行都從 git HEAD 原封不動搬過來。
- [x] 2.5 原路徑（`t_original >= m_thresh`）：把 `t_original`、`inner_original`、`outline_original` 原樣寫入三個欄位，**不套用任何下限**。
- [x] 2.6 飽和路徑（`t_original < m_thresh`）：
  - `thin_max_distance = m_thresh;`（只有 thin 飽和，不需要算飽和密度）
  - `thick_inner_max_distance = std::max(inner_original, std::max(2 * current.head_radius, current.minimal_move));`——下限取 `max(頭直徑, minimal_move)`，因為頭徑 0 時頭直徑就是 0。
  - `thick_outline_max_distance = std::max(outline_original, current.minimal_move);`
  - 三個 `std::max` 的兩個參數都必須是 `coord_t`，否則模板推導會編譯失敗。
- [x] 2.7 確認兩條路徑都**沒有**寫入 `minimal_distance_from_outline` 或其他欄位，且結尾的 `verify(result); return result;` 不變。
- [x] 2.8 確認 `apply_min_distance()`、`create()` 的其餘部分、以及 `USE_ISLAND_GUI_FOR_SETTINGS` 區塊都沒有被改動。
- [x] 2.9 **即時驗證（靜態，可在本對話完成）**：逐行對照 diff 與 design.md D2，確認——三個 `*_original` 的算式與 git HEAD 位元組相同；原路徑沒有任何 `std::max`；`thick_inner` 用的是 `std::sqrt`；`m_thresh` 沒有被寫回任何欄位。再用 0.6 mm 頭手算：3100% 時 `thick_inner` 約 1.485 mm、4200% 時約 1.276 mm；`INT_MAX`% 時 outline = 0.1 mm、inner = 0.6 mm。
  - 實作形式說明（2026-10-12）：原本三行縮放算式留在原位、一字不改，直接寫入 `result` 的三個欄位，`result.thin_max_distance` 即為 `t_original`；其後以 `if (result.thin_max_distance < m_thresh)` 進入飽和路徑覆寫。語意與 2.2～2.6 相同，但原路徑在 diff 上完全沒有變動。
  - 2.9 結果：前段算式、`apply_min_distance()`、GUI 區塊與 HEAD 逐字相同；`m_thresh` 只出現在宣告、比較與寫入 thin 三處；`minimal_distance_from_outline` 只在 `create()` 被賦值。Python 模擬 0.6 mm 頭：3100% inner 1.4849 mm、4200% 1.2758 mm、INT_MAX% outline 0.1000 mm／inner 0.6000 mm；11 種頭徑 × 約 2900 種密度皆未拋出例外，verify 最多額外 3 輪（模擬，非實機）。
- [x] 2.10 👤 **（選做）編譯檢查**：同 1.9，重新建置 `libslic3r`，回報結果。
- [x] 2.11 **🛑 Review 檢核點 2**：停下，等使用者確認飽和公式與型別轉換細節，才進入 Phase 3。

## 3. tests/sla_print/sla_supptgen_tests.cpp 測試套件實作

- [x] 3.1 在 `sla_supptgen_tests.cpp` 的 include 區加入 `#include <libslic3r/EngineErrorCodes.hpp>`（以及用到 `INT_MAX` 所需的 `<climits>`）。所有新測試附加在檔案尾端，tag 一律用 `[SupportIsland][SampleConfig]`。
- [x] 3.2 在檔內匿名 namespace 實作 `legacy_verify(SampleConfig &cfg, int depth = 0)`：以 git `a114d13bd` 版本的 `verify()` 逐字複製，只做兩件事——移除三個 `assert`；把遞迴呼叫改為傳入 `depth + 1`，`depth > 50` 時拋出測試專用例外。在註解標明來源 commit。
- [x] 3.3 在同一個匿名 namespace 實作 `legacy_apply_density()`：同樣自 `a114d13bd` 逐字複製，改為呼叫 `legacy_verify()`，**不可**呼叫新版 `verify()`。
- [x] 3.4 實作 `legacy_apply_min_distance()`：同樣逐字複製並改呼叫 `legacy_verify()`，供含 min_distance 的網格使用。
- [x] 3.5 實作比對輔助函式，對兩個 `SampleConfig` 的全部純量欄位逐一 `CHECK(a.x == b.x)`。欄位包括 13 個 `coord_t`、`max_length_ratio_for_two_support_points`、`count_iteration`、`simplification_tolerance`。失敗訊息須帶出頭徑與密度百分比（用 `INFO` 或 `CAPTURE`）。
- [x] 3.6 測試「逐欄位元一致的網格比對」：頭徑 `{0.3, 0.4, 0.5, 0.6, 0.8, 1.0}` × 自 10% 起逐 1% 遞增。密度以 `float(pct / 100.f)` 轉換。每點先用原 float 算式算出 legacy 的 `t`，`t < minimal_distance_from_outline` 時結束該頭徑的掃描。對 `create → legacy_apply_density` 與 `create → apply_density` 呼叫比對函式。
- [x] 3.7 測試「含 min_distance 的網格比對」：網格同 3.6，兩邊都再接 `apply_min_distance(…, 1.5f)`（legacy 端用 `legacy_apply_min_distance`）後比對。
- [x] 3.8 測試「thin 飽和」：0.6 mm 頭在 2205%、5000%、10000% 三種密度下，`thin_max_distance` 都等於 `minimal_distance_from_outline`；`thick_inner_max_distance` 隨密度增加而嚴格變小。
- [x] 3.9 測試「thick 照原公式繼續縮小」：0.6 mm 頭在 3100% 時 `thick_inner_max_distance` 約為 1.48 mm、4200% 時約為 1.27 mm（容差 ±0.01 mm），且兩者等於測試內用原開根號算式算出的值（`==`）。
- [x] 3.10 測試「極端密度不崩潰與下限」：頭徑 `{0, 0.1, 0.6}` × 密度 `{5000, 10000, INT_MAX}`%，走 `create → apply_density → apply_min_distance(…, 0)`。斷言 `REQUIRE_NOTHROW`；`thin_max_distance ≥ max(minimal_distance_from_outline, minimal_move)`；`thick_outline_max_distance ≥ minimal_move`；`thick_inner_max_distance ≥ max(2 * head_radius, minimal_move)`；對結果再呼叫一次 `verify()` 回傳 `true`。另外斷言 0.6 mm × `INT_MAX`% 時 outline 正好等於 0.1 mm、inner 正好等於 0.6 mm，頭徑 0 × `INT_MAX`% 時 `minimal_distance_from_outline` 仍為 0。
- [x] 3.11 測試「下限只在飽和路徑」與「小頭徑邊界」：
  - 0.2 mm 頭、4100%：`outline_original` 已低於 0.1 mm 但 thin 尚未飽和，結果與 legacy 逐欄相同，且 `thick_outline_max_distance < minimal_move`。
  - 0.1 mm 頭：取一個 `t_original ≥ 0.1 mm` 的密度，與 legacy 逐欄相同；再取一個 `minimal_distance_from_outline ≤ t_original < 0.1 mm` 的密度，`thin_max_distance == minimal_move`，且 `thick_outline_max_distance ≥ minimal_move`。比較時一律用 `cfg.minimal_move` 欄位，不要寫 `scale_(0.1)`：後者是 double，與截斷後的整數可能差 1。
- [x] 3.12 測試「互斥配置觸發錯誤」：對 `create(0.6f)` 的結果設 `thin_max_distance = minimal_distance_from_outline / 2`，呼叫 `verify()`。以 try/catch 捕捉，斷言確實有拋出例外、`dynamic_cast<const EngineErrorCarrier*>` 不為空，且 `engine_error().code` 有值（它是 `std::optional<EngineErrorCode>`），值等於 `EngineErrorCode::SUPPORT_POINT_SAMPLING_FAILED`。
- [x] 3.13 測試「標準頭徑首輪通過」：0.4、0.6、0.8 mm 的 `create()` 結果複製一份後呼叫 `verify()`，回傳 `true`。
- [x] 3.14 測試「Debug 可執行高密度路徑」：0.6 mm 頭、300%，`create → apply_density` 正常返回。這個案例在 Release 下也要執行；它的作用是在 Debug 組態確認舊 assert 已經移除。
- [x] 3.15 確認 `tests/sla_print/CMakeLists.txt` 沒有被修改。
- [x] 3.16 **即時驗證（靜態，可在本對話完成）**：對照 spec.md 的 15 個 Scenario，列出每個 Scenario 對應到哪一個 `TEST_CASE`。手動 CLI 那一個 Scenario 對應到 4.6。
  - 3.16 對照結果（spec 現有 19 個 Scenario，前版為 15 個；thin-only 飽和改版後增加）：
    - 原崩潰密度不再崩潰、超過門檻後 thin 不再縮小 → `thin spacing saturates past the crash threshold`
    - 極端密度不崩潰也不卡死、頭徑為 0 的一般密度、thick 間距觸及下限、頭徑為 0 時三個間距都有下限 → `extreme densities return with every spacing floored`
    - 互斥配置觸發錯誤 → `verify() reports a contradictory config with the engine error code`
    - 正常呼叫鏈不觸發錯誤 → `no head size or density makes the factory throw`
    - thick_inner 照原公式繼續縮小 → `thick spacings keep the legacy formulas past saturation`
    - 下限只在飽和路徑套用、小頭徑低密度維持不變、小頭徑高密度改為飽和 → `floors only apply once thin spacing saturates`（兩個 SECTION）
    - 逐欄位元一致的網格比對 → `density gives the legacy result bit for bit`
    - 含 min_distance 的網格比對 → `density with min distance gives the legacy result bit for bit`
    - legacy 對照的遞迴保護 → `legacy_verify fails instead of overflowing the stack`
    - 標準頭徑首輪通過 → `create() defaults pass verify() on the first pass`
    - 高密度路徑不因 assert 中止 → `a density that makes verify() clamp does not trip an assert`
    - 端到端整合驗證（手動）→ 任務 4.7；Release 組態執行測試 → 任務 4.2（RelWithDebInfo 已略過，見 4.4）
  - 超出 3.6～3.14 字面的補充（為了讓每個 Scenario 都有對應測試）：遞迴保護測試、無例外掃描測試（8 種頭徑 × 0～10000% 逐 1% 加上稀疏高密度與 INT_MAX）、2205% 加 min_distance 1.5 的不崩潰斷言、頭徑 0 在 100% 的不崩潰斷言、3.9 另驗 outline 仍等於線性原式。
  - legacy 函式逐字保留邏輯，只去除 assert 與註解、移除原檔的行尾空白，並加深度保護。
  - 檢核點 3 審查後的修正（2026-10-12）：新增 `legacy_create_unverified()`／`legacy_create()`，網格改為整條 legacy 鏈（含 create）對整條新鏈；「首輪通過」改測未校驗的原始預設值，頭徑擴為網格的 6 種；以 `check_constraints_hold()` 直接斷言 7 條約束，取代只會回 true 的 `verify(again)`；2205%／5000%／10000% 加 min_distance 1.5 時斷言 thin = 1.5 mm、兩個 thick ≥ 1.5 mm 且約束成立；改用 `std::numeric_limits<coord_t>::max()` 與 `unscale<double>()`；掃描測試每點只算一次 `apply_density`，0%～9% 只留 0%。
  - 邊界補強（2026-10-12）：網格頭徑加入 0.2 mm（head_radius 與 minimal_move 同為 100000 的邊界）與 1.5 mm；raw 首輪通過測試涵蓋 0、0.1、0.2～1.5 mm 共 10 種；無例外掃描的 min_distance 加入 0.15 mm。
  - 掃描補強（2026-10-12）：無例外掃描的頭徑加入 0.2 mm 與 1.5 mm（共 10 種），讓 0.2 mm 超過約 4340% 的飽和路徑實際被執行（離線模擬：0.2 mm 有 6653 個、1.5 mm 有 8321 個密度點進入飽和，皆無例外）；第二段密度從 10997% 起，不再重複 10000%；補上 `<cmath>`、`<cstdint>`、`<stdexcept>`、`<utility>`、`<vector>`。
- [x] 3.17 **🛑 Review 檢核點 3**：停下，等使用者確認測試覆蓋率與斷言條件，才進入 Phase 4。

## 4. 建置驗證與全流程驗收

- [x] 4.1 👤 **建置測試（Release）**：在 VS2022 開啟 `prusaslicer_fork\build\PrusaSlicer.sln`，組態選 `Release`，在「方案總管」→ `tests` → `sla_print_tests` 上按右鍵 →「建置」。回報「輸出」視窗最後一行；若失敗，貼出第一個錯誤（含檔名與行號）。
- [x] 4.2 👤 **執行新測試（Release）**：在命令列執行
  ```
  cd D:\repos\web_slicer_core\third_party\prusaslicer_fork\build\tests\sla_print\Release
  sla_print_tests.exe "[SampleConfig]"
  ```
  把最後的摘要（`All tests passed` 或失敗清單）貼回對話。
  - 4.1／4.2 最終版測試的結果（2026-10-12 15:12，使用者回報）：VS2022 Release 建置「10 成功，0 失敗，31 為最新狀態，6 已跳過」；`sla_print_tests.exe "[SampleConfig]"` → `All tests passed (1020968 assertions in 11 test cases)`，亂數種子 742942107。
  - 4.1／4.2 第三版（邊界補強後）測試的結果（2026-10-12 14:53，使用者回報；之後又補了掃描頭徑，需重跑後才能再勾選）：VS2022 Release 建置「10 成功，0 失敗，31 為最新狀態，6 已跳過」；`sla_print_tests.exe "[SampleConfig]"` → `All tests passed (955082 assertions in 11 test cases)`，亂數種子 2592403361。
  - 4.1／4.2 第二版測試的結果（2026-10-12 14:33，使用者回報；之後又補了邊界測試，需重跑後才能再勾選）：VS2022 Release 建置「10 成功，0 失敗，31 為最新狀態，6 已跳過」；`sla_print_tests.exe "[SampleConfig]"` → `All tests passed (643250 assertions in 11 test cases)`，亂數種子 2475914414。
  - 4.1／4.2 前一版測試的結果（2026-10-12，使用者回報；測試已依檢核點 3 審查修改，需重跑後才能再勾選）：VS2022 Release 建置「18 成功，0 失敗，23 為最新狀態，6 已跳過」；`sla_print_tests.exe "[SampleConfig]"` → `All tests passed (642998 assertions in 11 test cases)`，亂數種子 4228244129。
- [x] 4.3 👤 **執行整支 sla_print 測試（Release）**：同一個資料夾執行 `sla_print_tests.exe`（不帶參數），確認既有測試（特別是 `[SupportIsland]`）沒有因本變更而失敗。貼回摘要。
  - 4.3 結果（2026-10-12 15:21，使用者回報）：`sla_print_tests.exe`（不帶參數）→ `All tests passed (1216181 assertions in 190 test cases)`，亂數種子 143925310。終端出現的 Voronoi 與 `[raster-verify]` 訊息來自既有測試的正常輸出。
- [x] 4.4 （略過：使用者決定，見下方說明）👤 **RelWithDebInfo**：組態改選 `RelWithDebInfo`，重複 4.1～4.2，執行檔在 `build\tests\sla_print\RelWithDebInfo\`。
  - 決定（2026-10-12）：略過，未實際執行。RelWithDebInfo 與 Release 同樣定義 `NDEBUG`、計算邏輯等價，而其建置產物為舊版、需整個重編 libslic3r。spec「不同編譯組態的計算結果必須一致」已同步改為只以 Release 驗收，並註明日後重建時補跑。
- [x] 4.5 👤 **Debug（目前可能受阻）**：spec 要求測試在 Debug 下也要通過，但目前 `deps/build` 的 `DEP_DEBUG=OFF`，Debug 組態預期無法連結。請使用者決定其中一種：(a) 以 `DEP_DEBUG=ON` 重建 deps 後再跑 Debug；(b) 在本 change 記錄「Debug 驗證暫緩」，修訂 spec 的組態需求。決定前，這一項維持未勾選。
  - 決定（2026-10-12）：採 (b)。相依套件沒有 Debug 版，本次暫緩 Debug 驗證，以 Release／RelWithDebInfo 為主要驗收環境。spec 已同步修訂。
- [x] 4.6 👤 **建置正式引擎**：在 `D:\repos\web_slicer_core\scripts` 執行 `build_prusaslicer_fork_windows.bat`，確認產出 `third_party\prusaslicer_build\src\Release\slicer-engine.exe`。
  - 建議做法（2026-10-12）：基準引擎是從 `prusaslicer_fork\build` 建出來的，新引擎也從同一個 VS 方案建置，兩邊才可比：在 `PrusaSlicer.sln` 以 Release 組態建置 `PrusaSlicer_app_console`（輸出 `slicer-engine.exe`，會連帶建置產出 `slicer_core.dll` 的 `PrusaSlicer` 專案），產物在 `prusaslicer_fork\build\src\Release\`。上層腳本會另外以 `SLIC3R_GUI=OFF`、`SLIC3R_BUILD_TESTS=OFF` 建到 `prusaslicer_build`，組態與基準不同，留給正式打包使用。
- [x] 4.7 👤 **CLI 整合驗證**：用 1.1 記下的完整指令（50×50×10 mm 平放方塊、`support_head_front_diameter = 0.6`、`support_pillar_diameter = 1.0`、`--export-support-points`），分別以新引擎跑以下幾組：
  - 2204%、min_distance 0 → 應為 1609 點
  - 2204%、min_distance 1.5 → 應為 1039 點
  - 2205%、5000%、10000%（min_distance 0 與 1.5 各一次）→ 正常結束、退出碼 0、不再出現 `3221225725`

  把每組的點數與退出碼貼回對話。CLI 指令原文：`（於 1.1 填入）`
  - 驗證方式（2026-10-12）：同一條指令分別用新引擎與 `D:\baseline\slicer-engine-a114d13bd\slicer-engine.exe` 各跑一次，看 stdout 的 `Support points exported to … (N points)` 與 `echo %ERRORLEVEL%`。2204% 時新舊點數必須相同（且應為 1609／1039）；2205%、5000%、10000% 時舊引擎應以 `-1073741571`（即 3221225725）結束，新引擎應以 0 結束。匯出檔寫到 repo 外的資料夾（例如 `D:\cli_check\`），避免影響 4.9。
  - 建置紀錄（4.6）：VS2022 Release 建置 `PrusaSlicer_app_console`「10 成功，0 失敗，31 為最新狀態，6 已跳過」（2026-10-12 15:47，48.1 秒）；`prusaslicer_fork\build\src\Release\slicer-engine.exe` 時間戳 2026-10-12 15:47:19。
  - 實際使用的指令（2026-10-12，由 `D:\cli_check\run_cli_check.bat` 批次執行，輸出存於同資料夾的 `run_cli_check.log`）：`slicer-engine.exe --printer-technology SLA --support-head-front-diameter 0.6 --support-pillar-diameter 1.0 --support-points-density-relative <密度> --support-points-min-distance <min> --export-support-points D:\cli_check\<引擎>_<密度>_<min>.json C:\Users\kwshi\Desktop\box_50x50x10.stl`。模型為 12 個三角面、50×50×10 mm、底面在 z=0。其餘參數皆為引擎預設值。原始產生 1609／1039 點的指令已遺失，故其餘參數與當初不同。
  - 數據紀錄（4.7；「點數」取自 stdout 的 `(N points)`，「結束碼」取自 `echo %ERRORLEVEL%`）：

    | 密度 | min_distance | 舊引擎點數 | 舊引擎結束碼 | 新引擎點數 | 新引擎結束碼 | 預期 | 判定 |
    |---|---|---|---|---|---|---|---|
    | 2204% | 0 | 909 | 0 | 909 | 0 | 新舊相同、結束碼 0 | 通過：新舊匯出檔逐位元組相同 |
    | 2204% | 1.5 | 908 | 0 | 908 | 0 | 新舊相同、結束碼 0 | 通過：新舊匯出檔逐位元組相同 |
    | 2205% | 0 | — | -1073741571 | 911 | 0 | 舊 -1073741571；新 0 | 通過 |
    | 2205% | 1.5 | — | -1073741571 | 910 | 0 | 舊 -1073741571；新 0 | 通過 |
    | 5000% | 0 | — | -1073741571 | 2068 | 0 | 舊 -1073741571；新 0 | 通過 |
    | 5000% | 1.5 | — | -1073741571 | 1253 | 0 | 舊 -1073741571；新 0 | 通過 |
    | 10000% | 0 | — | -1073741571 | 4129 | 0 | 舊 -1073741571；新 0 | 通過 |
    | 10000% | 1.5 | — | -1073741571 | 1253 | 0 | 舊 -1073741571；新 0 | 通過 |

  - 判定（2026-10-12，使用者確認）：通過。驗收依據改為「新舊引擎同參數下匯出檔逐位元組相同」與「2205% 以上舊引擎崩潰、新引擎正常結束」，spec 端到端 Scenario 已同步修訂。
  - 與原基準的差異：2204% 的點數是 909／908，不是當初的 1609／1039。新舊引擎在同一指令下逐位元組相同，代表差異來自指令參數不同（原指令遺失），不是本變更造成。1609／1039 這兩個數字本次未重現。
  - 觀察：5000% 與 10000% 在 min_distance 1.5 時都是 1253 點。合理的解釋是 min_distance 把三個間距都拉到 1.5 mm，兩種密度得到相同的 island 配置；此解釋未另行驗證。
- [x] 4.8 （已執行，2026-10-12：改以同一指令重跑備份的舊引擎取得輸出檔；`OLD_2204_0.json` 與 `NEW_2204_0.json`、`OLD_2204_1.5.json` 與 `NEW_2204_1.5.json` 逐位元組相同）👤 **（選做）逐點比對**：把 2204% 兩組的輸出檔，與 1.1 備份的舊引擎輸出比對內容（例如用 `fc /b 舊檔 新檔`）。完全相同就是最強的端到端證據；若只有點數相同而內容不同，請回報，需要調查。
- [x] 4.9 確認最終 `git status` 只多了 `SampleConfigFactory.cpp` 與 `sla_supptgen_tests.cpp` 兩個修改，原本那三個未 commit 檔維持原狀。
  - 4.9 結果（2026-10-12）：`git status` 只有 `SampleConfigFactory.cpp` 與 `sla_supptgen_tests.cpp` 兩個修改，加上原本未 commit 的 `.gitignore`、`src/libslic3r/CMakeLists.txt`、`src/slic3r/CMakeLists.txt`（未動），以及本 change 的 `openspec/changes/fix-sla-island-extreme-density/`。CLI 驗證的 bat、log 與 JSON 都在 repo 外的 `D:\cli_check\`。
- [x] 4.10 把 4.2～4.8 的結果摘要記錄到本檔各項旁邊，供 `/opsx:verify` 與歸檔使用。
