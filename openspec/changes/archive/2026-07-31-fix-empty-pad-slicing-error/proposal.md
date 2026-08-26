## Why

DS-online 使用者對 3DBenchy（小船）做 Y 軸 -40° 旋轉、把 Critical Angle 拉到 90°、勾選 Pad Generation 後按 Generate，右上角出現紅色 **Request failed**（實際 `error_code = SUPPORT_GENERATION_FAILED`）。同一組參數只要取消勾選 Pad Generation，卻得到中性的 **No support needed** 而成功收尾。

實測 job（`agent/jobs/bb65f4d6`）鎖定完整因果鏈：

```
Critical Angle 90° ─▶ 支撐點過濾後 support tree 為空（0 支撐柱）
      │
      ├─ Pad OFF ─▶ pad_mesh 空、不拋錯 ─▶ CLI 印 "No support/pad mesh generated"
      │             ─▶ 後端分類為 COMPLETED / SUPPORT_NOT_NEEDED（中性成功）
      │
      └─ Pad ON  ─▶ pad footprint 來自 support tree，空 ─▶ pad_mesh 空
                    ─▶ validate_pad() 回 false ─▶ throw SlicingError
                       "No pad can be generated for this model with the current configuration"
                    ─▶ CLI catch → cerr → return false
                    ─▶ ★ 整個 export 中止，永遠到不了支撐 STL 匯出區塊
                    ─▶ stdout 無任何 marker ─▶ 後端分類器 fail-closed
                    ─▶ SUPPORT_GENERATION_FAILED（紅色錯誤）
```

兩個現象是**同一個狀態（零支撐柱）的兩張臉**，差別只在 Pad 開關決定它走「靜默空結果」還是「拋例外中止」。而 `pad_mesh` 為空在此情境下並不是錯誤——它是「沒有支撐柱，因此沒有 pad 可長」的必然幾何結果，語意上應與 Pad OFF 收斂到同一個中性終點。

此外，例外在 `process()` 階段拋出，而支撐 STL 的匯出在其後，因此**只要 pad 失敗，整趟 export 就會中止**。這個結構性缺陷與 Critical Angle 無關，任何「零支撐柱 + Pad ON」的組合都會踩到，Critical Angle 90° 只是最容易觸發的一條路徑。

> 補述（實測修正）：原稿曾把「平底自支撐模型」列為另一種觸發情境，**實測為誤**。軸對齊實體的底面法線為 `(0,0,-1)`、極角 `polar == π`，即使閾值 90° 仍通過過濾而生成支撐柱。要構造零支撐必須**傾斜**模型（本變更的測試 fixture 採繞 Y 軸 40°）。

修復必須落在 C++ 層：例外中止使 stdout 沒有 marker、磁碟上沒有 STL，Python 分類器**在物理上無從區分**「零支撐導致 pad 空」與「真正的 pad 參數錯誤」，在該層攔截必然變成一刀切、會吞掉真實錯誤。

## What Changes

### 行為變更（唯一一處）

- `SLAPrint::Steps::generate_pad()` 在 `validate_pad()` 判定失敗時加入降級判準：
  - 若 **pad mesh 為空且 support mesh 亦為空** → 視為「無事可做」，令 `pad_mesh = {}` 並讓步驟正常完成，**不再拋出 `SlicingError`**。
  - 其餘情況（support mesh 非空、pad 卻長不出來）→ **維持拋出 `SlicingError`**，fail-closed 不被削弱。
- 降級後 CLI 的支撐匯出區塊會正常執行：`slaposPad` 標記為 done、`pad_mesh` 空、`support_mesh` 空 → `combined_mesh` 空 → 印出既有的 `"No support/pad mesh generated"`，落進後端分類器現成的中性路徑。
- 降級情境下 `process()` 不再中止，支撐匯出流程得以完整執行。
- **非破壞性**：不新增/移除/改名任何設定項，不改變任何既有成功路徑的輸出。

### 文字與說明修正（不改行為）

`support_critical_angle`（label 為 "Overhang threshold"）的實作語意是「**數值越小，生成的支撐越多**」——實作判準為 `polar_min = π/2 + threshold`。但 tooltip、原始碼註解與設計文件三處都寫成相反方向（宣稱 90° = 支撐所有 overhang）。經決策**維持現有支撐密度不變**（30°/45°/90° 的實測行為一律不動），僅將說明文字校正為與實作一致：

- `PrintConfig.cpp` 的 tooltip
- `DefaultSupportTree.cpp` 過濾判準處的註解
- `SupportTreeUtils.hpp`（Branching tree 副本）補上一致說明
- `SupportTree.hpp` 中 `overhang_angle_threshold` 欄位註解
- `doc/sla_support_angle_refactoring.md` 對照表「效果」欄的 0° / 90° 兩列描述

### 測試

- **本 repo**：新增 Catch2 案例（`tests/sla_print/`）覆蓋「零支撐 + `pad_enable`」不拋例外、`pad_mesh` 為空；以及「有支撐、pad 失敗」仍拋例外。
- **父 repo（`web_slicer_core`）**：`agent/tests/test_support_e2e.py` 的 Scenario 2（「pad_enable=True 但零支撐 → SUPPORT_NOT_NEEDED」）目前以 stub CLI 人工餵入 `"(pad only)"` 字串。該字串在現行 agent 組態下**不可達**——`SLAConfig` 無 `pad_around_object` 欄位，`embed_object` 永遠 disabled，`validate_pad` 退化為 `!pad.empty()`，零支撐必拋例外。這正是本缺陷從未被測到的原因。須改為真引擎案例（零支撐 + Pad ON → `"No support/pad mesh generated"` → `SUPPORT_NOT_NEEDED`）。

  > 檔案屬父 repo：`D:\repos\web_slicer_core\agent\tests\test_support_e2e.py`（此變更一併涵蓋，詳見 tasks.md）

### 明確不做（已決策的取捨）

- **不修改支撐角度過濾公式**，30°/45°/90° 的實測支撐密度完全不變。
- **不調整** `resources/profiles/*.ini` 的 `critical_angle = 90`。已知後果：以本 fork 的 GUI 開預設 SLA profile 幾乎不生支撐；Agent 路徑不受影響，因為 `generate_config_ini` 永遠明寫參數值。
- **不移除** `"(pad only)"` marker，保留為 zero-elevation（`pad_around_object`）專屬路徑。
- **不改動** Python 分類器的分類邏輯（僅補模組 docstring 註解）、Agent API、前端預設值與 Critical Angle 滑桿範圍。

### 已知限制

- **「有支撐但 pad 失敗」仍會連坐丟棄已生成的支撐。** 例外中止 `process()` 的機制本身不變，因此在 fail-closed 分支（support mesh 非空、pad 卻長不出來）下，已生成的支撐柱仍會隨著 export 中止而丟失。本變更只讓「零支撐柱」這一種情境不再走進該分支——降級路徑成立的前提就是 support mesh 為空，該情境本就沒有支撐可保留。要一併解決連坐，需改變例外策略或支撐匯出的時機，超出本次已決策的範疇，另案處理。

## Capabilities

### New Capabilities
- `sla-pad-generation`: SLA pad 生成步驟的成敗判準與降級行為——涵蓋 `validate_pad` 失敗時「空 pad + 空 support tree 視為無事可做」與「有 support 卻無 pad 視為真實錯誤」的分流、降級後 CLI 支撐 STL 匯出與 stdout marker 的可觀測契約，以及 pad 失敗不得連坐丟棄已生成支撐。
- `sla-overhang-threshold-semantics`: `support_critical_angle` / `branchingsupport_critical_angle`（"Overhang threshold"）的語意契約——釘住「數值越小支撐越多」的實作方向，並要求 tooltip、原始碼註解與設計文件與之一致，避免日後被誤認為缺陷而反向「修正」造成既有專案支撐密度靜默翻轉。

### Modified Capabilities
<!-- 既有 spec（sla-layer-blur、sla-support-binary-rasterization）皆不涉及 pad 生成或支撐角度，無需修改 -->

## Impact

- **程式碼（本 repo）**
  - [src/libslic3r/SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp) — `generate_pad()` 的 `validate_pad` 失敗分支加入降級判準（唯一行為變更點；pad 邏輯不分樹種，Default / Branching 共用此處）。
  - [src/libslic3r/PrintConfig.cpp](../../../src/libslic3r/PrintConfig.cpp)、[src/libslic3r/SLA/DefaultSupportTree.cpp](../../../src/libslic3r/SLA/DefaultSupportTree.cpp)、[src/libslic3r/SLA/SupportTreeUtils.hpp](../../../src/libslic3r/SLA/SupportTreeUtils.hpp)、[src/libslic3r/SLA/SupportTree.hpp](../../../src/libslic3r/SLA/SupportTree.hpp) — 僅註解與 tooltip 文字。
  - [doc/sla_support_angle_refactoring.md](../../../doc/sla_support_angle_refactoring.md) — 對照表描述校正。
  - 唯讀依賴（不修改）：`SLAPrint.cpp` 的 `validate_pad()`、`src/CLI/ProcessActions.cpp` 的支撐 STL 匯出區塊與 `catch(std::exception)`。
- **父 repo（`web_slicer_core`）**
  - `agent/tests/test_support_e2e.py` — Scenario 2 改為真引擎案例（驗收本變更的關鍵證據）。
  - `agent/support_classifier.py` — 僅新增模組 docstring 註解（說明 `(pad only)` marker 在非 zero-elevation 模式下不可達），**分類邏輯零變更**，降級後自動落進既有的中性分類路徑。
  - `agent/sla_operations.py`、`agent/jobs.py`、API 契約 — **零變更**。
  - `scripts/package_slicer_engine_windows.ps1` — 相容性修正（非本變更的行為需求，但為完成 Task 7 重編打包所必需）。原腳本僅認 `legal/slicer-engine/` 一條路徑與單一檔名，在本機環境下取不到 legal pack 而中斷打包。改為：canonical `legal\slicer-engine\` 優先，找不到時回退 `legal\slicer-engine-agpl\` 並發出明顯 WARN；檔名容忍 `NOTICE.md`/`NOTICE` 與 `SOURCE_OFFER.md`/`SOURCE-OFFER.md` 兩種寫法，但輸出一律正規化為 canonical 檔名；一併複製 `MODIFICATIONS.md`；掃描 `REPLACE_WITH_` 佔位符並警示該產物為 TEST-ONLY、不得出貨。兩個 legal 目錄的分工依 `docs/single-node-cloud/agpl-boundary.md` 維持不變，未做合併或重指向。
  - submodule 指標需前進至含本變更的 commit。
- **前端（`DS-online`）** — 本變更不觸及。Critical Angle 說明文案方向修正另由 `clarify-critical-angle-copy` 變更處理，兩者可獨立發版、無耦合。
- **相容性 / 行為風險**
  - GUI 行為變更：模型無支撐且啟用 pad 時，原本彈出「No pad can be generated…」錯誤，改為靜默不產生 pad。此為預期取捨。
  - 既有專案的支撐密度、pad 幾何、成功路徑輸出一律不變。
- **發版**：需重編 Windows（`build_win.bat` / `CMakePresets.json`）與 macOS（父 repo 的 `scripts/build_prusaslicer_fork_macos.sh`），並更換 Agent bundle。
- **驗收重點**：小船 -40° + Critical 90° + Pad ON → 中性成功；Pad OFF → 中性成功（回歸）；傾斜 40° 方塊 + Critical 90° + Pad ON → 中性成功；球體 + Critical 45° + Pad ON → `(includes supports and pad)` 且 `hasSupportMesh=true`（正常路徑未被吞掉）；有支撐但 pad 真失敗 → 仍為 `SUPPORT_GENERATION_FAILED`。