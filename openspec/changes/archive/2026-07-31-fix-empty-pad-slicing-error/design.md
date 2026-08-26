## Context

`SLAPrint::Steps::generate_pad()` 在 `pad_enable` 開啟時呼叫 `create_pad()`，並以 `validate_pad()` 檢查結果：

```cpp
bool validate_pad(const indexed_triangle_set &pad, const sla::PadConfig &pcfg)
{
    return !pad.empty() || (pcfg.embed_object.enabled && !pcfg.embed_object.everywhere);
}
```

`embed_object` 來自 `pad_around_object`（zero-elevation 模式）。Agent 的 `SLAConfig` 並未提供該欄位，因此在產品路徑上 `embed_object.enabled` 恆為 false，判準退化為 `!pad.empty()`——**pad 一空就必然拋出 `SlicingError`**。

非 zero-elevation 模式下 pad 的 footprint 來自支撐樹底部。當支撐樹為空（零支撐柱）時 pad 必然為空，於是這個「幾何上無事可做」的狀態被當成錯誤處理。例外在 `process()` 階段拋出，被 `src/CLI/ProcessActions.cpp` 的 `catch (const std::exception&)` 接住後 `return false`，**支撐 STL 匯出區塊完全不會執行**：

```
process()                                   ProcessActions.cpp
  └ generate_pad() ── throw ──┐
                              └──▶ catch → cerr → return false
                                      ╳ 跳過：export_support_stl 區塊
                                        ╳ 跳過：combined_mesh 組裝
                                        ╳ 跳過：marker 輸出
```

下游的 Python 分類器（父 repo `agent/support_classifier.py`）以 stdout / stderr 文字判定結果，五步決策樹的 Step 3/4 全靠 CLI 印出的 marker。marker 沒印出、STL 沒落地，分類器只能走 Step 5 fail-closed，回報 `SUPPORT_GENERATION_FAILED`。

**約束條件（已由決策定案）**：不得修改支撐角度過濾公式，30°/45°/90° 的實測支撐密度必須逐點不變；不得改動 Python 分類器與 Agent API 契約；不得新增或改名任何設定項。

## Goals / Non-Goals

**Goals:**

- 讓「零支撐柱 + Pad 開啟」與「零支撐柱 + Pad 關閉」收斂到同一個中性終點（`SUPPORT_NOT_NEEDED`）。
- 消除「pad 失敗連坐丟棄已生成支撐」的結構缺陷。
- 維持 fail-closed：真正的 pad 失敗（有支撐卻長不出 pad）仍必須是錯誤。
- Python 與前端零程式碼變更——降級後自動落進既有的中性分類路徑。
- 把 `support_critical_angle` 的真實語意方向以文件與 spec 釘死，避免日後被誤當缺陷反向修正。

**Non-Goals:**

- 不修正 `overhang_angle_threshold` 的比較公式（見決策 5）。
- 不調整 `resources/profiles/*.ini` 的 `critical_angle = 90`。
- 不移除 `"(pad only)"` marker，不新增任何 CLI 輸出字串。
- 不新增錯誤碼、不擴充 Agent API 回應欄位。
- 不改動前端預設值與 Critical Angle 滑桿範圍（文案修正屬 `clarify-critical-angle-copy`）。

## Decisions

**決策 1：修復點落在 `SLAPrint::Steps::generate_pad()`，而非 Python 分類器或 CLI catch 區。**

三個候選層各自的資訊可得性：

| 層 | 看得到 pad 為空？ | 看得到 support mesh 為空？ | 判定 |
|---|---|---|---|
| `generate_pad()` | ✔ 直接持有 `pad_mesh` | ✔ `po.support_mesh()` | **採用** |
| `ProcessActions.cpp` catch | ✘ 只有泛型 `std::exception` | ✘ 例外已中止 `process()` | 否決 |
| Python `support_classifier.py` | ✘ stdout 無 marker | ✘ STL 未落地 | 否決 |

- 替代方案（否決）：**Python 層攔截 stderr 的 `"No pad can be generated"` 字串並翻成 `SUPPORT_NOT_NEEDED`**。該層在物理上無法區分「零支撐導致 pad 空」與「真實 pad 參數錯誤」，必然一刀切；且該字串在其他情境（pad 參數不合理、hollowing 後模型破碎、`pad_around_object`）也會出現，會吞掉真實錯誤。此外直接違反分類器既有的「validate 錯誤一律 fail-closed」設計原則。
- 替代方案（否決）：**在 CLI `catch` 後改為不中止、繼續執行匯出區塊**。`catch` 接住的是所有 `std::exception`，放行等於讓任何切片錯誤都繼續往下跑，blast radius 遠大於本問題。

**決策 2：降級判準採三條件合取，而非單看 pad 是否為空。**

```
validate_pad() == false
        │
        ├─ pad_mesh 空  ∧  supports_enable  ∧  support_mesh 空
        │        └──▶ 降級：pad_mesh = {}，步驟正常完成，不拋例外
        │
        └─ 其餘一切情況
                 └──▶ 維持 throw SlicingError（fail-closed 不被削弱）
```

`supports_enable` 條件的作用是把降級限制在「使用者要求生成支撐、但一根柱子也沒長出來」這個唯一情境。

- `po.m_config.supports_enable.getBool()` 與 `po.support_mesh()` 在此步驟皆可直接取用——前一個步驟 `support_tree` 結尾已在用 `po.support_mesh().empty()` 做 log，存取路徑既有、無需新增。
- 替代方案（否決）：**只用「pad 空 ∧ support mesh 空」兩條件**。這會連帶涵蓋「`supports_enable = 0` 但 `pad_enable = 1`」的組合（GUI 才會出現，產品路徑不會），把該情境原本的錯誤訊號改成靜默。防禦性修補應取最小 blast radius，故加上 `supports_enable` 作為守門。
- 替代方案（否決）：**只用「pad 空」單一條件**。等同讓 `validate_pad` 失敗永不報錯，直接廢掉 fail-closed。

**決策 3：降級路徑不新增任何輸出字串，完全複用既有 marker。**

不拋例外後，`slaposPad` 被標記為 done、`pad_mesh` 為空，CLI 匯出區塊的既有判斷自然導出正確結果：

```
ProcessActions.cpp
  is_step_done(slaposSupportTree) ✔ → support_mesh.empty() → has_support = false
  is_step_done(slaposPad)         ✔ → pad_mesh.empty()     → has_pad     = false
  combined_mesh.empty()           ✔ → 印出 "No support/pad mesh generated"
        │
        ▼
agent/support_classifier.py  Step 3 → COMPLETED / SUPPORT_NOT_NEEDED
```

這條路徑與 Pad OFF 情境（已由 job `14e952f2` 實測驗證）完全重合。

- 替代方案（否決）：**在 C++ 印一個新的專屬 marker（如 `"Pad skipped: no supports"`）**。需要 Python 分類器同步新增比對字串、擴大跨 repo 耦合，且會讓兩個語意相同的狀態產生兩條分類路徑。零新增字串才能保住「Python 分類邏輯零變更」這個目標（該檔最終僅新增模組 docstring 註解，決策樹未動）。

**決策 4：降級時將 `pad_mesh` 顯式清空，與 `pad_enable = false` 分支的狀態一致。**

`generate_pad()` 的 `else if (po.m_supportdata)` 分支本來就會執行 `po.m_supportdata->pad_mesh = {}`。降級路徑採相同處置，使「pad 開啟但無事可做」與「pad 關閉」在後續步驟（`slice_supports` 的 gate、`pad_mesh()` 存取、GUI 場景重繪）看到的物件狀態完全一致，不引入第三種中間態。

`has_imported_support()` 的早退（雙軌匯入支撐路徑）位於本判斷之前，不受影響。

**決策 5：不修正角度公式，改以 spec 釘住既有語意。**

`support_critical_angle`（label `"Overhang threshold"`）的實作判準為 `polar < π/2 + threshold → skip`，實際效果是**數值越小、支撐越多**；而 tooltip、`DefaultSupportTree.cpp` 註解、`SupportTree.hpp` 註解與 `doc/sla_support_angle_refactoring.md` 的對照表，四處都描述成相反方向。

- 替代方案（否決）：**把判準改為 `polar < π − threshold`**。該式與現行式僅在 threshold = 45° 時等值，其餘全部鏡射（等效閾值 `90° − t`）。修正後所有非 45° 的既有設定支撐密度都會改變——前端預設 30° 會明顯變少、fork profile 的 90° 會劇變。已決策優先保障既有專案行為不變。
- 因此本變更僅校正說明文字，並新增 `sla-overhang-threshold-semantics` capability 將「數值越小支撐越多」寫成規格，同時把「現行式與 `π − threshold` 的差異」記錄在設計文件中，讓未來任何反向修正都必須是一次帶遷移計畫的明確決策，而非被當成 bug 順手改掉。

**決策 6：測試分兩層，各自覆蓋不同的失效模式。**

| 層 | 位置 | 覆蓋 |
|---|---|---|
| 單元（Catch2） | 本 repo `tests/sla_print/` | 零支撐 + `pad_enable` → 不拋例外、`pad_mesh` 空；有支撐但 pad 失敗 → 仍拋例外 |
| 端對端（pytest） | 父 repo `agent/tests/test_support_e2e.py` | 真引擎跑完整 CLI → stdout marker → 分類器 → job status = `SUPPORT_NOT_NEEDED` |

父 repo 的 Scenario 2 目前以 stub CLI 人工餵入 `"(pad only)"`。該字串在現行 agent 組態下**不可達**（`embed_object` 恆為 false ⇒ `validate_pad` 退化為 `!pad.empty()` ⇒ 零支撐必拋例外），這正是本缺陷從未被測到的根本原因。必須改為真引擎案例，否則修復無法被驗證。

- 「有支撐但 pad 失敗」的反向案例實機難以重現（pad footprint 來自支撐樹，有柱子時 pad 幾乎不可能為空），故安排在 Catch2 層以直接構造的 mesh 覆蓋，不強求 E2E。

## Risks / Trade-offs

- **[GUI 行為改變]** 支撐啟用但零支撐柱、且 pad 開啟時，PrusaSlicer GUI 原本彈出「No pad can be generated…」錯誤，改為靜默不產生 pad。
  → 緩解：此為決策 2 已知且接受的取捨；`generate_pad` 降級時保留 `BOOST_LOG_TRIVIAL(warning)` 記錄，diagnostics 仍可追。CLI 使用者不受影響——匯出區塊會明確印出 `"No support/pad mesh generated"`。

- **[降級可能遮蔽未來的真實缺陷]** 若日後支撐樹因其他原因意外變空，本修改會讓它表現為中性成功而非錯誤。
  → 緩解：中性結果在前端仍是可見訊號（「No support needed」），且 `has_support_mesh = false` 如實回填、不會偽造支撐存在；搭配 warning log 可回溯。

- **[真引擎 E2E 測試需要 fixture]** 現有 `_export_sphere` 產生的球體需要支撐，無法觸發零支撐情境。
  → 緩解：新增**傾斜方塊** fixture（程式生成、繞 Y 軸 40°）搭配 Critical Angle 90°。父 repo 既有的 `_needs_engine` 標記可在無引擎環境自動 skip，不阻塞 CI。
  → ⚠️ 實作階段修正：原稿寫的「平底自支撐 fixture」不成立——軸對齊方塊的底面 `polar == π`，閾值 90° 仍會生支撐。實際落地為 `_export_tilted_box()`（`agent/tests/test_support_e2e.py`）與 Catch2 的 `CubePadPrint{rot_y_deg = 40}`。

- **[跨 repo 同步]** 本變更同時涉及 submodule 與父 repo 測試，順序錯誤會導致測試對著舊引擎跑而假性通過或失敗。
  → 緩解：明確順序——先合本 repo C++ 變更 → 雙平台重編 → 父 repo submodule 指標前進 → 再啟用真引擎測試。

- **[雙平台重編成本]** 需重編 Windows（`build_win.bat` / `CMakePresets.json`）與 macOS（父 repo `scripts/build_prusaslicer_fork_macos.sh`），並更換 Agent bundle。
  → 緩解：變更範圍極小（單一函式的一個分支 + 註解），無新依賴、無 ABI 影響，回滾即還原 submodule 指標。

- **[角度語意維持已知錯位]** 決策 5 選擇保留與文件相反的實作方向，等於長期背負一個「反直覺參數」——滑桿最大值 90° 是支撐最少的一端。
  → 緩解：以 spec 明文釘住、tooltip 與註解全面校正方向；前端說明文案由 `clarify-critical-angle-copy` 變更補上「數值越小，生成的支撐越多」。