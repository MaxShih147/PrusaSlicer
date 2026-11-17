# sla-island-sample-config Specification

## Purpose

定義 SLA 自動支撐點的 island 取樣配置（`SampleConfig`）由支撐頭直徑與相對密度（`support_points_density_relative`）導出時的穩定性契約，範圍為 `SampleConfigFactory` 的 `create → apply_density → apply_min_distance` 呼叫鏈，以及其中的 `verify()` 約束修正。

本能力的核心不變量為：**任何密度與頭徑輸入都必須在有限步內得到合法配置或明確的錯誤碼，不得 Stack Overflow 或卡死；頭徑 ≥ 0.2 mm 時，原本不會崩潰的輸入結果必須位元等級不變。** 高密度下 `thin_max_distance` 於飽和門檻停止縮小，`thick_inner_max_distance` 與 `thick_outline_max_distance` 依原公式持續縮小，直到觸及防零下限。這些下限讓取樣間距不會變成 0，但只限制點數密度，不保證極端輸入下不會耗用大量記憶體（例如頭徑 < 0.2 mm 搭配極高密度時，內部格點間距停在 0.1 mm）。

本能力不處理 `coord_t`（int32）的數值溢位：極大頭徑、超低密度或極大 `support_points_min_distance` 可能使 `verify()` 的上限計算溢位，此時以 `SUPPORT_POINT_SAMPLING_FAILED` 結束。

## Requirements

### Requirement: 取樣配置導出必須在有限步內終止

`SampleConfigFactory` 由頭徑與相對密度導出 island 取樣配置（`create → apply_density → apply_min_distance`）時，對任何 `support_points_density_relative`（0 至 `INT_MAX`）與任何 `support_head_front_diameter ≥ 0` 的組合，系統 SHALL 在有限步內返回，MUST NOT 發生 Stack Overflow、無窮迴圈或無窮遞迴。

`verify()` MUST 以有限迴圈實作：首輪檢查之後，至多再執行 20 輪。`verify()` MUST NOT 以遞迴方式重跑檢查。

`verify()` 的回傳值 MUST 維持原語意：首輪未修改任何欄位時回傳 `true`，否則回傳 `false`。

#### Scenario: 原崩潰密度不再崩潰

- **GIVEN** `support_head_front_diameter = 0.6`
- **AND** `support_points_density_relative = 2205`
- **WHEN** 依序呼叫 `create`、`apply_density`、`apply_min_distance`
- **THEN** 三個函式 MUST 皆正常返回
- **AND** MUST NOT 拋出例外

#### Scenario: 極端密度不崩潰也不卡死

- **GIVEN** `support_head_front_diameter` 為 0、0.1、0.6 之一
- **AND** `support_points_density_relative` 為 5000、10000、`INT_MAX` 之一
- **WHEN** 依序呼叫 `create`、`apply_density`、`apply_min_distance`
- **THEN** 每一種組合 MUST 皆正常返回
- **AND** 結果的 7 條幾何約束 MUST 成立：`max_length_for_one_support_point < max_length_for_two_support_points`、`thick_min_width < thin_max_width`、`2r + 2m ≤ max_length_for_one_support_point ≤ 2t + 2r + 2m`、`max_length_for_two_support_points ≤ 2t + 4r + 2m`、`2r + 2m ≤ thin_max_width ≤ 2t + 2r`（t = `thin_max_distance`、r = `head_radius`、m = `minimal_distance_from_outline`）

#### Scenario: 頭徑為 0 的一般密度

- **GIVEN** `support_head_front_diameter = 0`
- **AND** `support_points_density_relative = 100`
- **WHEN** 導出取樣配置
- **THEN** 系統 MUST 正常返回

---

### Requirement: 無法收斂時必須回報明確錯誤

當 `verify()` 在首輪加 20 輪之後仍未收斂，系統 MUST 拋出 `CodedRuntimeError`，錯誤碼 MUST 為 `SUPPORT_POINT_SAMPLING_FAILED`。系統 MUST NOT 將未收斂的配置交給下游取樣演算法。

此錯誤 MUST 經由既有的 `EngineErrorCarrier` 管道被 CLI 取得。系統 MUST NOT 為此新增錯誤碼。

在 `create`、`apply_density`、`apply_min_distance` 的正常呼叫鏈下，此錯誤路徑 MUST NOT 被觸發。

#### Scenario: 互斥配置觸發錯誤

- **GIVEN** 由 `create(0.6)` 產出的配置
- **AND** 將其 `thin_max_distance` 手動設為 `minimal_distance_from_outline / 2`
- **WHEN** 呼叫 `verify()`
- **THEN** 系統 MUST 拋出例外
- **AND** 該例外 MUST 可轉型為 `EngineErrorCarrier`
- **AND** 其錯誤碼 MUST 為 `SUPPORT_POINT_SAMPLING_FAILED`

#### Scenario: 正常呼叫鏈不觸發錯誤

- **GIVEN** 頭徑 `{0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5}` mm 中的任一值
- **AND** 任一整數百分比密度
- **AND** `support_points_min_distance` 為 0、0.15 或 1.5 mm
- **WHEN** 依序呼叫 `create`、`apply_density`、`apply_min_distance`
- **THEN** 系統 MUST NOT 拋出 `SUPPORT_POINT_SAMPLING_FAILED`

---

### Requirement: 高密度下 thin 間距必須飽和，thick 間距必須有防零下限

`apply_density()` MUST 先以原本的算式計算三個縮放後的間距，記為 `t_original`、`inner_original`、`outline_original`。三者的型別與運算順序 MUST 與修改前相同：thin 與 outline 為 int32 ÷ float，inner 為 double 開根號（`sqrt(x² / density)`）。

飽和門檻 MUST 以下式計算：

```
m_thresh = max(minimal_distance_from_outline, minimal_move)
```

當 `t_original ≥ m_thresh` 時，系統 MUST 把三個原始值原樣寫入，結果 MUST 與修改前相同。此路徑 MUST NOT 套用任何下限。

當 `t_original < m_thresh` 時（飽和路徑）：

- `thin_max_distance` MUST 設為 `m_thresh`，不再隨密度縮小。
- `thick_inner_max_distance` MUST 為 `max(inner_original, max(2 × head_radius, minimal_move))`。它 MUST 照原本的開根號公式隨密度繼續縮小，直到觸及下限。
- `thick_outline_max_distance` MUST 為 `max(outline_original, minimal_move)`。它 MUST 照原本的線性公式隨密度繼續縮小，直到觸及下限。

`m_thresh` 與兩個 thick 下限 MUST NOT 寫回 `minimal_distance_from_outline` 或任何其他欄位。

#### Scenario: 超過門檻後 thin 間距不再縮小

- **GIVEN** `support_head_front_diameter = 0.6`
- **WHEN** 密度分別為 2205%、5000%、10000%
- **THEN** 三種密度的 `thin_max_distance` MUST 皆等於 `minimal_distance_from_outline`（0.3 mm）
- **AND** 三種密度的 `thick_inner_max_distance` MUST 隨密度增加而嚴格變小

#### Scenario: thick_inner 照原公式繼續縮小

- **GIVEN** `support_head_front_diameter = 0.6`
- **WHEN** 密度為 3100%
- **THEN** `thick_inner_max_distance` MUST 約為 1.48 mm（±0.01 mm）
- **AND** 密度為 4200% 時，`thick_inner_max_distance` MUST 約為 1.27 mm（±0.01 mm）
- **AND** 兩者 MUST 等於 `static_cast<coord_t>(std::sqrt(sqr((double) 原 thick_inner_max_distance) / density))` 的計算值

#### Scenario: thick 間距觸及下限

- **GIVEN** `support_head_front_diameter = 0.6`
- **AND** `support_points_density_relative = INT_MAX`
- **WHEN** 導出取樣配置
- **THEN** `thick_outline_max_distance` MUST 等於 `minimal_move`（0.1 mm）
- **AND** `thick_inner_max_distance` MUST 等於頭直徑（0.6 mm）

#### Scenario: 頭徑為 0 時三個間距都有下限

- **GIVEN** `support_head_front_diameter = 0`
- **AND** `support_points_density_relative = INT_MAX`
- **WHEN** 導出取樣配置
- **THEN** `thin_max_distance`、`thick_outline_max_distance`、`thick_inner_max_distance` MUST 皆不小於 0.1 mm
- **AND** `minimal_distance_from_outline` MUST 維持為 0

#### Scenario: 下限只在飽和路徑套用

- **GIVEN** `support_head_front_diameter = 0.2`
- **AND** 密度 4100%（此時 `outline_original` 已低於 0.1 mm，但 `t_original ≥ m_thresh`）
- **WHEN** 導出取樣配置
- **THEN** 所有純量欄位 MUST 與修改前相同
- **AND** `thick_outline_max_distance` MUST 小於 0.1 mm

---

### Requirement: 頭徑 ≥ 0.2 mm 時原本未崩潰的輸入結果必須位元等級不變

對於頭徑 ≥ 0.2 mm、且修改前不會崩潰的任何密度輸入，修改後產出的 `SampleConfig` 每一個純量欄位 MUST 與修改前位元等級相同。純量欄位包括全部 13 個 `coord_t` 欄位、`max_length_ratio_for_two_support_points`、`count_iteration` 與 `simplification_tolerance`。

「修改前不會崩潰」的判定 MUST 以修改前的 float 算式算出的 `thin_max_distance ≥ minimal_distance_from_outline` 為準，MUST NOT 以預先寫死的密度百分比為準。各頭徑的門檻不同（例：0.4 mm 約 2596%，0.6 mm 為 2204%）。

此相容性 MUST 同時涵蓋 `apply_min_distance` 介入的路徑。

#### Scenario: 逐欄位元一致的網格比對

- **GIVEN** 測試內嵌修改前的 `legacy_create()`、`legacy_apply_density()` 與遞迴版 `legacy_verify()`（去除 assert）作為對照，整條舊鏈（含 `create`）對整條新鏈
- **AND** 頭徑 `{0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5}` mm × 自 10% 起逐 1% 遞增的整數百分比密度（0.2 mm 時 `head_radius` 與 `minimal_move` 相等，是相容性承諾的邊界）
- **AND** 每一點的 legacy `thin_max_distance ≥ minimal_distance_from_outline`
- **WHEN** 分別以新舊實作導出配置
- **THEN** 所有純量欄位 MUST 以 `==` 比對相等

#### Scenario: 含 min_distance 的網格比對

- **GIVEN** 與上一個 Scenario 相同的網格
- **AND** 兩邊都再呼叫 `apply_min_distance(…, 1.5)`
- **WHEN** 比對新舊結果
- **THEN** 所有純量欄位 MUST 以 `==` 比對相等

#### Scenario: legacy 對照的遞迴保護

- **GIVEN** 測試內嵌的 `legacy_verify()` 遞迴深度超過 50 層
- **WHEN** 網格邊界判斷有誤，導致呼叫了不收斂的輸入
- **THEN** 該測試案例 MUST 以失敗結束
- **AND** 測試程序 MUST NOT Stack Overflow

#### Scenario: 端到端整合驗證（手動）

- **GIVEN** 50×50×10 mm 平放方塊、`--printer-technology SLA`、`support_head_front_diameter = 0.6`、`support_pillar_diameter = 1.0`、`--export-support-points`
- **AND** 修改前（a114d13bd）的引擎與修改後的引擎使用完全相同的參數
- **WHEN** 以密度 2204%、`support_points_min_distance` 為 0 與 1.5 分別執行 CLI
- **THEN** 新舊引擎的匯出檔 MUST 逐位元組相同，且結束碼皆為 0
- **AND** 以密度 2205%、5000%、10000% 執行時，新引擎 MUST 正常結束（結束碼 0）並匯出支撐點，舊引擎則以 Stack Overflow 結束（`-1073741571`，即 3221225725）
- **AND** 此 Scenario 屬手動整合驗證，不是阻斷門檻；位元相容性以上述網格比對為準

最初回報的 1609／1039 點來自一條已遺失的 CLI 指令，參數與本 Scenario 不同，因此不作為比對值。2026-10-12 以引擎預設參數實測，2204% 新舊皆為 909／908 點且匯出檔逐位元組相同。

---

### Requirement: 頭徑 < 0.2 mm 的行為改變必須限於飽和區間

頭徑 < 0.2 mm 時，`minimal_move`（0.1 mm）大於 `minimal_distance_from_outline`，所以 `m_thresh = minimal_move`。`t_original ∈ [minimal_distance_from_outline, minimal_move)` 的密度區間在修改前不會崩潰，修改後 MUST 走飽和路徑：thin 飽和，兩個 thick 間距套用下限。這是為了防止記憶體耗盡而刻意做的行為改變，MUST NOT 被判定為回歸。

`t_original ≥ minimal_move` 的密度區間，結果 MUST 與修改前位元等級相同。

#### Scenario: 小頭徑低密度維持不變

- **GIVEN** `support_head_front_diameter = 0.1`
- **AND** 密度使 `t_original ≥ 0.1 mm`
- **WHEN** 導出取樣配置
- **THEN** 所有純量欄位 MUST 與修改前相同

#### Scenario: 小頭徑高密度改為飽和

- **GIVEN** `support_head_front_diameter = 0.1`
- **AND** 密度使 `minimal_distance_from_outline ≤ t_original < 0.1 mm`
- **WHEN** 導出取樣配置
- **THEN** `thin_max_distance` MUST 等於 0.1 mm
- **AND** `thick_outline_max_distance` MUST 不小於 0.1 mm
- **AND** 此結果與修改前不同，MUST 被判定為符合規格

---

### Requirement: create() 的預設值必須自洽

對每一種頭徑，`create()` 產出的配置 MUST 在 `verify()` 首輪即通過，不需要任何修正。

`create()` 必須在所有編譯組態下都呼叫 `verify()`。斷言 MUST 寫成「先存回傳值、再 assert」，MUST NOT 把 `verify()` 呼叫直接寫在 `assert()` 內。

`verify()` 內部 MUST NOT 含有「斷言輸入已合法」的 assert。原第 7、15、24 行的 assert MUST 移除。

#### Scenario: 標準頭徑首輪通過

- **GIVEN** 頭徑為 `{0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5}` mm 之一
- **AND** `create()` 尚未經過 `verify()` 的原始預設值
- **WHEN** 對原始預設值呼叫 `verify()`
- **THEN** `verify()` MUST 回傳 `true`（首輪即未修改任何欄位）
- **AND** 原始預設值 MUST 與 `create()` 的回傳值逐欄相同

#### Scenario: 高密度路徑不因 assert 中止

- **GIVEN** 密度 300%（此時 `verify()` 需要修正數值）
- **WHEN** 導出取樣配置
- **THEN** 系統 MUST NOT 因 assert 而中止
- **AND** 本次變更以 Release 驗證此 Scenario；Debug 與 RelWithDebInfo 的驗證暫緩或略過，理由見「不同編譯組態的計算結果必須一致」

---

### Requirement: 不同編譯組態的計算結果必須一致

在 Debug 與 Release／RelWithDebInfo 下，相同輸入產出的 `SampleConfig` MUST 完全相同。兩者的差別 MUST 僅限於 Debug 多執行的 assert 檢查。

本能力的所有自動化測試 MUST 在 Release 組態下執行並通過。Release 是本次變更唯一的驗收環境。

**RelWithDebInfo 驗證略過**（2026-10-12 決定）：RelWithDebInfo 與 Release 同樣定義 `NDEBUG`，本能力的計算邏輯在兩者之間等價；而其建置產物為舊版，需整個重新編譯 libslic3r，成本高、預期效益低。因此本次變更不以 RelWithDebInfo 測試作為驗收條件。日後若重建 RelWithDebInfo，SHALL 補跑本能力的測試。

**Debug 驗證暫緩**：目前第三方相依套件只建置了 Release 版（`deps/build` 的 `DEP_DEBUG=OFF`），Debug 組態無法連結，因此本次變更不以 Debug 測試作為驗收條件。設計上 Debug 與 Release 的計算結果仍 MUST 一致（兩行式 assert 保證 `verify()` 在所有組態都會執行）。日後若以 `DEP_DEBUG=ON` 重建相依套件，SHALL 補跑 Debug 測試。

#### Scenario: Release 組態執行測試

- **WHEN** 在 Release 組態下執行 `sla_print_tests "[SampleConfig]"`
- **THEN** 所有測試案例 MUST 皆通過
