# sla-pad-generation Specification

## Purpose
TBD - created by archiving change fix-empty-pad-slicing-error. Update Purpose after archive.
## Requirements
### Requirement: 無幾何可生成時 pad 步驟須優雅降級

當 `validate_pad()` 判定失敗，且同時滿足「pad mesh 為空」、「`supports_enable` 為真」、「support mesh 為空」三項條件時，pad 生成步驟 SHALL 視此為「無幾何可生成」而非錯誤：MUST NOT 拋出 `SlicingError`，MUST 將 pad mesh 清空，並讓該步驟正常標記完成。

三條件為合取關係，缺一不可。降級 MUST NOT 依賴例外訊息字串比對。

#### Scenario: 零支撐柱且啟用 pad

- **WHEN** 模型在目前參數下未產生任何支撐柱（support mesh 為空），且 `supports_enable = 1`、`pad_enable = 1`
- **THEN** pad 生成步驟不拋出例外，pad mesh 為空，切片流程繼續執行至結束

#### Scenario: 降級後的物件狀態與 pad 關閉時一致

- **WHEN** pad 生成步驟以降級路徑完成
- **THEN** pad 步驟被標記為已完成，且 pad mesh 為空——與 `pad_enable = 0` 分支所產生的物件狀態相同，不引入第三種中間態

#### Scenario: 降級留下可追溯的診斷紀錄

- **WHEN** pad 生成步驟以降級路徑完成
- **THEN** 系統輸出一筆 warning 等級的日誌，說明因無支撐柱而未生成 pad

### Requirement: 真實 pad 失敗須維持 fail-closed

除降級判準三條件同時成立的情況外，`validate_pad()` 判定失敗時系統 SHALL 一律拋出 `SlicingError`。本變更 MUST NOT 削弱任何既有的 pad 失敗偵測。

#### Scenario: 支撐存在但 pad 無法生成

- **WHEN** support mesh 非空，但 `validate_pad()` 判定 pad 無效
- **THEN** 系統拋出 `SlicingError`，切片流程中止

#### Scenario: 支撐未啟用但 pad 無法生成

- **WHEN** `supports_enable = 0`、`pad_enable = 1`，且 `validate_pad()` 判定 pad 無效
- **THEN** 系統拋出 `SlicingError`，維持變更前的行為

### Requirement: 降級後支撐匯出流程須完整執行並輸出既有 marker

降級發生時，`process()` MUST 正常結束，使 `--export-support-stl` 的支撐匯出區塊得以執行。當支撐與 pad 皆為空時，CLI SHALL 於 stdout 輸出既有字串 `No support/pad mesh generated`。

本變更 MUST NOT 新增、改寫或移除任何 CLI 輸出 marker。

#### Scenario: 降級後 stdout 輸出中性 marker

- **WHEN** pad 生成步驟以降級路徑完成，且以 `--export-support-stl` 執行
- **THEN** stdout 包含 `No support/pad mesh generated`，且 stderr 不含 `No pad can be generated for this model with the current configuration`

#### Scenario: 降級時不產生支撐 STL 檔案

- **WHEN** 支撐與 pad 皆為空而走降級路徑
- **THEN** 系統不寫出 `*_support.stl`，亦不輸出 `(supports only)`、`(pad only)` 或 `(includes supports and pad)` 任一 marker

#### Scenario: 正常支撐路徑不受影響

- **WHEN** 模型在目前參數下產生了支撐柱，且 `pad_enable = 1`
- **THEN** 支撐與 pad 合併後寫出 `*_support.stl`，stdout 輸出 `(includes supports and pad)`——與變更前完全相同

### Requirement: 降級結果須被下游分類為中性成功

降級所產生的 CLI 輸出，MUST 使父 repo 的支撐結果分類器判定為成功且中性：job 狀態為 `completed`、`support_outcome` 為 `SUPPORT_NOT_NEEDED`、`has_support_mesh` 為 `false`、`error_code` 為空。

分類器的**分類邏輯** MUST NOT 因本變更而修改——決策樹的判斷順序、marker 比對規則與 `support_outcome` 對應關係一律維持原狀，降級結果須自動落進既有的中性路徑。

本變更對 `agent/support_classifier.py` 僅新增模組 docstring 註解（說明 `(pad only)` marker 在非 zero-elevation 模式下不可達），**邏輯零變更**。此類純註解修改不違反本條要求。

#### Scenario: 小船旋轉後以高 Critical Angle 搭配 Pad 生成支撐

- **WHEN** 3DBenchy 繞 Y 軸旋轉 -40°、Critical Angle 設為 90°、勾選 Pad Generation，並執行支撐生成
- **THEN** job 狀態為 `completed`、`support_outcome` 為 `SUPPORT_NOT_NEEDED`、`has_support_mesh` 為 `false`、`error_code` 為空，前端顯示中性提示而非錯誤

#### Scenario: 相同模型取消 Pad Generation 的結果不變

- **WHEN** 同一模型與角度下取消勾選 Pad Generation 並執行支撐生成
- **THEN** 結果與勾選 Pad Generation 時完全一致（`completed` / `SUPPORT_NOT_NEEDED` / `has_support_mesh = false`）

#### Scenario: 非小船的零支撐模型搭配 Pad

- **WHEN** 一個繞 Y 軸傾斜 40° 的方塊在 `support_critical_angle = 90°`、`supports_enable = 1`、`pad_enable = 1` 下執行支撐生成（該組合實測為零支撐柱）
- **THEN** 結果為 `completed` / `SUPPORT_NOT_NEEDED`，而非 `SUPPORT_GENERATION_FAILED`

> **注意：軸對齊的平底模型不是零支撐模型。** 其底面法線為 `(0, 0, -1)`、極角 `polar == π`，即使 `support_critical_angle = 90°`（判準 `polar < π/2 + threshold = π`）仍會通過過濾而生成支撐柱，實測輸出 `(includes supports and pad)`。要構造零支撐情境**必須傾斜模型**。此誤解正是本缺陷長期未被測試覆蓋的原因，規格層不得再以「平底自支撐模型」作為零支撐的例證。

### Requirement: 既有的 pad 特例路徑不得受影響

匯入支撐（雙軌）與 zero-elevation（`pad_around_object`）兩條既有路徑的行為 SHALL 完全不變。

#### Scenario: 匯入支撐 mesh 時跳過原生 pad 生成

- **WHEN** 物件帶有匯入的支撐 mesh
- **THEN** pad 生成步驟在進入 `pad_enable` 判斷之前即早退，不執行任何 pad 生成或降級邏輯

#### Scenario: zero-elevation 模式下空 pad 仍屬合法

- **WHEN** `pad_around_object` 啟用且未強制 everywhere，pad mesh 為空
- **THEN** `validate_pad()` 依既有規則判定為有效，不進入失敗分支，亦不觸發降級判準

