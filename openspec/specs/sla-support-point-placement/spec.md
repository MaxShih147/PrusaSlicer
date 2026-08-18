# sla-support-point-placement Specification

## Purpose

定義 SLA 自動支撐點自 island 取樣層產生後，至送入支撐樹之前的放置行為：投影至模型表面時的方向性選擇、產出結果與切片網格相位的無關性、表面投影的位移上限、modifier（blocker / enforcer）過濾階段的邊界處理，以及模型無有效切片層時的錯誤回報。

本能力的核心不變量為：**支撐點必須落在朝下的承載面上，且產出結果不得因切片網格相位而改變**。極薄件（厚度接近或小於 `layer_height`）是這兩項最容易失效的情境，亦為本能力的主要適用對象。

## Requirements

### Requirement: 支撐點必須投影至朝下的承載面

自動產生的 SLA 支撐點由 island 取樣層的 `print_z` 建立後，投影至模型實際表面時，系統 SHALL 優先選擇**朝下的面**（命中三角形的幾何面法線 z 分量為負），而非幾何上最接近的面。

判定 MUST 使用命中面的幾何面法線，且 MUST NOT 依射線方向翻轉法線——自模型內側向上命中上表面時，其面法線仍為朝上，MUST 被排除。

當向上與向下兩個方向的命中面皆為朝下面時，系統 SHALL 選擇距離較近者。當兩者皆非朝下面（垂直壁或退化幾何）時，系統 SHALL 回退至「選擇距離較近者」的既有行為，以保證既有幾何類別的結果不變。

#### Scenario: 取樣層落在薄板中面之上

- **GIVEN** 一片厚 0.2 mm 的水平薄板，其下表面位於 z=0.00、上表面位於 z=0.20
- **AND** 支撐點建立於取樣層 z=0.125（位於中面 0.10 之上）
- **WHEN** 支撐點投影至模型表面
- **THEN** 支撐點 MUST 落在下表面 z=0.00
- **AND** 支撐點 MUST NOT 落在上表面 z=0.20，即使上表面在幾何距離上較近（0.075 < 0.125）

#### Scenario: 取樣層落在薄板中面之下

- **GIVEN** 同一片厚 0.2 mm 的薄板
- **AND** 支撐點建立於取樣層 z=0.075（位於中面 0.10 之下）
- **WHEN** 支撐點投影至模型表面
- **THEN** 支撐點 MUST 落在下表面 z=0.00

#### Scenario: 兩個方向皆非朝下面時回退既有行為

- **WHEN** 向上與向下的命中面法線 z 分量皆不為負
- **THEN** 系統 MUST 選擇距離較近的命中結果
- **AND** 投影結果 MUST 與未套用本需求前的結果一致

---

### Requirement: 支撐點產出必須與切片網格相位無關

同一模型在相同 `layer_height` 下，支撐點的數量與位置 MUST NOT 因切片網格相位改變而改變。切片網格原點由 `minZ = bb.min(Z) − get_elevation()` 決定，故 `support_object_elevation` 的小數部分會改變相位；此相位變化 MUST NOT 影響支撐點產出。

**適用範圍限於有效切片層相位。** 當某相位下切片網格中不存在任何涵蓋模型的層級時（層高相對於模型高度過大，實際條件約為模型高度 < 1.5 × `layer_height`），該相位在支撐點產生之前即於 `slice_model()` 階段中止。此類相位 MUST 由本需求的判準中排除，並改由「模型無切片層時的錯誤訊息必須指出真實成因」需求承接。本需求 MUST NOT 被解讀為要求消除此類無切片層相位。

#### Scenario: elevation 全相位掃描結果一致

- **GIVEN** 厚 0.2 mm 的薄板模型，`layer_height = 0.15`
- **WHEN** `support_object_elevation` 自 5.00 至 5.15 以 0.01 遞增逐一切片
- **THEN** 每一個**產生了有效切片層**的相位 MUST 產出相同數量的支撐點（19 點）
- **AND** 所有支撐點的 z 座標 MUST 皆為 0.00（下表面）
- **AND** 無有效切片層的相位（本組態下為 `elev = 5.13` 與 `5.14`，物件高 0.2 < 1.5 × 0.15 = 0.225）MUST 拋出指向層高的切片錯誤，且此結果 MUST NOT 計為本 Scenario 的失敗

#### Scenario: 模型僅橫跨單一切片層

- **GIVEN** `m_model_height_levels.size() == 1`
- **WHEN** 執行支撐點產生
- **THEN** 系統 MUST 產出與 `size() >= 2` 時相同的支撐點集合
- **AND** MUST NOT 產出零個支撐點

#### Scenario: 多種層高下的一致性

- **GIVEN** 厚 0.2 mm 的薄板模型
- **WHEN** 分別以 `layer_height` 0.05、0.10、0.15 切片
- **THEN** 每一種層高 MUST 產出支撐點且數量不為零

---

### Requirement: 表面投影位移上限必須為已定義的值

`move_on_mesh_surface()` 所使用的位移上限（`allowed_move`）MUST 由已初始化的資料導出。當 `m_model_height_levels` 的元素數量不足以計算層間距時，系統 MUST 使用設定的 `layer_height` 作為替代值，MUST NOT 讀取容器邊界之外的元素。

此需求的實作 MUST 在「支撐點投影至朝下承載面」需求之後施作。在該需求尚未實作前套用本需求，將使目前可正常運作的網格相位一併失效。

#### Scenario: 層級數量不足時使用設定層高

- **GIVEN** `m_model_height_levels.size() < 2`
- **WHEN** 計算 `allowed_move`
- **THEN** 系統 MUST 使用設定的 `layer_height` 值
- **AND** MUST NOT 存取 `m_model_height_levels[1]`

#### Scenario: 層級數量充足時維持既有計算

- **GIVEN** `m_model_height_levels.size() >= 2`
- **WHEN** 計算 `allowed_move`
- **THEN** 結果 MUST 等於 `m_model_height_levels[1] − m_model_height_levels[0]` 加上既有的 epsilon 項

---

### Requirement: Modifier 過濾階段不得無聲丟棄支撐點

依 blocker/enforcer 遮罩過濾支撐點時，若支撐點的 z 座標超出切片層級陣列的範圍而無法定位對應層級索引，系統 MUST 將索引夾限至最後一個有效層級，MUST NOT 直接捨棄該支撐點。

當本階段有任何支撐點被夾限或移除時，系統 MUST 輸出彙總診斷日誌，內容 MUST 包含受影響的點位數量、點位 z 範圍，以及切片層級陣列的首末值。

該日誌 MUST 採 `debug` 等級（`--loglevel 5` 可見），MUST NOT 採 `warning` 或更高等級。理由：越界情形現已被安全處理（點位被保留且正確評估遮罩），並非需要使用者介入的異常；升為 `warning` 會在 GUI 與主控台對常態切片產生無謂告警。此日誌的用途為開發者診斷，非使用者告警。

每次呼叫 MUST 最多輸出一行彙總，MUST NOT 逐點輸出。

#### Scenario: 支撐點 z 超出層級陣列範圍

- **GIVEN** 切片層級陣列的最末值為 0.125
- **AND** 存在 z 座標為 0.20 的支撐點
- **WHEN** 執行 modifier 過濾
- **THEN** 該支撐點 MUST 被保留
- **AND** 其遮罩判定 MUST 以最後一個有效層級為準

#### Scenario: 有點位受影響時輸出診斷

- **WHEN** modifier 過濾階段有任何支撐點的層級索引被夾限
- **THEN** 系統 MUST 輸出包含受影響筆數、z 範圍與層級陣列首末值的日誌
- **AND** 該日誌 MUST 於 `--loglevel 5`（debug）下可見
- **AND** 該日誌 MUST NOT 於預設日誌層級下出現

#### Scenario: 無點位受影響時不產生雜訊

- **WHEN** modifier 過濾階段所有支撐點皆正常定位
- **THEN** 系統 MUST NOT 輸出上述診斷日誌

---

### Requirement: 模型無切片層時的錯誤訊息必須指出真實成因

當切片網格中不存在任何涵蓋模型的層級而導致切片無法進行時，系統回報的錯誤訊息 MUST 指出實際成因為層高相對於模型高度過大，並 MUST 提供可行動的建議（調降層高）。

該訊息 MUST NOT 將成因歸因於網格破損或需要修復模型。

#### Scenario: 層高大於模型高度

- **GIVEN** 高度 0.5 mm 的模型
- **AND** `layer_height = 0.6`
- **WHEN** 執行切片
- **THEN** 錯誤訊息 MUST 指出層高相對模型高度過大
- **AND** 錯誤訊息 MUST NOT 包含網格破損或修復模型的敘述

---

### Requirement: 常態模型的支撐點產出必須維持不變

對於厚度除以 `layer_height` 大於或等於 2 的模型，本能力的所有變更 MUST NOT 改變支撐點的數量與位置。

**例外（正面修復，非回歸）**：因浮點尾數誤差而被 modifier 過濾階段誤殺的有效支撐點，於「Modifier 過濾階段不得無聲丟棄支撐點」需求生效後被救回。此類點數增加 MUST 被判定為符合規格，MUST NOT 計為本需求的違反。

判定準則：新增的點位 MUST 皆源自層級索引夾限（可由該階段的 debug 診斷日誌之受影響筆數核對），且新增筆數 MUST 等於該日誌回報的筆數。實測案例 `frog_legs.obj` 於 `cfg_base.ini` 下由 66 點增為 67 點，日誌回報 `1 of 67 support point(s) ... were clamped`，兩者相符，屬本例外涵蓋範圍。

本例外 MUST NOT 被擴大解釋：除上述來源外的任何點數或座標變動，仍 MUST 判定為回歸。

#### Scenario: 常態厚度模型逐點一致

- **GIVEN** 厚度除以 `layer_height` 大於或等於 2 的既有模型
- **AND** 該模型於 modifier 過濾階段無任何支撐點觸發層級索引夾限
- **WHEN** 於變更前後分別執行支撐點產生
- **THEN** 支撐點數量 MUST 完全相同
- **AND** 每一個支撐點的座標 MUST 完全相同

#### Scenario: 誤殺點位被救回屬正面修復

- **GIVEN** 厚度除以 `layer_height` 大於或等於 2 的既有模型
- **AND** 該模型有 N 個支撐點因浮點尾數誤差而觸發層級索引夾限
- **WHEN** 於變更前後分別執行支撐點產生
- **THEN** 支撐點數量 MUST 恰好增加 N 點
- **AND** 此變動 MUST 被判定為符合規格，MUST NOT 計為回歸

