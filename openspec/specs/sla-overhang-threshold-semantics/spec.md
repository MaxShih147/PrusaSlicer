# sla-overhang-threshold-semantics Specification

## Purpose
TBD - created by archiving change fix-empty-pad-slicing-error. Update Purpose after archive.
## Requirements
### Requirement: 懸空角度閾值的方向語意須維持既有實作

`support_critical_angle` 與 `branchingsupport_critical_angle`（UI label `Overhang threshold`）映射至 `SupportTreeConfig::overhang_angle_threshold`，支撐頭放置的過濾判準 SHALL 維持為「當法線極角 `polar < π/2 + overhang_angle_threshold` 時跳過該支撐點」。

此判準的方向語意為：**數值越小，生成的支撐越多**。0° 支撐所有朝下的面，90° 僅支撐法線正對下方的完全水平面。

本規格為刻意凍結的語意契約。任何改變此映射的修改 MUST 被視為破壞性變更，MUST 附帶遷移計畫，MUST NOT 以「修正錯誤」為由在不調整既有預設值的前提下逕行變更。

#### Scenario: 閾值越小支撐越多

- **WHEN** 同一模型分別以 `support_critical_angle` 為 0°、45°、90° 生成支撐
- **THEN** 產生的支撐柱數量呈遞減關係（0° ≥ 45° ≥ 90°）

#### Scenario: 閾值設為最大值時僅支撐水平朝下面

- **WHEN** `support_critical_angle` 設為 90°，且模型不含法線正對下方的完全水平面
- **THEN** 不產生任何支撐柱，support mesh 為空

#### Scenario: 反轉判準屬破壞性變更

- **WHEN** 有人提議將判準改為 `polar < π − overhang_angle_threshold`
- **THEN** 該提議 MUST 以破壞性變更處理——需說明此式僅在 45° 與現行式等值、其餘閾值全部鏡射為 `90° − t`，並同步規劃既有專案與預設值的遷移

### Requirement: 兩棵支撐樹的過濾判準須同向

Default tree 與 Branching/Organic tree 各自持有一份支撐頭放置邏輯。兩者 SHALL 使用相同的過濾判準與相同的方向語意，MUST NOT 出現單邊修改。

#### Scenario: 兩樹對相同閾值有一致方向

- **WHEN** 同一模型以相同的 critical angle 值分別用 Default tree 與 Branching tree 生成支撐
- **THEN** 兩者的支撐量隨閾值變化的方向一致（皆為數值越小支撐越多）

### Requirement: 說明文字須與實作方向一致

所有描述此參數的文字 SHALL 反映「數值越小，生成的支撐越多」。系統 MUST NOT 在任何位置宣稱「設為 90° 可支撐所有懸空面」或等義的相反敘述。

涵蓋範圍：參數定義的 tooltip、兩棵支撐樹過濾判準處的原始碼註解、`SupportTreeConfig::overhang_angle_threshold` 的欄位註解、以及設計文件中的閾值對照表。

#### Scenario: 參數 tooltip 描述正確方向

- **WHEN** 檢視 `support_critical_angle` 的 tooltip
- **THEN** 文字說明數值越小支撐越多，且不含「Set to 90° to support all overhangs」或等義敘述

#### Scenario: 過濾判準處的原始碼註解正確

- **WHEN** 檢視 Default tree 與 Branching tree 中過濾判準所在位置的註解
- **THEN** 兩處註解皆說明「0° 支撐所有懸空面、90° 僅支撐完全水平朝下面」，且兩處敘述一致

#### Scenario: 設計文件對照表與實作相符

- **WHEN** 檢視設計文件中閾值與效果的對照表
- **THEN** 表中所列公式與每一列的「效果」描述，皆可由 `polar < π/2 + threshold → 跳過` 直接推導得出

### Requirement: 既有支撐密度不得改變

本變更 MUST NOT 改變任何 critical angle 設定值所產生的支撐幾何。

支撐頭放置階段（`add_pinheads`）使用遺傳演算法優化器，其早停條件與平行排程使輸出在執行間存在變異。該變異**無法**透過固定 `SLA_GA_MAX_ITER` 消除——實測在固定迭代數下重複 8 次仍得到三種不同的三角形總數。因此驗證 MUST 採用分散式容差判準，MUST NOT 使用任何「完全相同 / 逐點一致」的措辭，亦 MUST NOT 以支撐 STL 的原始位元組雜湊（raw SHA-256）作為判準。

驗證 SHALL 為：對同一組參數重複執行**至少 5 次**，收集三角形總數的觀測集合，並要求

- **主判準**：變更前的基準值 MUST 落在變更後的觀測集合內；或觀測集合的極差（最大值減最小值）佔基準值的比例 MUST 小於 5%。
- **次判準**：以 `stl_diff.py` 比對時，差異三角形佔總數的比例 MUST 小於 5%。

#### Scenario: 常用設定值的輸出落在容差內

- **WHEN** 以 30°、45°、90° 三個常用設定值對同一模型生成支撐，每組重複執行至少 5 次，比對變更前後
- **THEN** 基準值落在觀測集合內，或觀測極差佔比小於 5%

#### Scenario: 固定 GA 迭代數不得被當成決定性保證

- **WHEN** 以固定的 `SLA_GA_MAX_ITER` 重複執行同一組參數
- **THEN** 三角形總數仍可能在多次執行間不同，該設定 MUST NOT 被當作可重現單一輸出的依據

#### Scenario: 原始位元組雜湊不得作為判準

- **WHEN** 對同一組參數以同一份二進位重複執行支撐生成
- **THEN** 支撐 STL 的 raw SHA-256 可能每次都不同，該值 MUST NOT 被用來判定支撐密度是否改變

#### Scenario: 內建 SLA profile 的設定值不變

- **WHEN** 檢視內建 SLA profile 中的 critical angle 設定
- **THEN** 其值維持變更前的內容，不因說明文字校正而調整

