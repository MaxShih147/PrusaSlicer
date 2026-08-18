# sla-support-head-penetration Specification

## Purpose

定義 SLA 支撐頭（pinhead）與錨點（anchor）刺入模型的有效深度如何依承載面的局部可用深度動態夾限，以避免在薄件上貫穿至承載面另一側而在上表面留下凸點。

涵蓋：壁厚量測的幾何定義（沿頭軸、單位方向向量、射線起點偏移）、射線無命中時的 fail-safe 與其診斷、夾限相對於角度搜尋（optimizer）的施加時機，以及主頭與錨點兩類提交點在語意上的差異。

本能力採「上表面零凸點優先」政策，明確接受在極薄物件上因咬合深度不足而於列印時脫落的物理代價。當局部可用深度 ≥ 2 × 設定刺入深度時，夾限完全失效，常態件行為與未套用本能力時完全相同。

## Requirements

### Requirement: 支撐頭刺入深度必須依局部壁厚動態夾限

支撐頭的有效刺入深度 MUST 依承載面的局部可用深度動態夾限：

```
penetration = min(configured_penetration, local_thickness × 0.5)
```

其中 `configured_penetration` 為 `support_head_penetration`（Branching/Organic 樹為 `branchingsupport_head_penetration`），`local_thickness` 為沿支撐頭軸線量得的可用深度。

夾限結果 MUST 寫入該支撐頭的 `Head::penetration_mm` 成員，使所有依賴刺入深度的下游計算自動保持一致。系統 MUST NOT 以修改支撐頭網格生成邏輯的方式達成防貫穿。

本需求採「上表面零凸點優先」政策：明確接受在極薄物件上因咬合深度不足而於列印時脫落的物理代價。

#### Scenario: 刺入深度大於壁厚

- **GIVEN** 厚 0.2 mm 的水平薄板
- **AND** `support_head_penetration = 0.3`
- **WHEN** 於下表面放置支撐頭
- **THEN** 有效刺入深度 MUST 為 0.1（= 0.2 × 0.5）
- **AND** 支撐頭幾何 MUST NOT 突出於上表面之上

#### Scenario: 刺入深度小於壁厚一半

- **GIVEN** 厚 10 mm 的模型
- **AND** `support_head_penetration = 0.4`
- **WHEN** 於承載面放置支撐頭
- **THEN** 有效刺入深度 MUST 為 0.4（夾限失效，維持設定值）

#### Scenario: 支撐幾何與模型的布林交集驗證

- **GIVEN** 任意模型於任意 `support_head_penetration` 設定下完成支撐生成
- **WHEN** 將支撐網格與模型網格取布林交集，並檢查承載面另一側
- **THEN** 承載面另一側 MUST NOT 存在任何支撐幾何

---

### Requirement: 局部壁厚必須沿支撐頭軸線以單位方向向量量測

局部壁厚 MUST 沿支撐頭的軸線量測，入模方向定義為 `−Head::dir`。量測 MUST 使用射線查詢，且傳入的方向向量 MUST 為單位向量。

射線起點 MUST 自接觸點沿反入模方向退開一個極小量 `ε` 以避免自我命中，量得距離後 MUST 加回 `ε`。`ε` MUST 遠小於最薄可列印壁厚。

系統 MUST NOT 沿表面法線量測厚度——刺入深度定義於支撐頭軸線上，故軸向距離才是正確的可用深度。

#### Scenario: 傾斜支撐頭的軸向量測

- **GIVEN** 支撐頭軸線與承載面法線夾角不為零
- **WHEN** 量測局部可用深度
- **THEN** 量得的值 MUST 為沿頭軸的距離，而非垂直壁厚
- **AND** 夾限結果 MUST NOT 較垂直壁厚計算更為保守

#### Scenario: 射線自側緣出射

- **GIVEN** 傾斜的支撐頭，其軸線延伸後自薄板側緣出射而非自上表面出射
- **WHEN** 量測局部可用深度
- **THEN** 量得的值 MUST 為至側緣出射點的軸向距離
- **AND** 夾限後的支撐頭尖端 MUST 留在材料內部

#### Scenario: 方向向量必須正規化

- **WHEN** 對任一支撐頭或錨點執行壁厚量測
- **THEN** 傳入射線查詢的方向向量長度 MUST 為 1
- **AND** 此條件 MUST 在 Release 建置下亦成立，MUST NOT 僅依賴 debug 斷言

#### Scenario: 量測所用網格包含前置加工結果

- **GIVEN** 已啟用中空（hollowing）或已鑽排水孔的模型
- **WHEN** 量測局部可用深度
- **THEN** 量得的值 MUST 反映中空後的實際壁厚
- **AND** 鄰近排水孔的支撐點 MUST 量得縮減後的實際可用深度

---

### Requirement: 射線無命中時必須採 Fail-safe

封閉流形網格上，自表面內側向內射出的射線必定命中。當射線無命中時（破損網格、非流形或自交），系統 MUST 將該支撐頭的有效刺入深度設為 0。

當本階段有任何支撐頭觸發 fail-safe 時，系統 MUST 輸出彙總警告日誌，內容 MUST 包含觸發的支撐頭數量。

#### Scenario: 破損網格觸發 fail-safe

- **GIVEN** 含破面或非流形區域的模型
- **AND** 某支撐點的入模射線無命中
- **WHEN** 計算該支撐頭的有效刺入深度
- **THEN** 有效刺入深度 MUST 為 0

#### Scenario: Fail-safe 必須可診斷

- **WHEN** 支撐生成過程中有任何支撐頭觸發 fail-safe
- **THEN** 系統 MUST 輸出包含觸發數量的警告日誌
- **AND** 系統 MUST NOT 靜默地產出零咬合的支撐

---

### Requirement: 夾限必須於角度搜尋完成之後施加

刺入深度的夾限 MUST 僅執行一次，且 MUST 在該支撐頭的所有角度搜尋（optimizer）完成之後、最終物件提交的當下施加。

角度搜尋過程 MUST 全程使用 `configured_penetration`，MUST NOT 因夾限而改變搜尋行為或停止條件。

系統 MUST NOT 於 optimizer 的目標函式內執行壁厚量測射線。

#### Scenario: 搜尋行為不受夾限影響

- **GIVEN** 任一支撐頭或錨點的角度搜尋
- **WHEN** 執行 optimizer
- **THEN** optimizer 的初值、邊界與停止條件 MUST 以 `configured_penetration` 計算
- **AND** 搜尋所得的角度結果 MUST 與未套用夾限時完全相同

#### Scenario: 每個支撐頭僅量測一次

- **WHEN** 對單一支撐點完成支撐頭放置
- **THEN** 壁厚量測射線 MUST 僅執行一次
- **AND** MUST NOT 隨 optimizer 的評估次數增加

---

### Requirement: 主頭與錨點的夾限提交點語意不同

夾限的提交點依支撐頭類型而異，兩者 MUST NOT 互換。

**主頭（pinhead）**：提交點 MUST 位於 junction 與 pillar 計算之前。夾限 MUST 連帶更新 `fullwidth()` 與 junction 位置，使支撐頭與其 pillar 維持相連。

**錨點（anchor）**：提交點 MUST 位於錨點加入建構器（`add_anchor()`）的當下。此時橋接端點已固定，夾限 MUST NOT 改變任何橋接或拓撲結構。

對於 Branching/Organic 樹，夾限 MUST 於 `add_anchor()` 呼叫處施加，MUST NOT 於讀取 `junction_point()` 建立橋接端點之前施加。

#### Scenario: 主頭夾限後與 pillar 維持相連

- **GIVEN** 主頭的刺入深度被夾限
- **WHEN** 計算其 junction 與 pillar
- **THEN** junction 位置 MUST 依夾限後的 `fullwidth()` 重新計算
- **AND** 支撐頭與 pillar 之間 MUST NOT 存在幾何間隙

#### Scenario: Branching 樹錨點夾限不改變橋接端點

- **GIVEN** Branching/Organic 樹的錨點已完成角度搜尋
- **WHEN** 施加刺入深度夾限
- **THEN** 用於建立橋接端點的 junction 位置 MUST 與未夾限時相同
- **AND** 橋接的可行性檢查結果 MUST 與未夾限時相同

#### Scenario: 錨點網格與橋接端重疊而非脫開

- **GIVEN** 錨點的刺入深度被夾限至小於設定值
- **WHEN** 產生錨點網格
- **THEN** 錨點網格 MUST 涵蓋橋接端點所在位置
- **AND** 錨點與橋接之間 MUST NOT 存在幾何間隙

#### Scenario: 錨點寬度與夾限後的刺入深度一致

- **GIVEN** 錨點的寬度由包含刺入深度的距離導出
- **WHEN** 施加夾限
- **THEN** 錨點寬度 MUST 依夾限後的刺入深度重新計算

---

### Requirement: 局部壁厚充足時夾限必須完全失效

當局部可用深度大於或等於 `configured_penetration` 的兩倍時，夾限 MUST 不產生任何作用，支撐頭幾何 MUST 與未套用本能力時完全相同。

本需求的適用門檻為 `local_thickness ≥ 2 × configured_penetration`。壁厚落於 `(0, 2 × configured_penetration)` 區間的模型會被夾限影響，此為預期行為而非缺陷。

#### Scenario: 厚壁模型逐點一致

- **GIVEN** 所有支撐點的局部可用深度皆大於或等於 `2 × configured_penetration` 的模型
- **WHEN** 於變更前後分別執行支撐生成
- **THEN** 支撐網格 MUST 完全相同
- **AND** 每一個支撐頭的刺入深度 MUST 等於 `configured_penetration`

#### Scenario: 中間帶壁厚被夾限為預期行為

- **GIVEN** 局部壁厚 0.6 mm 的模型
- **AND** `support_head_penetration = 0.4`
- **WHEN** 放置支撐頭
- **THEN** 有效刺入深度 MUST 為 0.3
- **AND** 此結果 MUST 被視為符合規格，而非回歸

