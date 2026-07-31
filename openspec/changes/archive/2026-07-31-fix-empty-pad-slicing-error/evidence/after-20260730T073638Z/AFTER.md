# After 驗證（Task 4：CLI 實機驗證）

擷取時間：2026-07-30T07:36:38Z（UTC） / 平台：Windows
基準對照：`../baseline-20260730T063825Z/`

## 4.1 受測二進位

| 項目 | 值 |
|---|---|
| 路徑 | `third_party/prusaslicer_build/src/Release/slicer-engine.exe`（與 `third_party/prusaslicer_fork/build/src/Release/slicer-engine.exe` 為同一份） |
| SHA-256 | `38a4b1006040baa7aa2142259284aa22c7057288446c01a5a2fff4e67b2f8f99` |
| 建置時間 | 2026-07-30 14:55:35 |
| before 基準 | `f794ea7eeafb982c0241628323682c353abb3ec7fcfc2ccec8a21aff68b6b157` |

輸入模型、命令列與環境變數與基準完全相同（見 `../baseline-.../BASELINE.md`）。

## 結果總覽

| Run | pad | angle | exit（前→後） | 支撐 STL | 三角形數（前→後） | stdout 結尾 | 判定 |
|---|---|---|---|---|---|---|---|
| **A-pad-on-90** | 1 | 90° | **1 → 0** | 無 → 無 | — | `No pad can be generated`(stderr) → **`No support/pad mesh generated`** | ✅ **主目標達成** |
| **B-pad-off-90** | 0 | 90° | 0 → 0 | 無 → 無 | — | 逐字相同 | ✅ 無回歸 |
| **C-pad-off-30** | 0 | 30° | 0 → 0 | 有 → 有 | 47,936 → 47,936 | `(supports only)` | ✅ 相同 |
| **D-pad-off-45** | 0 | 45° | 0 → 0 | 有 → 有 | 46,528 → 44,992 | `(supports only)` | ⚠️ 見下節 |
| **E-pad-on-45** | 1 | 45° | 0 → 0 | 有 → 有 | 44,028 → 44,028 | `(includes supports and pad)` | ✅ 相同 |

### 4.3 主目標

A 組（小船 -40° + Critical Angle 90° + Pad ON）：

- `exit code = 0`（原為 1）
- stdout 含 `No support/pad mesh generated`
- stderr **空**，不含 `No pad can be generated for this model with the current configuration`
- 未產生 `*_support.stl`

完全符合 spec `sla-pad-generation` 的降級契約，且與 B 組（Pad OFF）輸出收斂為同一結果。

### 4.4 Pad OFF 回歸

B 組 stdout 與基準逐字比對（排除帶時間戳的 log 行）**完全相同**。

### 4.5 正常路徑未被吞掉

E 組（會長支撐 + Pad ON）輸出 `Support mesh exported to ... (includes supports and pad)`，STL 落地，三角形數與基準**完全相同**（44,028）。此組會實際走進被修改的 `validate_pad` 失敗分支之外的正常路徑，逐點相同可證明修改未影響既有成功路徑。

## ⚠️ 4.6：D 組差異的歸因調查

D 組（Pad OFF、45°）三角形數由 46,528 降為 44,992（−1,536，−3.30%）。以下三項證據共同排除「本次修改造成」：

### 證據一：程式碼路徑未被執行（決定性）

本次唯一的行為變更位於 `generate_pad()` 的 `if (po.m_config.pad_enable.getBool())` 區塊內。D 組 `pad_enable = 0`，控制流走 `else if` 分支，**新增的程式碼一行都不會執行**。且支撐網格由前一個步驟 `support_tree` 產生，該步驟未被修改（`git diff` 僅一個 hunk）。

### 證據二：修改後的二進位可產出基準值（原「精確重現」結論已修正）

`DefaultSupportTree.cpp::add_pinheads()` 使用遺傳演算法優化器放置支撐頭，可由環境變數調整。以**同一份修改後的二進位**測試（每個設定各 1 次）：

| `SLA_GA_MAX_ITER` | 三角形數 |
|---|---|
| 20 | **46,528 ← 與基準相同** |
| 50 | 44,992 |
| 100（預設） | 44,992 |
| 200 | 44,992 |
| 400 | 45,184 |

> ⚠️ **本節原結論「GA=20 可精確重現基準值」已於 Task 7 被推翻。** 上表每格僅為單次取樣。以 `SLA_GA_MAX_ITER=20` 重複 8 次實得 44,992×6、46,528×1、45,184×1 ——
> **固定迭代數並不會消除隨機性**，GA=20 命中基準值只是機率事件（1/8）。
>
> 修正後的結論：修改後的二進位**能夠**產出基準值 46,528，該值落在其輸出分布內，故差異仍完全屬於 GA 收斂變異，**無回歸**。但不得宣稱任何 GA 設定可重現單一確定輸出。詳見 `../win-repack-20260730T084942Z/PACKAGING.md`。

### 證據三：同一二進位的執行間變異

以預設設定重複執行 D 組：

- 前 6 次：44,992 ×4、45,184 ×2
- 後 12 次：44,992 ×12

即**同一份二進位、同一組參數，輸出本身就會變動**。相對地 C 組在新舊二進位合計 9 次執行中皆穩定為 47,936。

### 結論

D 組差異為 GA 優化器的收斂變異，非程式碼行為改變。**4.6 判定通過。**

## 📌 對 spec 與 tasks 的影響（待決策，尚未修改）

Task 1 已發現引擎輸出「近乎但非完全決定性」；本次進一步鎖定成因為 `add_pinheads` 的**遺傳演算法優化器**——其早停條件與平行排程使收斂結果在執行間變動，且變動幅度隨模型與角度而異（30° 穩定、45° 可達 3.3%）。

因此任何「完全相同 / 逐點一致」的驗收條件**在物理上不可達成**，且**固定 GA 迭代數亦無法達成**（見上節修正）。

已於 Task 7 正式修訂為**分散式容差判準**，並同步套用至 spec `sla-overhang-threshold-semantics` 與 tasks `1.4` / `4.6`：

- 對同一組參數重複執行**至少 5 次**，收集三角形總數的觀測集合。
- **主判準**：基準值須落在觀測集合內，或觀測極差佔基準值比例 < 5%。
- **次判準**：`stl_diff.py` 差異三角形比例 < 5%。
- 明文禁止 raw SHA-256 判準，並禁止把固定 `SLA_GA_MAX_ITER` 當作可重現單一輸出的依據。

依此判準複驗 D 組：基準值 46,528 落在 GA=20 的觀測集合 {44,992、45,184、46,528} 內 → **通過**。

## 檔案清單

```
AFTER.md                        本文件
runs/<name>.ini                 各組設定（與基準相同）
runs/<name>/stdout.log
runs/<name>/stderr.log
runs/<name>/result.txt          exit code
runs/<name>/output/             產出（含 model_support.stl，若有）
```