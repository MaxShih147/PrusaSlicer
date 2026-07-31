# Task 7：跨平台建置與封裝（Windows）

擷取時間：2026-07-30T08:49:42Z（UTC）

## Build tree 規則（已釐清）

開發流程為：於 `third_party/prusaslicer_fork/build/` 編譯 → **手動複製** `slicer_core.dll` 與二進位檔覆蓋至 `third_party/prusaslicer_build/`（Agent 的讀取路徑）。兩者為獨立的 CMake build tree，非 symlink。

## 7.1 / 7.2 建置與同步

| 檔案 | `fork/build` | `prusaslicer_build` | 狀態 |
|---|---|---|---|
| `slicer_core.dll` | `2c59148f…` 16:12:49 | `2c59148f…` 16:12:49 | ✅ 同步 |
| `slicer-engine.exe` | `38a4b100…` 14:55:35 | `38a4b100…` 14:55:35 | ✅ 同步 |

- 兩處 `src/Release/` 目錄下**所有檔案位元組全數一致**。
- `find src -newer slicer_core.dll` 為空 → DLL 已涵蓋全部原始碼變更（含第 5 章文字校正）。
- `slicer-engine.exe` 時間戳較舊屬正常：它是薄啟動器，邏輯全在 `slicer_core.dll`。

### 7.2 驗證（以 `prusaslicer_build` 引擎）

| 項目 | 結果 |
|---|---|
| 4.3 主目標（A：pad ON, 90°） | `exit=0`、stdout `No support/pad mesh generated`、stderr 空、無 STL ✅ |
| 4.5 正常路徑（E：pad ON, 45°） | `(includes supports and pad)`、44,028 tri（與基準相同）✅ |
| B（pad OFF, 90°） | `exit=0`、`No support/pad mesh generated`、無 STL ✅ |

## 7.3 / 7.4 macOS — 未執行

本機為 Windows，無法執行 `scripts/build_prusaslicer_fork_macos.sh` 與後續驗證。**待於 macOS 機器補做**。

## 7.5 Windows bundle 封裝 — 部分完成 ⚠️

執行 `scripts/package_slicer_engine_windows.ps1`（預設參數：來源 `third_party/prusaslicer_build/src/Release`、輸出 `<RepoRoot>/slicer-engine`）。

**二進位已成功落地並驗證**（與 build tree 位元組一致）：

| 檔案 | staged | build tree |
|---|---|---|
| `slicer-engine.exe` | `38a4b100…` | `38a4b100…` ✅ |
| `slicer_core.dll` | `2c59148f…` | `2c59148f…` ✅ |
| `OCCTWrapper.dll` | `9c625228…` | `9c625228…` ✅ |

共 1,012 個檔案（含 `resources/`）。

**腳本在 AGPL legal pack 步驟中止**，屬**既有的路徑不符**，與本次變更無關：

| 腳本期望（`$RepoRoot\legal\slicer-engine`） | repo 實際（`legal/slicer-engine-agpl/`） |
|---|---|
| `NOTICE.md` | `NOTICE`（無副檔名） |
| `SOURCE_OFFER.md` | `SOURCE-OFFER.md`（連字號） |

差異涵蓋目錄名、副檔名與連字號/底線三處。AGPL 合規檔案未擅自改名，打包腳本亦未修改——需另行決定以哪一側為準。

因此 `slicer-engine/` 目前為**未含 legal pack 的部分 bundle**，可供測試但**不可出貨**。

> ⚠️ **副作用**：`agent/config.py` 的引擎解析第一順位即為 `slicer-engine/bin/slicer-engine.exe`。此目錄產生後，Agent 已由開發建置路徑切換到此 bundle 路徑。

## 7.6 以 bundle 引擎路徑重跑驗證 ✅

引擎：`slicer-engine/bin/slicer-engine.exe`（非開發建置路徑）

| 項目 | 結果 |
|---|---|
| 4.3 主目標（A：pad ON, 90°） | `exit=0`、stdout `No support/pad mesh generated`、stderr 空、無 STL ✅ |
| 正常路徑（E：pad ON, 45°） | `(includes supports and pad)`、44,028 tri ✅ |
| Agent Python E2E | `pytest agent/tests/test_support_e2e.py` → **7 passed**（經 bundle 路徑）✅ |

打包後行為與開發建置路徑完全一致。

## ⚠️ 4.6 判準需再次修訂（重要）

上一輪將判準改為「**固定 GA 迭代下**三角形總數與檔案大小完全相同」，依據是「`SLA_GA_MAX_ITER=20` 精確重現基準值 46,528」。本輪發現**該依據為單次取樣，結論不成立**。

以 D 組（pad OFF、45°）在 `SLA_GA_MAX_ITER=20` 下重複 8 次：

| 三角形數 | 次數 |
|---|---|
| 44,992 | 6 |
| 46,528（基準值） | 1 |
| 45,184 | 1 |

**固定迭代數並不會消除隨機性。** 基準值仍落在分布內（1/8），故仍無回歸；但「完全相同」在任何 GA 設定下都無法作為判準。

已觀測到的 D 組取值集合：{44,992、45,184、46,528}，最大離散度 1,536 tri = **3.30%**，亦超過上一輪訂的 0.5% 次判準容差。

**建議改為分布式判準**（尚未修改 spec，待指示）：

1. **主判準**：重複執行 N 次（建議 ≥ 5），基準值必須落在觀測取值集合內，或觀測極差 < 5%。
2. **次判準**：以 `stl_diff.py` 比對，差異三角形比例 < 5%。
3. 移除任何「完全相同 / 逐點一致」的措辭——本引擎在支撐頭放置階段使用隨機優化器，該性質無法透過設定消除。

需連帶修訂：
- spec `sla-overhang-threshold-semantics`：需求「既有支撐密度不得改變」與其三個 scenario
- tasks `1.4`、`4.6`
- `evidence/after-.../AFTER.md` 的「證據二」段落（其單次取樣結論須標註為已被推翻）

**不影響修正本身的結論**：D 組差異與本次變更無關的決定性理由仍是「`pad_enable=0` 時新增程式碼一行都不執行」，該論證不依賴任何幾何比對。