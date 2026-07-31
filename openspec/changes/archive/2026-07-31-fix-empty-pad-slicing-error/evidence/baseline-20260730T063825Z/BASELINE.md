# Before 基準（Task 1：基準鎖定）

擷取時間：2026-07-30T06:38:25Z（UTC） / 平台：Windows

## 1.1 基準二進位

| 項目 | 值 |
|---|---|
| 路徑 | `third_party/prusaslicer_build/src/Release/slicer-engine.exe` |
| SHA-256 | `f794ea7eeafb982c0241628323682c353abb3ec7fcfc2ccec8a21aff68b6b157` |
| 大小 / 時間 | 164,352 bytes / 2026-07-27 14:56 |
| fork HEAD | `9dee8f6be87db316c87b5cc50d38e07343f2654a`（branch `release/v1.0.5`） |

**未重新建置的理由**：`src/` 下除 `platform/msw/PrusaSlicer.rc.in`（Windows 資源樣板，與 pad / 支撐邏輯無關）外，沒有任何檔案的 mtime 新於此二進位；工作區改動僅 `.gitignore`、`src/libslic3r/CMakeLists.txt`、`src/slic3r/CMakeLists.txt`。此二進位即為產生本案兩份重現 job（`agent/jobs/bb65f4d6`、`agent/jobs/14e952f2`）的同一份執行檔，A / B 兩組輸出已逐字比對確認（見 1.5）。

## 共用輸入

| 項目 | 值 |
|---|---|
| 模型 | `agent/jobs/bb65f4d6/input/model.stl`（3DBenchy，已烘焙 Y 軸 -40° 旋轉） |
| SHA-256 | `aca56d24bf43ffe2515c96afb00d67a004479ff683a32d3c94ebb34d53cb2ec4` |
| 備註 | 與 `agent/jobs/14e952f2/input/model.stl` 位元組完全相同 |

## 執行方式

複製 `agent/sla_operations.py::generate_supports` 的命令列與環境：

```
LC_ALL=C LANG=C LANGUAGE=C \
  slicer-engine.exe --export-support-stl \
    --output <run>/output/model.sl1 \
    --load <run>.ini \
    <input>/model.stl
```

各 `.ini` 由 `agent/jobs/bb65f4d6/config.ini` 僅改 `pad_enable` 與 `support_critical_angle` 產生，其餘欄位（含 `layer_height = 0.15` 的支撐偵測層高）保持不變。

## 各組結果

| Run | pad_enable | critical_angle | exit | 支撐 STL | 三角形數 | 檔案大小 | stdout 結尾 marker |
|---|---|---|---|---|---|---|---|
| **A-pad-on-90** | 1 | 90.0 | **1** | 無 | — | — | 無（停在 `Generating pad`），stderr = `No pad can be generated for this model with the current configuration` |
| **B-pad-off-90** | 0 | 90.0 | 0 | 無 | — | — | `No support/pad mesh generated` |
| **C-pad-off-30** | 0 | 30.0 | 0 | 有 | 47,936 | 2,396,884 | `(supports only)` |
| **D-pad-off-45** | 0 | 45.0 | 0 | 有 | 46,528 | 2,326,484 | `(supports only)` |
| **E-pad-on-45** | 1 | 45.0 | 0 | 有 | 44,028 | 2,201,484 | `(includes supports and pad)` |

> **E 組為 1.1–1.5 範圍外的額外基準**，供 tasks 4.5「正常路徑（支撐 + pad）未被吞掉」作 before 對照——該對照只能在修改程式碼前取得。

支撐密度隨角度遞減（30° 47,936 > 45° 46,528），與 spec `sla-overhang-threshold-semantics`「數值越小支撐越多」一致。

### 檔案 SHA-256（僅供存查，不可作為回歸判準——見下節）

```
4261264d5c5fad3421e9198eed52126bcf7ebd1ce2cc3d2ecc9a71e6f6241ee0  C-pad-off-30/output/model_support.stl
7edc29e16db604399e3b468be0a46d73004717b8e859bd377dbc145e72e107fb  D-pad-off-45/output/model_support.stl
2164bcf465cdf64eabf4ce251fe908533dde44e7b5198c736345e96d79ab740b  E-pad-on-45/output/model_support.stl
```

### 排序後 canonical digest（`canonical_stl_digest.py`）

```
d30bffcc20fa103237778838b27c65c69d3797c1062856007a6631f83fd8014e  tri=47936  C-pad-off-30
173168ad8979094d3034046bf6079f23399ade8981b5724c26f887b31009273e  tri=46528  D-pad-off-45
8f8ca74084a04ceaac89a63feb0bee85e489bd450a8aabda1095d139a233661c  tri=44028  E-pad-on-45
```

## ⚠️ 決定性調查（影響回歸判準）

以 C 組參數連續執行 5 次：

| 指標 | 結果 | 可用性 |
|---|---|---|
| 三角形數 / 檔案大小 | 47,936 / 2,396,884，**5/5 完全相同** | ✅ 可作回歸判準 |
| 檔案 raw SHA-256 | **每次幾乎都不同**（三角形寫出順序隨平行排程變動） | ❌ 不可用 |
| 排序後 canonical digest | 4/5 相同，1 次不同 | ⚠️ 需容差 |

對兩份 digest 不同的輸出做集合比對：三角形總數相同（47,936），其中 **128 個三角形不同（0.267%）**，其餘 47,808 個完全一致。即引擎幾何**近乎但非完全決定性**，少量支撐元件的位置或細分會在不同執行間微幅改變。

**對 tasks 的影響**：`1.4` 與 `4.6` 原訂以 SHA-256 逐一比對的方法不成立，需改判準。建議：

1. **主判準（嚴格）**：三角形數與檔案大小完全相同。
2. **次判準（幾何）**：canonical digest 相同即通過；若不同，以 `stl_diff.py` 比對，差異三角形比例須 < 0.5%。
3. 若次判準落在容差邊緣，重跑 3 次取多數。

## 附帶觀察（非本案範疇）

- A 組 exit code 為 **1**（非 0）。分類器刻意不採信 exit code（`support_classifier.py` 設計 D1 針對的是 `SLAPrint::validate()` 回 exit 0 的情形），與本案結論無衝突，僅記錄。
- 所有組別 stdout 皆含一行 `[error] Detected missing Voronoi vertex even after the rotation of input.`，A/B/C/D/E 一致出現，與 pad 無關，屬既有現象。

## 檔案清單

```
BASELINE.md                     本文件
canonical_stl_digest.py         排序後 digest 工具
stl_diff.py                     三角形集合差異比對工具
runs/<name>.ini                 各組設定
runs/<name>/stdout.log          
runs/<name>/stderr.log          
runs/<name>/result.txt          exit code 與耗時
runs/<name>/output/             產出（含 model_support.stl，若有）
```