## Why

SLA 自動支撐點在 `support_points_density_relative` 過高時，slicer-engine 會以 Stack Overflow 當掉（Windows 結束碼 `3221225725`）。實測環境為 `support_head_front_diameter = 0.6`、50×50×10 mm 方塊、`--export-support-points`：

- 密度 **2204%** 正常（min_distance 為 0 → 1609 點；`support_points_min_distance = 1.5` → 1039 點）
- 密度 **2205%** 當掉，且**無論是否設定 `support_points_min_distance` 皆崩潰**

### 根因：`verify()` 的互斥約束 × 無窮遞迴

`SampleConfigFactory::apply_density()` 以 `thin_max_distance = 原值 / density` 線性縮小間距（記為 `t`），隨後 `verify()` 對 `thin_max_width` 同時施加兩條約束：

```
verify_min:  thin_max_width ≥ 2·r + 2·m      (r = head_radius, m = minimal_distance_from_outline)
verify_max:  thin_max_width ≤ 2·t + 2·r

兩者可同時成立  ⟺  t ≥ m
```

當 `t < m` 時兩條約束互斥，`thin_max_width` 每輪在兩個值之間來回跳動，`SampleConfigFactory.cpp:51` 的 `if (!res) while (!verify(cfg));` 永不收斂，遞迴耗盡堆疊。

> 注意：真正的可行性條件是 `t ≥ m`，**不是** `t ≥ head_radius`。兩者目前相等，僅因 `create()` 將 `m` 設為 `head_radius`；以 `head_radius` 為防線，在 `m` 被調整後將重新引入同一缺陷。

崩潰門檻為 `t₀ / m`（`t₀` 為 `create()` 產出的 `thin_max_distance`），隨頭徑而異：

```
頭徑 (mm)    0.3     0.4     0.5     0.6     0.8     1.0
最後正常 %   3130    2596    2333    2204    2151    2232
```

0.6 mm 為實測；其餘為依原始碼 float/double/截斷語意的離線模擬，尚待以真實 build 驗證。

### 外部 min_distance 無法攔截

`SLAPrintSteps.cpp` 約 1012 行的呼叫順序為 `apply_min_distance(apply_density(create(...)))`，崩潰發生在 `apply_density()` 內部的 `verify()`，`apply_min_distance()` 根本沒有機會將間距拉回。

### 同族隱患：卡死與記憶體耗盡

`support_points_density_relative` 僅設 `min = 0`、**無上限**；`support_head_front_diameter` 亦允許 0。在更極端的輸入下：

- **卡死（Hang）**：間距截斷為 0 時，`UniformSupportIsland.cpp` 的 `sample_outline()` 以 `sample_distance = 0` 驅動的 `while` 迴圈永不結束。
- **記憶體耗盡（OOM）**：頭徑為 0 時 `m = 0`，間距可縮至 µm 級；粗估 50 mm 方塊即產生約 60 萬取樣點，再經 Voronoi cell 建構與 30 輪對齊，計算量與記憶體不可控。

### 附帶缺陷：Debug build 在 > 205% 即中斷

`verify()` 內第 7、15、24 行的 `assert` 斷言「輸入已合法」，但密度約超過 205% 起 `verify()` 本就必須修正數值，故 Debug build 在此區間必定 abort。這使高密度路徑**在 Debug 下從未被執行過**，亦會阻斷本變更的極端密度測試。

## What Changes

- **飽和密度防禦（`apply_density()`）**
  - 保留原本的 float 除法算式計算 `t_original`，型別與運算順序一字不改。
  - 以 `m_thresh = max(minimal_distance_from_outline, minimal_move)` 為門檻（`create()` 中 `minimal_move = 0.1 mm`）。
  - `t_original ≥ m_thresh`：走原路徑，三個間距的計算與既有程式碼完全相同。
  - `t_original < m_thresh`（飽和路徑）：**只有 `thin_max_distance` 飽和**，直接設為 `m_thresh`。`thick_inner_max_distance` 與 `thick_outline_max_distance` 照原公式隨密度繼續縮小，各加一個防零下限：
    - `thick_inner_max_distance ≥ max(頭直徑, minimal_move)`（仍維持開根號縮放）
    - `thick_outline_max_distance ≥ minimal_move`
  - 兩個 thick 下限**只在飽和路徑套用**。若在原路徑也套用，0.2 mm 頭的 `thick_outline` 約在 4069% 就低於 0.1 mm，但 thin 要到約 4340% 才飽和，中間這段原本正常的結果會被改到。
  - 0.6 mm 頭的目標值：3100% 時 `thick_inner` 約 1.48 mm，4200% 時約 1.27 mm（離線驗算為 1.485 mm、1.276 mm）。
  - `m_thresh` 只用於分支判斷與 thin 的飽和值，**不寫回** `minimal_distance_from_outline`。

- **驗證收斂重構（`verify()`）**
  - 於 `.cpp` 匿名 namespace 抽出 `verify_once()`：執行原有 7 條檢查一輪，不遞迴，未修改任何值時回傳 `true`。
  - `verify()` 改為「首輪 + 至多 20 輪」的有限迴圈；回傳值維持原語意（首輪是否未修改）。
  - 超過上限仍未收斂時拋出 `CodedRuntimeError`（`SUPPORT_POINT_SAMPLING_FAILED`），取代 Stack Overflow。
  - 原遞迴等價於「重複單輪檢查直到某輪未修改任何值」，且回傳 `true` 的一輪必然未修改任何值，故迴圈改寫對合法輸入位元等級不變。

- **除錯斷言遷移（`create()`）**
  - 移除 `verify()` 內第 7、15、24 行已過時的 `assert`。
  - 在 `create()` 改以兩行式斷言保留原意圖（預設值必須自洽）：`[[maybe_unused]] const bool clean = verify(result); assert(clean);`。不可寫成 `assert(verify(...))`，否則 Release build 將連同 `verify()` 一併移除。

- **測試覆蓋（`tests/sla_print/`）**
  - 測試內嵌舊版的 `create()`、`apply_density()`、`apply_min_distance()` 與**遞迴 `verify()`**（去除 assert）作為對照，整條舊鏈對整條新鏈，於頭徑 0.2～1.5 mm（8 種）× 各頭徑崩潰門檻前的密度網格上，逐欄比對新舊結果位元一致。網格須先判定 legacy 的 `t ≥ m` 再呼叫，避免測試本身 Stack Overflow。
  - 預設值自洽：0～1.5 mm 共 10 種頭徑的原始預設值（尚未經過 `verify()`）必須首輪即通過 `verify()`，且與 `create()` 的結果逐欄相同。
  - 極端輸入：5000%、10000%、`INT_MAX`% 密度與 0 mm 頭徑，函式正常返回；`thin_max_distance` 不小於 `m_thresh`，兩個 thick 間距不小於各自的下限，7 條幾何約束直接斷言成立。
  - 無例外掃描：10 種頭徑 × 0%～`INT_MAX`% × `min_distance` 0.15／1.5 mm，皆不得拋出例外。
  - thick 持續縮小：0.6 mm 頭在 3100%、4200% 時，`thick_inner_max_distance` 分別約為 1.48 mm、1.27 mm。
  - 例外路徑：手動建構互斥配置（如 `m > t`）傳入 `verify()`，須捕捉到 `CodedRuntimeError`。
  - 端到端整合驗證：50×50×10 mm 方塊、0.6 mm 頭，新舊引擎同參數比對。2204% 匯出檔逐位元組相同；2205% 以上新引擎正常結束、舊引擎崩潰。

- **相容性**
  - 頭徑 ≥ 0.2 mm（`m ≥ minimal_move`）時，凡原本未崩潰的密度輸入，`SampleConfig` 結果位元等級不變。
  - **刻意的行為改變**：頭徑 < 0.2 mm 時，`t` 落在 `[m, 0.1 mm)` 的密度區間原本不崩潰，改為 thin 於 0.1 mm 飽和，`thick_outline` 也受 0.1 mm 下限約束（例：0.1 mm 頭約自 4127% 起，原崩潰點約 8254%）。此為防 OOM 的取捨，非回歸。
  - Release build 行為僅於上述兩類輸入改變；Debug build 另移除了會中斷執行的 assert。
  - 不新增、不變更任何設定參數、預設值、preset 或 UI；無 **BREAKING**。

## Capabilities

### New Capabilities
- `sla-island-sample-config`: 規範 SLA island 取樣配置（`SampleConfig`）由頭徑與相對密度導出時的穩定性契約——任何密度與頭徑輸入皆必須在有限步內產出合法配置或回報明確錯誤，不得 Stack Overflow、卡死或記憶體耗盡；規範高密度下 `thin_max_distance` 的飽和門檻，以及兩個 thick 間距照原公式縮小、但不低於防零下限的行為；規範頭徑 ≥ 0.2 mm 時原本未崩潰輸入的位元等級相容性，以及頭徑 < 0.2 mm 的刻意行為改變邊界。

### Modified Capabilities
<!-- 無。既有 `sla-support-point-placement` 的範圍明定為「自 island 取樣層產生後，至送入支撐樹之前」的放置行為；本變更位於其上游的取樣配置導出階段，該規格的需求不受影響。 -->

## Impact

**切片器（prusaslicer_fork）**

- [src/libslic3r/SLA/SupportIslands/SampleConfigFactory.cpp](../../../src/libslic3r/SLA/SupportIslands/SampleConfigFactory.cpp) — 唯一修改的原始碼檔：`verify()`、`create()`、`apply_density()`；新增 `#include <libslic3r/EngineErrorCodes.hpp>`（同在 libslic3r 內，無新外部依賴）。
- `tests/sla_print/` — 新增測試；若為新檔，需同步加入 [tests/sla_print/CMakeLists.txt](../../../tests/sla_print/CMakeLists.txt) 的來源清單（或併入既有的 `sla_supptgen_tests.cpp` 以免動 CMake，於 design 決定）。

**明確不修改**

- `SampleConfigFactory.hpp`：`verify()` 簽名不變（仍回傳 `bool`，但現在可能拋出例外）。
- `SLAPrintSteps.cpp`：呼叫鏈與順序不變；例外沿既有 `CodedRuntimeError` 管道由 CLI 透過 `EngineErrorCarrier` 取得錯誤碼。
- 工作區既有未 commit 修改：`.gitignore`、`src/libslic3r/CMakeLists.txt`、`src/slic3r/CMakeLists.txt`。

**下游行為**

- 密度超過門檻時，island 的細長區域（thin）點數飽和、不再隨密度增加；粗區域（thick）的外框與內部點數仍隨密度增加，直到觸及下限。0.6 mm 頭的 `thick_outline` 約自 6201% 起停在 0.1 mm，`thick_inner` 約自 18988% 起停在 0.6 mm。非 island 區域的取樣（`SupportPointGenerator.cpp` 約 501、518 行）不受影響，仍隨密度繼續變密。
- `web_slicer_core` `agent/`：沿用既有 `SUPPORT_POINT_SAMPLING_FAILED`，`agent/error_codes.py` **無需修改**。正常輸入下此錯誤路徑不應被觸發。
- GUI：`GLGizmoSlaSupports.cpp` 直接呼叫 `verify()` 的位置位於已停用的 `USE_ISLAND_GUI_FOR_SETTINGS` 區塊內，目前不受影響；若日後啟用，GUI 執行緒須處理新的例外。

**已知殘留風險**

- thick 下限擋得住「間距變 0」造成的卡死，但只把點數上限壓到「每 0.1 mm 一點（外框）、每 max(頭直徑, 0.1 mm) 一格（內部）」。頭徑 < 0.2 mm 在極端密度下，內部格點間距會停在 0.1 mm，50 mm 方塊約 25 萬點，仍可能很慢或吃掉大量記憶體。0.6 mm 頭停在 0.6 mm 時，50 mm 方塊約 7 千點。這兩個數字都是粗估，沒有實測。
- 20 輪上限僅有離線模擬佐證（合法區間最多 4 輪），無形式化收斂證明；上限加例外即為此風險的保險。
- 最初回報的 1609／1039 點來自一條已遺失的 CLI 指令，無法重現，因此端到端驗證改為新舊引擎同參數逐位元組比對。多執行緒下點數的穩定性未另行驗證；本次兩次執行（新舊各一）的 2204% 輸出完全相同。
- `NDEBUG` 行為依 CMake 預設（Debug 不定義；Release／RelWithDebInfo 定義）。prusaslicer_fork 與上層 repo 的頂層 CMake 未見覆寫，但上層其餘 CMake 檔未全面檢查。

**與上游的關係**

`SampleConfigFactory.cpp` 的 `verify()` 遞迴與 `apply_density()` 縮放邏輯繼承自 upstream PrusaSlicer。未來 rebase 時需檢查上游是否已修正，避免衝突或重複修補。
