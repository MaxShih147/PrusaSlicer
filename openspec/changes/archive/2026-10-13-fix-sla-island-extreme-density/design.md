## Context

island 取樣配置由 `SLAPrintSteps.cpp` 約 1012 行一次組出：

```
create(head_diameter)              ── 依頭徑導出預設值，內呼 verify()
   └─► apply_density(cfg, density) ── 縮小三個間距，內呼 verify()   ◄── 2205% 在此 Stack Overflow
          └─► apply_min_distance(cfg, min_distance) ── 拉高三個間距，內呼 verify()
```

`density` 來自 `float(support_points_density_relative / 100.f)`，`support_points_density_relative` 為 `coInt`、`min = 0`、無上限；`support_head_front_diameter` 亦允許 0，且 `SLAPrint::validate()` 不擋。

`verify()` 現況為 7 條「夾限即回報 false」的檢查，任一條修改數值就以 `while (!verify(cfg));` 遞迴重跑。可行性條件推導如下（`t` = `thin_max_distance`、`r` = `head_radius`、`m` = `minimal_distance_from_outline`）：

```
thin_max_width ∈ [2r + 2m, 2t + 2r]          → 需 t ≥ m
max_length_for_one < max_length_for_two ≤ 2t+4r+2m，且 one ≥ 2r+2m → 需 t + r > 0
thick_min_width < thin_max_width，後者 ≥ 1  → 由上兩條推得
```

`t < m` 時無解，遞迴永不終止。`create()` 產出的配置永遠滿足 `t₀ ≥ m`（`t₀ ≈ 3.12·(9.11 r² + 1.3)`，r 以 mm 計，恆大於 r），所以崩潰只會由 `apply_density()` 觸發。

型別約束：`coord_t` 為 `int32_t`（[libslic3r.h](../../../src/libslic3r/libslic3r.h)），專案為 C++17。原 `apply_density()` 中 `thin_max_distance` 與 `thick_outline_max_distance` 走 **int32 ÷ float → float → 截斷**，`thick_inner_max_distance` 走 **double 開根號 → 截斷**。位元等級相容性完全依賴這些運算的型別與順序。

編譯與測試在上層 repo（`D:\repos\web_slicer_core`）進行。

## Goals / Non-Goals

**Goals:**

- 任何 `support_points_density_relative`（0 至 `INT_MAX`）與 `support_head_front_diameter ≥ 0` 的組合，`create → apply_density → apply_min_distance` 皆在有限步內返回合法配置，或拋出明確的 `CodedRuntimeError`。不得 Stack Overflow、卡死或 OOM。
- 頭徑 ≥ 0.2 mm 時，凡原本未崩潰的密度輸入，產出的 `SampleConfig` 每個欄位位元等級不變。
- 移除 `verify()` 內會在高密度時中斷 Debug build 的 assert，讓高密度路徑在 Debug 下可以執行。本次實際驗收只在 Release 進行：Debug 驗證暫緩（相依套件沒有 Debug 版），RelWithDebInfo 驗證略過（見 Open Questions）。
- 唯一修改的原始碼檔為 `SampleConfigFactory.cpp`，測試附加於既有的 `sla_supptgen_tests.cpp`，不碰任何 CMake 檔。

**Non-Goals:**

- 不防禦「手動填值」的任意 `SampleConfig`（如 GUI 停用區塊 `USE_ISLAND_GUI_FOR_SETTINGS` 直接編輯欄位、把 `thick_outline_max_distance` 設為 0）。這類輸入在密度 100% 時本來就可能卡死，不在本變更範圍。
- 不處理極大頭徑或超低密度下 `coord_t`（int32）的數值溢位。這是既有問題。`verify_once` 以 int32 計算 `2t + 4r + 2m` 等上限，超過約 2147 mm 即溢位（有號溢位屬未定義行為）。離線驗算：頭徑約 ≥ 4 mm 搭配 10% 密度即會溢位；頭徑 15 mm（柱徑上限 15 mm，頭徑不得大於柱徑）在 100% 也會溢位。`support_points_min_distance` 同樣沒有上限：`apply_min_distance()` 會把 thin_max_distance 拉到 min_distance，超過約 1073 mm 時 `2t` 即超出 int32；超過約 2147 mm 時連 `static_cast<coord_t>(scale_(min_distance_mm))` 本身都會溢位。發生時新版會在 21 輪後拋出 `SUPPORT_POINT_SAMPLING_FAILED`，不再無窮遞迴，但結果仍屬未定義行為。2026-10-12 決定不在本變更修正，以免擴大範圍。
- 不調整非 island 區域的密度縮放（`SupportPointGenerator.cpp` 約 501、518 行）。
- 不修改 `SLAPrint::validate()`，也不新增設定上限。
- 不讓 thin 區域的點數無限增加。超過門檻後 thin 間距停在 `m_thresh`，這是接受的取捨。thick 區域則照原公式繼續變密，直到觸及下限。

## Decisions

### D1：防線放在 `apply_density()`，`verify()` 只當保險

**決定**：由 `apply_density()` 保證輸出滿足 `t ≥ m`，讓 `verify()` 在正常呼叫下必定有解。`verify()` 只負責「有限步內收斂，否則報錯」。

**替代方案**：
- 只改 `verify()`，加次數上限後回傳當下的值。**否決**：無解時 `thin_max_width` 每輪在兩值間跳動，停下時拿到哪個值只取決於輪數的奇偶。這等於把一個任意、不合法的配置交給下游。
- 在 `SLAPrintSteps.cpp` 把呼叫順序改成先 `apply_min_distance` 再 `apply_density`，或只 verify 一次。**否決**：現行「verify 兩次」會在第一次以較小的 `t` 夾限 `max_length_*`，第二次不會放寬。改順序或合併後被夾限的值會變少，1039 點那組基準因此改變，違反相容性。

### D2：以「結果分支」決定是否飽和，而不是預先比較密度門檻

**決定**：

```
if is_approx(density, 1)  → return current          （原樣保留）
if density < 0.1          → density = 0.1            （原樣保留）

t_original       = static_cast<coord_t>(current.thin_max_distance / density)              ← 原算式
inner_original   = static_cast<coord_t>(std::sqrt(sqr((double) current.thick_inner_max_distance) / density))  ← 原算式
outline_original = static_cast<coord_t>(current.thick_outline_max_distance / density)     ← 原算式
m_thresh         = max(current.minimal_distance_from_outline, current.minimal_move)

if t_original ≥ m_thresh:                                   ← 原路徑：三個值原樣寫入
    thin_max_distance          = t_original
    thick_inner_max_distance   = inner_original
    thick_outline_max_distance = outline_original
else:                                                       ← 飽和路徑
    thin_max_distance          = m_thresh                                   ← 只有 thin 飽和
    thick_inner_max_distance   = max(inner_original, max(2 * current.head_radius, current.minimal_move))
    thick_outline_max_distance = max(outline_original, current.minimal_move)
verify(result)
```

**理由**：
- 崩潰與否由原 float 運算算出的 `t` 決定。若改成先比較 `density` 與 `t₀ / m` 再決定，兩種算法在門檻附近可能差 1 個 ulp，導致「原本不崩潰卻被夾到」。看結果再分支，就能依構造保證：只要走原路徑，結果必然相同，不需要額外證明。
- 三個 `*_original` 都必須沿用原本的算式：thin 與 outline 是 int32 ÷ float，inner 是 double 開根號。把它們「改成 double 比較精準」會直接破壞位元相容性，實作與 review 時要特別注意。三個值先算好再分支，與原碼的運算完全相同，只是寫入順序不同。
- **只有 thin 飽和**：超過門檻後，thick 區域仍照原公式隨密度變密。`thick_inner_max_distance` 因此維持開根號縮放，**不可**寫成線性除法。0.6 mm 頭的離線驗算值如下：

  ```
  密度      thick_outline     thick_inner
  2205%     0.281 mm          1.761 mm
  3100%     0.200 mm          1.485 mm   ← 目標約 1.48 mm
  4200%     0.148 mm          1.276 mm   ← 目標約 1.27 mm
  10000%    0.100 mm（下限）   0.827 mm
  下限起點  約 6201%           約 18988%（停在 0.6 mm）
  ```

- thin 直接設為 `m_thresh`，不需要算飽和密度。依定義 `t₀ / (t₀ / m_thresh) = m_thresh`，用除法算只會多出浮點誤差。
- **兩個 thick 下限只在飽和路徑套用。** 0.2 mm 頭的 `thick_outline` 約在 4069% 就低於 0.1 mm，但 thin 要到約 4340% 才飽和。如果原路徑也套用下限，4069%～4340% 這段原本正常的結果就會被改到。原路徑也不需要防零：此時 `t ≥ m_thresh ≥ 0.1 mm`，`thick_outline` 約為 `0.9375 × t`，至少 0.094 mm；`thick_inner` 更大。
- `thick_inner` 的下限取 `max(頭直徑, minimal_move)`，不只取頭直徑。頭徑為 0 時頭直徑就是 0，擋不住間距變 0。頭直徑的意義是讓內部格點的頭不互相重疊。
- `thick_outline` 的下限取 `minimal_move`（0.1 mm），也就是對齊步長。
- `verify()` 不讀這兩個 thick 欄位，所以 thick 下限不影響收斂。

**相容性的成立條件**：頭徑 ≥ 0.2 mm 時，`m = r ≥ 0.1 mm = minimal_move`，所以 `m_thresh = m`。於是「走原路徑」等於「`t_original ≥ m`」，也就等於「原本有解、不崩潰」。

**替代方案**：
- 三個間距在同一點一起飽和（本設計的前一版）。**否決**：超過門檻後 thick 區域就不再變密。需求是 thick 區域仍要隨密度增加，例如 0.6 mm 頭在 3100%、4200% 時 `thick_inner` 要分別約為 1.48 mm、1.27 mm。
- 兩個 thick 下限在兩條路徑都套用。**否決**：會改到 0.2 mm 頭在 4069%～4340% 的結果，理由見上。
- 分別把三個間距夾到 `m`。**否決**：`thick_outline_max_distance` 縮得比 `t` 快，0.6 mm 頭約在 2067% 就小於 `m`，會改到 2067%～2204% 這段原本正常的結果。
- `min(density, t₀ / m)` 預先夾密度。**否決**：理由同上述 ulp 問題。

### D3：thin 飽和下限採 `minimal_move`

**決定**：`m_thresh = max(m, current.minimal_move)`。`create()` 將 `minimal_move` 設為 0.1 mm。`m_thresh` 只用於分支判斷與飽和計算，**不寫回** `minimal_distance_from_outline`。

**理由**：頭徑為 0 時 `m = 0`，若只用 `m`，飽和永遠不觸發，`t` 可縮到 µm 級，造成 OOM。0.1 mm 已是對齊步長，比它更細的間距沒有列印意義。寫回 `m` 的話，連頭徑 0、密度 100% 這種現在可用的輸入也會被改變。

**代價**：頭徑 < 0.2 mm 時，`t ∈ [m, 0.1 mm)` 的密度區間原本不崩潰，現在改為飽和。例如 0.1 mm 頭約自 4127% 起改變，原崩潰點約為 8254%。這是 proposal 已核准的刻意行為改變。

**替代方案**：
- 1 µm 絕對下限。**否決**：只防得住除以零，防不住 OOM。0 mm 頭時 50 mm 方塊仍約有 60 萬點。
- 另立一個專用常數。**暫不採用**：`minimal_move` 已存在且語意相近。缺點是日後若有人調整對齊步長，飽和門檻會跟著變動（見 Risks）。

### D4：`verify()` 改為有限迴圈加例外

**決定**：

```cpp
namespace {
// 原 7 條檢查，順序、lambda 與 `res &=` 寫法完全不變；僅移除三個 assert。
bool verify_once(SampleConfig &cfg);
constexpr int max_extra_verify_passes = 20;
}

bool SampleConfigFactory::verify(SampleConfig &cfg) {
    if (verify_once(cfg))
        return true;
    for (int i = 0; i < max_extra_verify_passes; ++i)
        if (verify_once(cfg))
            return false;
    throw Slic3r::CodedRuntimeError(Slic3r::make_engine_error(
        Slic3r::EngineErrorCode::SUPPORT_POINT_SAMPLING_FAILED,
        "SampleConfig convergence failed: cyclic geometric constraint"));
}
```

**等價性**：原遞迴等於「重複跑單輪檢查，直到某一輪未修改任何值」。某一輪回傳 `true` 代表 7 條檢查都沒動到數值，所以之後再多跑也不會改變任何值。迴圈版本因此與原遞迴在每個合法輸入上位元一致。回傳值維持原語意：首輪是否完全未修改。

**實作陷阱**：
- `res &= check(...)` 必須保留。改成 `res = res && check(...)` 會短路，後面幾條檢查就不跑了，修正順序與結果都會改變。
- `verify_once()` 放在 `.cpp` 的匿名 namespace，不進標頭。測試透過 `verify()` 的回傳值就能檢查「首輪即通過」。
- 例外類別與錯誤碼沿用 [UniformSupportIsland.cpp:2147](../../../src/libslic3r/SLA/SupportIslands/UniformSupportIsland.cpp#L2147) 的前例（`CodedRuntimeError` + `SUPPORT_POINT_SAMPLING_FAILED`），所以 CLI 已有的 `EngineErrorCarrier` 管道不需要改。
- 新增 `#include <libslic3r/EngineErrorCodes.hpp>`。

**上限 20 的依據**：離線模擬顯示頭徑 0.2～1.5 mm、密度 10% 至各自門檻之間，最多只需 4 輪。這只是模擬，沒有形式化證明，所以保留上限加例外作為保險。上限取 20（約 5 倍餘裕）而非 10：每輪只是 7 次整數比較，多跑幾輪的成本可以忽略，卻能在飽和邊界附近多留緩衝。

### D5：assert 由 `verify()` 遷到 `create()`

**決定**：移除 `verify()` 內第 7、15、24 行的 `assert`。在 `create()` 改為：

```cpp
[[maybe_unused]] const bool clean = verify(result);
assert(clean);
```

**理由**：這三個 assert 斷言「進來的配置本來就合法」，但密度超過約 205% 起，`verify()` 本來就該修正數值，所以 assert 放在這裡已經過時。原作者真正的意圖是「`create()` 的預設值必須自洽」，搬到 `create()` 後仍然保留。模擬顯示每種頭徑的 `create()` 都是首輪即通過。

**不可寫成 `assert(verify(result))`**：`NDEBUG` 下整個運算式會被移除，Release 版的 `create()` 就完全不做 verify 了。

**NDEBUG**：依 CMake 預設，Debug 不定義 `NDEBUG`，Release 與 RelWithDebInfo 會定義。`SampleConfigFactory.cpp` 沒有 `#undef NDEBUG`。照上述兩行寫法，三種組態的計算結果一致，差別只在 Debug 多做一次檢查。

### D6：測試附加於 `sla_supptgen_tests.cpp`，以逐欄一致性為核心門檻

**決定**：所有新增測試寫在既有的 [tests/sla_print/sla_supptgen_tests.cpp](../../../tests/sla_print/sla_supptgen_tests.cpp)，不更動 `tests/sla_print/CMakeLists.txt`。該檔已 include `SampleConfigFactory.hpp` 與 `SampleConfig.hpp`，另需加入 `<libslic3r/EngineErrorCodes.hpp>` 以捕捉例外。tag 使用 `[SupportIsland][SampleConfig]`。

**驗收分兩層**：

1. **核心門檻：`SampleConfig` 逐欄位元一致（Catch2，必須全過）**
   - 在測試檔的匿名 namespace 內嵌 `legacy_verify()`（原遞迴版本，**去除 assert**，否則 Debug 會在 205% 中斷）、`legacy_create_unverified()`（舊版 `create()` 去掉結尾的 verify，即原始預設值）、`legacy_create()` 與 `legacy_apply_density()`、`legacy_apply_min_distance()`（都呼叫 `legacy_verify()`）。legacy 必須呼叫自己的 verify，不能呼叫新版，否則 verify 的重構等於自己跟自己比。
   - `legacy_verify()` 加一個遞迴深度參數，上限 50 層。超過時拋出測試專用例外，讓該測試案例以 FAIL 結束，而不是讓整個測試程序 Stack Overflow。合法區間的遞迴深度只有個位數，這個保護不影響對照結果；它只在網格邊界判斷寫錯時起作用。
   - 網格：整條舊鏈 `legacy_create → legacy_apply_density` 對整條新鏈 `create → apply_density`，連 `create()` 一起逐欄比對。頭徑 `{0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5}` mm × 整數百分比密度，自 10% 起逐 1% 遞增；0.2 mm 是 `head_radius` 等於 `minimal_move` 的邊界，1.5 mm 是設計時模擬過的上限。密度與正式程式一樣以 `float(pct / 100.f)` 轉換。每一點先以原 float 算式算出 legacy 的 `t`，若 `t < m` 即停止該頭徑的掃描，避免測試自己 Stack Overflow。
   - 比對 `SampleConfig` 的全部純量欄位（13 個 `coord_t`、`max_length_ratio_for_two_support_points`、`count_iteration`、`simplification_tolerance`），全部要求 `==`。`prepare_config` 兩條路徑都不會碰，所以不比。
   - 另跑一組 `apply_min_distance(…, 1.5f)` 的網格，覆蓋「verify 兩次」的路徑。

2. **補充斷言（Catch2）**
   - 預設值自洽：對 `{0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5}` mm 共 10 種頭徑的**原始預設值**（`legacy_create_unverified()`）呼叫 `verify()` 必須回傳 `true`，且原始值與 `create()` 逐欄相同。不能直接對 `create()` 的結果呼叫 `verify()`：那是已修正過的值，必定回傳 `true`，測不出問題。
   - 極端輸入：密度 5000%、10000%、`INT_MAX`% × 頭徑 0、0.1、0.6 mm。驗證函式正常返回，`thin_max_distance ≥ m_thresh`，`thick_outline_max_distance ≥ minimal_move`，`thick_inner_max_distance ≥ max(頭直徑, minimal_move)`，且以 `check_constraints_hold()` 直接斷言 7 條幾何約束成立（以 int64 計算，避免檢查本身溢位）。不採「對結果再呼叫一次 `verify()`」：結果已是不動點，必定回傳 `true`。
   - 2205%、5000%、10000% 加 `min_distance` 1.5 mm：thin 必須等於 1.5 mm，兩個 thick 必須 ≥ 1.5 mm，7 條約束成立。
   - 無例外掃描：上述 10 種頭徑 × 0%、10%～10000% 逐 1%、之後每 997% 一點到 100 萬%、加 `INT_MAX`%，`min_distance` 為 0.15 與 1.5 mm，皆不得拋出例外。0.15 mm 會落在飽和後的 thin 與 thick 之間，只拉高部分間距。
   - thick 持續縮小：0.6 mm 頭在 3100%、4200% 時，`thick_inner_max_distance` 分別約為 1.48 mm、1.27 mm（±0.01 mm），且等於原公式的計算值。
   - 下限只在飽和路徑：0.2 mm 頭在 4100% 時 `thick_outline` 已低於 0.1 mm、thin 尚未飽和，結果必須與 legacy 逐欄相同。
   - 例外路徑：對 `create(0.6f)` 的結果手動設定 `thin_max_distance = minimal_distance_from_outline / 2`，呼叫 `verify()` 必須拋出例外。捕捉後以 `dynamic_cast<const EngineErrorCarrier*>` 確認錯誤碼為 `SUPPORT_POINT_SAMPLING_FAILED`。

3. **全流程整合驗證（CLI，手動，於上層 repo 執行）**
   - 50×50×10 mm 方塊、`--printer-technology SLA`、0.6 mm 頭、1 mm 柱、`--export-support-points`，以**完全相同的參數**分別執行修改前（a114d13bd，已備份）與修改後的引擎。
   - 2204%（min_distance 0 與 1.5）：新舊匯出檔必須逐位元組相同，結束碼皆為 0。
   - 2205%、5000%、10000%（min_distance 0 與 1.5）：新引擎必須正常結束（結束碼 0），舊引擎以 Stack Overflow 結束（`-1073741571`）。
   - 不以 1609／1039 這兩個點數為驗收值：它們來自一條已遺失的指令，參數與本次不同。新舊同參數比對不依賴當初的參數。實際指令與數據記錄在 tasks 4.7。
   - 這一層是整合信心，不是位元相容性的判準。位元相容性以第 1 層為準。

**替代方案**：
- 開新測試檔。**否決**：需要改 CMake，違反變更最小化。
- 以外部 golden file 當基準。**否決**：多一個要維護的檔案；內嵌 legacy 可以讓網格自由擴充。

## Risks / Trade-offs

- [模擬與實機不一致：門檻表、4 輪收斂只來自離線模擬] → 第 1 層網格測試在真實 build 上重新驗證；若任一點需要超過 20 輪，測試會以例外失敗，屆時再調整上限。
- [原路徑被「順手優化」：改成 double、改 `&=` 為 `&&`、調整檢查順序] → design 中明列為陷阱，並以第 1 層逐欄網格把關，任何偏差都會失敗。
- [`minimal_move` 被他人調整，飽和門檻跟著移動] → 只影響頭徑 < 0.2 mm 或超過門檻的輸入。在 `apply_density()` 註解標明 `minimal_move` 兼任飽和下限。
- [超過門檻後 thin 區域點數不再增加，thick 區域仍持續變密] → proposal 已接受。thick 區域在觸及下限前仍照原公式變化。
- [thick 下限只限制點數密度，不保證不 OOM] → 頭徑 < 0.2 mm 在極端密度下，內部格點停在 0.1 mm，50 mm 方塊約 25 萬點，之後還要建 Voronoi、對齊 30 輪，可能很慢或吃掉大量記憶體。0.6 mm 頭停在 0.6 mm，50 mm 方塊約 7 千點。這兩個數字都是粗估，沒有實測。若要更強的保證，需把 `thick_inner` 的下限提高，例如對小頭徑另設一個較大的固定值。
- [legacy 對照本身抄錯，導致假通過] → legacy 直接自 git 中修改前的版本逐字複製，只移除 assert；實作時在測試註解記下來源 commit。
- [例外在無法處理的執行緒拋出] → 目前唯一呼叫點在 `SLAPrintSteps` 的支撐點步驟內，已在 `PrintBase` 的例外處理範圍。GUI 呼叫點位於停用的巨集內。
- [`[[maybe_unused]]` 或 `EngineErrorCodes.hpp` 的編譯相依] → 專案已是 C++17，`EngineErrorCodes.hpp` 也已被同模組的 `UniformSupportIsland.cpp` 引用，沒有新的連結相依。

## Migration Plan

- 不涉及資料、設定或 preset 遷移。
- 部署：在上層 repo 編譯並跑 `sla_print_tests "[SampleConfig]"`，再執行第 3 層 CLI 驗證。
- 回滾：還原 `SampleConfigFactory.cpp` 與 `sla_supptgen_tests.cpp` 兩個檔案即可。沒有其他檔案受影響。

## Open Questions

以下兩項已定案，保留紀錄：

- **第 3 層 CLI 驗證方式（已定案）**：50×50×10 mm 平放方塊、`--printer-technology SLA`、`support_head_front_diameter = 0.6`、`support_pillar_diameter = 1.0`、`--export-support-points`，新舊引擎同參數比對：2204% 匯出檔逐位元組相同；2205% 以上新引擎正常結束、舊引擎崩潰。最初回報的 1609／1039 點來自已遺失的指令，不作為驗收值；2026-10-12 以引擎預設參數實測，2204% 新舊皆為 909／908 點。此層定位為**手動整合驗證**，不是阻斷門檻；阻斷門檻是第 1 層 Catch2 逐欄比對。
- **編譯組態（已定案）**：上層建置以 Release／RelWithDebInfo 為主。因 D5 的兩行式 assert 已消除組態間的計算差異，第 1、2 層測試 MUST 在 Release 執行並通過。Debug 驗證暫緩（2026-10-12 決定）：相依套件 `DEP_DEBUG=OFF`，Debug 組態無法連結。RelWithDebInfo 驗證略過（2026-10-12 決定）：與 Release 同樣定義 `NDEBUG`、計算邏輯等價，而重建成本高。
