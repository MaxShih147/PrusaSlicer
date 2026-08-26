# SLA Support Angle Threshold Refactoring

## 背景

SLA 切片原本的 `support_critical_angle` 參數控制的是支撐柱橋接的坡度（`bridge_slope`），
與 FDM 的 `support_material_threshold`（根據傾斜角度決定是否產生支撐）語意完全不同。

此次重構：
1. **新增** `support_bracing_angle` / `branchingsupport_bracing_angle`，接管原 `support_critical_angle` 的橋接坡度功能，預設值 45°
2. **重新定義** `support_critical_angle` / `branchingsupport_critical_angle` 為 overhang threshold：傾斜角度超過此值的懸空面將不放置支撐頭，預設值 90°（等同不過濾，向後相容）

---

## 角度定義與轉換

| 概念 | 量測基準 | 範圍 | 說明 |
|------|---------|------|------|
| FDM `support_material_threshold` | 從水平面量起 | 0°–90° | 超過閾值的面不生成支撐 |
| SLA `support_critical_angle`（新） | 從水平面量起 | 0°–90° | 超過閾值的面不放置支撐頭 |
| SLA polar angle | 從 +Z 軸量起 | 0°–180° | π=法向朝下（水平面），π/2=法向朝側面（垂直壁） |

**轉換公式：** `polar_min = π/2 + threshold_rad`（放支撐的條件為 `polar >= polar_min`）

由於懸空面的坡度 = `π − polar`，上式等價於「**坡度 ≤ π/2 − threshold 才放支撐頭**」。
因此 **threshold 數值越小，生成的支撐越多**。

| threshold（從水平） | SLA polar 最小值 | 實際放支撐的坡度上限 | 效果 |
|--------------------|----------------|--------------------|------|
| 90°（預設） | π（180°） | 0° | **僅對完全水平朝下的面放支撐**（幾乎不過濾即通過的只有正下方面） |
| 45° | 3π/4（135°） | 45° | 只對接近水平（≤ 45° 坡度）的懸空面放支撐 |
| 0° | π/2（90°） | 90° | **所有朝下的懸空面都放支撐**（垂直壁除外） |

> ⚠️ **方向注意**：本表的 0° 與 90° 兩列曾被誤記為相反的效果。以上為實作的真實行為，
> 已由實測驗證（3DBenchy 繞 Y 軸 -40° 時，90° 得到零支撐；30° 的支撐量大於 45°）。
>
> 另一種可能的語意是 `polar_min = π − threshold_rad`（即「90° = 不過濾」）。該式與現行式
> **僅在 threshold = 45° 時等值**，其餘閾值全部鏡射為 `90° − t`。改採該式會使所有非 45°
> 的既有專案支撐密度改變，屬**破壞性變更**，須附遷移計畫，不得逕行修改。
> 詳見 capability `sla-overhang-threshold-semantics`。

---

## 修改檔案清單

### `src/libslic3r/PrintConfig.hpp`

在 `SLAPrintObjectConfig` X-macro 中新增欄位：

```cpp
// Overhang angle threshold: support will not be placed on surfaces steeper than this angle (from horizontal).
((ConfigOptionFloat, support_critical_angle))/*= 90*/

// The default angle for connecting support sticks and junctions.
((ConfigOptionFloat, support_bracing_angle))/*= 45*/
```

`branchingsupport_*` 前綴的同名欄位也做了相同新增。

---

### `src/libslic3r/PrintConfig.cpp`

在 `add_SLA_support_params()` 函式中修改 `support_critical_angle` 定義，並新增 `support_bracing_angle`：

- `support_critical_angle`：label 改為 `"Overhang threshold"`，預設值改為 `90`，mode 為 `comAdvanced`
- `support_bracing_angle`：label 為 `"Bracing angle"`，預設值 `45`，mode 為 `comExpert`

---

### `src/libslic3r/SLA/SupportTree.hpp`

在 `SupportTreeConfig` struct 新增欄位：

```cpp
// Overhang angle threshold in radians. A support head is placed only where
// the surface's slope from the horizontal plane is at most
// (PI/2 - overhang_angle_threshold), so a SMALLER value supports MORE
// surfaces: 0 supports every overhang, PI/2 (the default) supports only
// perfectly horizontal down-facing surfaces.
double overhang_angle_threshold = M_PI / 2;
```

---

### `src/libslic3r/SLAPrint.cpp`

**`make_support_cfg()` 函式：**

Default tree 段：
```cpp
// 原: scfg.bridge_slope = c.support_critical_angle.getFloat() * PI / 180.0;
scfg.bridge_slope             = c.support_bracing_angle.getFloat()  * PI / 180.0;
scfg.overhang_angle_threshold = c.support_critical_angle.getFloat() * PI / 180.0;
```

Branching/Organic tree 段：
```cpp
scfg.bridge_slope             = c.branchingsupport_bracing_angle.getFloat()  * PI / 180.0;
scfg.overhang_angle_threshold = c.branchingsupport_critical_angle.getFloat() * PI / 180.0;
```

**invalidation list**（`SLAPrint::apply()`）新增：
- `"support_bracing_angle"`
- `"branchingsupport_bracing_angle"`

---

### `src/libslic3r/SLA/DefaultSupportTree.cpp`

在 `add_pinheads()` 的 `filterfn` lambda 中，`normal_cutoff_angle` 判斷之後插入 overhang 過濾：

```cpp
// skip if the tilt is not sane
if (polar < PI - m_sm.cfg.normal_cutoff_angle) return;

// Skip surfaces that tilt too far from horizontal to count as an overhang.
// Rearranged, this places a head only where the surface's slope from the
// horizontal plane is at most (PI/2 - overhang_angle_threshold) -- so a
// SMALLER threshold supports MORE surfaces: 0 supports every overhang,
// PI/2 supports only perfectly horizontal down-facing surfaces.
if (polar < M_PI / 2.0 + m_sm.cfg.overhang_angle_threshold) return;
```

---

### `src/libslic3r/SLA/SupportTreeUtils.hpp`

與 `DefaultSupportTree.cpp` 相同的過濾邏輯（branching tree 使用的獨立實作）：

```cpp
if (polar < PI - m.cfg.normal_cutoff_angle) return false;

// skip if the surface is not steep enough to need support
if (polar < M_PI / 2.0 + m.cfg.overhang_angle_threshold) return false;
```

---

### `src/libslic3r/Preset.cpp`

在 SLA print preset 選項列表新增：
- `"support_bracing_angle"` （在 `"support_critical_angle"` 之後）
- `"branchingsupport_bracing_angle"` （在 `"branchingsupport_critical_angle"` 之後）

---

### `src/slic3r/GUI/ConfigManipulation.cpp`

新增 UI toggle：
```cpp
toggle_field("support_bracing_angle",        supports_en && is_default_tree);
toggle_field("branchingsupport_bracing_angle", supports_en && is_branching_tree);
```

---

### `src/slic3r/GUI/Tab.cpp`

在 "Connection of the support sticks and junctions" optgroup 新增 `support_bracing_angle`（靠近 `support_critical_angle`）。

---

### `resources/profiles/PrusaResearchSLA.ini` 與 `resources/profiles/AnycubicSLA.ini`

在 `[sla_print:*common print*]` 段：

```ini
# 修改前
support_critical_angle = 45

# 修改後
support_bracing_angle = 45
support_critical_angle = 90
```

---

## 向後相容性

- 舊 preset 若只有 `support_critical_angle = 45`（無 `support_bracing_angle`）：橋接坡度會 fallback 至 `support_bracing_angle` 的預設值 45°，行為不變
- 新預設值 `support_critical_angle = 90` 等同不過濾，完整保留原有支撐生成行為
- 只需在有自訂需求時才調整 `support_critical_angle`（例如設為 45° 以只對接近水平的懸空面放支撐）
