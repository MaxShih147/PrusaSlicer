## Why

極薄模型（如 `dish.stl`，Ø21.168 × 厚 **0.2 mm**）在 SLA 切片時暴露出兩組互相銜接的缺陷。

### 問題一：支撐完全無法生成

`Automatic support points: 0` → 支撐樹為空 → `SLAPrint::validate()` 判定 `There are unprintable objects`，整趟切片作廢。此問題已在 `web_slicer_core` 的 `agent/jobs/` 中重現多次（`9145a214`、`4ae72d98`、`81595229`、`e27938fb`…，全數回報 `SUPPORT_NOT_NEEDED` 且無輸出）。

經 instrumented build 逐點追蹤，根因已**完全定位**，且與三個直覺假設全部不符：

- **不是 Critical Angle 過濾**——支撐點在進入 support tree 之前就已歸零，角度過濾根本沒機會執行。
- **不是支撐點產生器失效**——`generate_support_points()` 在失敗案例中**正確回傳 19 筆**。
- **不是模型太薄本身**——同一顆 0.2 mm 模型在 2/3 的網格相位下可正常產出 19 筆支撐點。

真因是 `move_on_mesh_surface()` 的「就近吸附」缺少方向性約束。支撐點建立在 island 取樣層的 `print_z`；當該取樣層落在薄板**中面之上**時，「最近的面」變成**上表面**，於是全部支撐點被吸附到模型頂部（`zmin = zmax = 0.2`）。這些點隨即被 `filter_support_points_by_modifiers()` 的 `lower_bound → end()` 分支無聲丟棄（19 → 0，零診斷訊息）；即使僥倖存活，其法線為 `(0,0,+1)`、polar = 0，也會在 `add_pinheads()` 的 `normal_cutoff_angle` 再被砍一次。

失敗判準已驗證為**確定性**：

```
失敗 ⟺ 模型內最低 slice level 的高度 a > 薄板中面 (thickness / 2)

  上表面 z=0.20 ━━━━━━━━━━━━━ ◄── 點被錯誤吸附至此
         ▲ 0.075（較近，勝出）
  取樣層 z=0.125 ●
  中面   z=0.10 ┈┈┈┈┈┈┈┈┈┈┈┈
         ▼ 0.125
  下表面 z=0.00 ━━━━━━━━━━━━━ ◄── 正確位置
```

此判準對本次調查蒐集的**每一個**資料點皆成立（elevation 掃描 5.00→5.15 逐 0.01、三種厚度 0.2/0.3/0.5 mm、四種層高 0.05/0.10/0.15/0.40），無反例。問題的隱蔽性在於**參數的微小擾動即可翻盤**：僅將 `support_object_elevation` 由 `5.00` 改為 `5.05`（與支撐點採樣邏輯毫無語意關聯，只改變切片網格相位），結果即由 0 筆變為 19 筆。使用者無從得知失敗原因，錯誤訊息（`mesh being broken`）更指向完全錯誤的方向。

### 問題二：支撐頭刺穿模型頂面

問題一修復後，支撐得以生成，隨即暴露第二層缺陷：`support_head_penetration` 是純設定純量，底層**完全沒有厚度感知**。當設定刺入深度大於局部壁厚時，支撐頭直接貫穿頂面。以實際 job 參數（`head_front_diameter = 0.8` → `r_pin = 0.4`，`head_penetration = 0.3`，板厚 0.2）計算：

```
                現況 penetration=0.3              夾限至 0.1 (= 厚度×0.5)
  上表面 z=0.20 ┈┈┈┈┬─────┬┈┈┈ 冒出 Ø0.53 凸點     ┈┈┈┈┈┈┈┈┈┈┈┈  乾淨
                    │ ╭─╮ │                         ╭───╮
  下表面 z=0.00 ━━━━┼─┼─┼─┼━━━                 ━━━━┼─●─┼━━━━  球頂剛好在 0.1
                     ╲│╱                            ╲   ╱
```

推導 `pinhead()` 與 `get_mesh(const Head&)` 的座標得到一個有利的不變量——**pin 球最深點恰好等於 `penetration_mm`**：

```
平移後 pin 球頂 = (r_back + r_pin + length + r_pin) − (2·r_pin + width + r_back − penetration)
               = length − width + penetration = penetration    （get_mesh 傳入 length = h.width_mm）
```

因此夾限 `Head::penetration_mm` 對「上表面零凸點」是**必要且充分**的，與 `r_pin` 大小無關，**完全不需修改 `SupportTreeMesher`**。

### 共同背景

`SupportPointGenerator.cpp` 與 `SLAPrintSteps.cpp` 相關路徑經 diff 確認**與 upstream PrusaSlicer 2.9.6 逐位元組相同**，六項缺陷皆為自上游繼承，非本 fork 改壞。

## What Changes

本變更採納完整修復組合 #1～#6。**存在明確的施作順序相依**：

```
#1 ──────────────────────────────► 必須最先，否則 #2 造成回歸
  └─ #2                            #1 完成後才可施作
#3 ─── 獨立
#4 ─── 獨立
#6 ──────────────► #5             #6 是 #5 的前置（射線需單位向量）
                    └─ #5 需在 #1 之後才觀察得到效果

建議施作序：#1 → #2 → #3 → #4 → #6 → #5
```

- **#1（治本）為 `move_on_mesh_surface()` 的表面吸附加入方向性約束**：支撐點必須吸附到**朝下的面**（命中三角形法線 z 分量為負），而非單純的幾何最近面。等價的保守策略為優先採用向下射線命中結果，僅在下方無命中時才回退至向上命中。此為唯一能真正解決「支撐點被吸到模型頂部」的修改；缺少此項，#2～#4 皆無法讓極薄模型產出支撐。

- **#2 消除 `allowed_move` 的未初始化記憶體讀取（UB）**：`m_model_height_levels.size() < 2` 時越界讀取 `[1]`，`assert` 在 Release build 下不存在。改以設定的 `layer_height` 作為 fallback。
  **順序相依警告**：實測顯示該越界目前讀到 `0.0`，使 `allowed_move` 成為負值（`−0.0749999`），反而**歪打正著**強制走入正確的 `squared_distance` 分支。若先修 #2 而未修 #1，將使目前可正常運作的相位一併失效，屬**行為回歸**。

- **#3 修正 `filter_support_points_by_modifiers()` 的無聲丟棄**：`std::lower_bound` 回傳 `end()`（支撐點 z 超出 `slice_grid` 最末元素）時，現行程式碼缺少 `else` 分支，該點被直接捨棄且無任何日誌。改為 clamp 至最後一層索引，並在有點位被丟棄時輸出診斷日誌（丟棄筆數、z 範圍、grid 範圍）。此項為防禦與可觀測性補強——單獨施作**無法**解決問題一。

- **#4 修正誤導性的切片失敗訊息**：`closest_slice_record()` 落空時，現行訊息為 `can not be sliced. This can be caused by the model mesh being broken. Repairing it might fix the problem.`，但實際成因是**層高相對模型高度過大**（實測 0.5 mm 模型搭配 `layer_height = 0.6` 即觸發），與網格完整性無關。改為陳述實際成因並指出可行動的方向（調降層高）。

- **#6 補回 `taildir` 的正規化**：`connect_to_model_body()` 中 `Vec3d taildir = endp - hitp;` 未正規化，為 fork 相對 upstream 缺失的修正。`AABBMesh::query_ray_hit()` 具 `assert(is_approx(dir.norm(), 1.))` 前置條件，Release build 下該斷言不存在，傳入非單位向量會使回傳的 `m_t` 為參數 t 而非實際長度。此為既有缺陷修正，**且是 #5 的前置依賴**；因其會改變 anchor 的既有幾何，故獨立列項而非併入 #5。

- **#5 動態刺入深度防貫穿（Zero Top-Surface Puncture）**：於支撐頭幾何生成端以射線量測局部壁厚並夾限刺入深度。

  - **量測**：`local_thickness = emesh.query_ray_hit(hp − ε·dir_in, dir_in).distance() + ε`，其中 `dir_in = −head.dir`（入模方向；`get_mesh()` 的四元數將局部 `(0,0,−1)` 映至 `head.dir`，故局部 +z 即 `−head.dir`）。`ε` 取極小值以避免自我命中。
  - **夾限公式**：`penetration = min(configured_penetration, local_thickness × 0.5)`
  - **政策：上表面零凸點優先**。明確接受在極薄物件上因咬合深度不足、列印時可能脫落的物理代價，優先保證幾何上表面完全乾淨。
  - **Fail-safe**：射線無命中（破面／非流形網格）時 `penetration = 0`，並輸出**彙總警告日誌**（觸發點數量），使其可診斷而非靜默。
  - **時序：先搜尋、後夾限**。所有角度搜尋結束後，於最終 `Head` / `Anchor` 物件提交的當下夾限一次，維持既有搜尋行為的穩定度。此決策同時消除了 `w` 與 optimizer 之間的循環相依。
  - **兩類提交點語意不同**：主頭（pinhead）的提交點在 junction／pillar 計算之前，`fullwidth()` 連帶更新，使頭與柱維持相連；錨點（anchor）的提交點在 `add_anchor()` 呼叫處，橋接端點已固定，anchor 網格重疊橋接端（而非脫開），**拓撲零變化**。
  - 適用於 Default 樹與 Branching/Organic 樹的主頭與錨點，共四個提交點。

- **新增回歸測試涵蓋**：以確定性判準（最低 slice level vs 薄板中面）為基礎，涵蓋失敗相位、成功相位、`m_model_height_levels` 的 `size() == 1` 與 `size() >= 2` 兩種組態，確保 #2 的順序相依不會回歸；並涵蓋防貫穿的布林交集驗證與薄壁 anchor 的幾何驗證。

- 本變更**不改變**任何既有設定參數的語意、預設值或 UI，**不引入**新的設定項目，**無 BREAKING**。

## Capabilities

### New Capabilities
- `sla-support-point-placement`: 規範 SLA 自動支撐點由 island 取樣層投影至模型實際表面的行為契約——支撐點必須落在承載支撐的**朝下表面**而非幾何最近表面；規範切片網格相位、層高與模型厚度之間的邊界條件下支撐點必須維持穩定；規範支撐點在 modifier 過濾階段不得因網格索引越界而被無聲丟棄；並規範層高過大導致模型無切片層時的錯誤語意。（涵蓋 #1、#2、#3、#4）
- `sla-support-head-penetration`: 規範支撐頭刺入模型的深度契約——刺入深度必須依局部壁厚動態夾限，保證支撐頭幾何不穿透承載面的另一側；規範壁厚量測的射線方向與退化情形（無命中）的 fail-safe 行為；規範夾限的施加時序（角度搜尋完成後）與主頭／錨點兩類提交點的拓撲影響邊界。（涵蓋 #5、#6）

### Modified Capabilities
<!-- 無既有 openspec/specs 規範變更。既有 `sla-layer-blur` 與 `sla-support-binary-rasterization` 屬光柵化階段，與本變更的支撐點放置與支撐頭幾何階段無交集。 -->

## Impact

**切片器（prusaslicer_fork）**

- [src/libslic3r/SLA/SupportPointGenerator.cpp](../../../src/libslic3r/SLA/SupportPointGenerator.cpp) — `move_on_mesh_surface()`（約 1563–1615 行）：命中面選擇邏輯加入方向性約束（**#1，治本**）。此函式亦被支撐點的其他消費端使用，需確認不影響常態路徑。
- [src/libslic3r/SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp) — `SLAPrint::Steps::support_points()`（約 884 行）：`allowed_move` 的越界讀取改為 `layer_height` fallback（**#2**）。
- [src/libslic3r/SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp) — `filter_support_points_by_modifiers()`（約 739–789 行）：`lower_bound` 落於 `end()` 時 clamp 並補診斷日誌（**#3**）。
- [src/libslic3r/SLAPrintSteps.cpp](../../../src/libslic3r/SLAPrintSteps.cpp) — `SLAPrint::Steps::slice_model()`（約 691–694 行）：切片失敗訊息改寫（**#4**）。訊息字串位於 i18n 涵蓋範圍，需同步處理翻譯條目。
- [src/libslic3r/SLA/DefaultSupportTree.cpp](../../../src/libslic3r/SLA/DefaultSupportTree.cpp) — `connect_to_model_body()`（約 731 行）：`taildir` 正規化（**#6**）。
- [src/libslic3r/SLA/DefaultSupportTree.cpp](../../../src/libslic3r/SLA/DefaultSupportTree.cpp) — `add_pinheads()` 接受區塊（約 527–531 行）與 `connect_to_model_body()` 的 `add_anchor()` 呼叫（約 742 行）：Default 樹的主頭與錨點夾限（**#5**）。
- [src/libslic3r/SLA/SupportTreeUtils.hpp](../../../src/libslic3r/SLA/SupportTreeUtils.hpp) — `optimize_pinhead_placement()` 接受區塊（約 408–412 行）：Branching 樹主頭夾限（**#5**）。
- [src/libslic3r/SLA/BranchingTreeSLA.cpp](../../../src/libslic3r/SLA/BranchingTreeSLA.cpp) — `m_builder.add_anchor(*anchor)` 呼叫處（約 298 行）：Branching 樹錨點夾限（**#5**）。**必須在此處而非 291 行之前**，否則 `anchor->junction_point()` 位移將改變橋接端點，破壞「後夾限」所要保住的搜尋穩定度。

**下游行為（無程式碼修改，但結果改變）**

- `SLAPrint::validate()`：極薄模型不再誤報 `There are unprintable objects`。
- [src/libslic3r/SLAPrint.cpp](../../../src/libslic3r/SLAPrint.cpp) 約 733 行的 elevation 驗證使用全域 `head_fullwidth()`，per-head 夾限會使該估算略偏樂觀。影響輕微，需記錄。
- `web_slicer_core` `agent/`：`support_outcome` 不再對極薄模型錯誤回報 `SUPPORT_NOT_NEEDED`；`has_support_mesh` 將轉為 `true`。**後端無需修改**，但既有測試若以「極薄模型 → 無支撐」為預期，需同步更新。

**已知殘留風險（皆為所選政策的有界後果）**

| 風險 | 界限 | 處置 |
|---|---|---|
| `w` 以未夾限值驗證，提交後頭部向外多伸出 `(configured − clamped)` | ≤ `configured_penetration` | 薄壁模型的 anchor 回歸測試 |
| Fail-safe 使破面模型支撐零咬合 | 僅限非流形網格 | 彙總警告日誌，使用者可據以修模 |
| 夾限僅在 `局部壁厚 ≥ 2 × penetration` 時失效 | 壁厚 < 0.8 mm（@ penetration 0.4）會被靜默調降 | 門檻明寫入 spec 驗收條件，**不宣稱「常態零影響」** |

**不影響**

- 任何設定參數的語意、預設值、preset 檔案與 UI 版面。
- 支撐樹的路由與互連演算法、pad 生成、光柵化與 SL1/PRZ 輸出格式。
- 常態模型（厚度 ÷ 層高 ≥ 2 且局部壁厚 ≥ 2 × penetration）的支撐點數量與位置——此為本變更的驗收條件之一。

**與上游的關係**

六項缺陷均存在於 upstream PrusaSlicer 2.9.6。本變更為 fork 端的獨立修復，未來 rebase 至上游新版時需檢查是否已被上游修正，以避免衝突或重複修補。
