## Context

本設計對應 proposal 中的兩組缺陷：**支撐點無法抵達支撐樹**（#1～#4）與**支撐頭刺穿模型頂面**（#5、#6）。六項缺陷全部位於 `slaposSupportPoints` 與 `slaposSupportTree` 兩個步驟之間，且全部與 upstream PrusaSlicer 2.9.6 逐位元組相同。

### 現行資料流程與失效點

```
slice_model()
  │  minZ = bb.min(Z) − get_elevation()          ← 網格原點綁在 elevation 上
  │  grid: minZs+ilhs, +lhs, ... ≤ maxZs
  │  slindex_it = closest_slice_record(bb.min(Z))
  ├─► m_model_height_levels = [slindex_it .. end]   ← 極薄件可能只剩 1～2 個元素
  ├─► m_model_slices        = slice_csgmesh_ex(...)
  │   ✗ 若 grid 內無任何模型層 → 拋出 "mesh being broken"        ← #4 訊息誤導
  └─► prepare_for_generate_supports()
         └─ prepare_generator_data(slices, heights, ...)

support_points()
  ├─► generate_support_points(data, ...)              → 19 筆（此處正常）
  │
  ├─► allowed_move = height_levels[1] − height_levels[0]
  │   ✗ size()<2 時 [1] 為未初始化讀取（UB）                      ← #2
  │
  ├─► move_on_mesh_surface(pts, emesh, allowed_move)
  │   ✗ 「最近面」無方向性 → 點被吸到上表面 (z=0.2)              ← #1  ★根因
  │
  └─► filter_support_points_by_modifiers(pts, mask, height_levels)
      ✗ lower_bound 回傳 end() → 19 筆無聲丟棄，零日誌            ← #3
      → Automatic support points: 0

support_tree()  ← 即使點存活，位於上表面者法線為 (0,0,+1)、polar=0
                  仍會被 add_pinheads() 的 normal_cutoff_angle 砍掉

（#1～#4 修復後，支撐得以生成，第二層缺陷才顯現）
add_pinheads() / connect_to_model_body() / optimize_pinhead_placement()
  ✗ head_penetration_mm 為純設定純量，無厚度感知 → 刺穿頂面      ← #5
  ✗ taildir 未正規化 → query_ray_hit() 前置條件違反              ← #6
```

### 關鍵約束

- `m_model_height_levels` 的元素數量由**切片網格相位**決定，而相位由 `get_elevation()` 的小數部分決定。使用者無法直接控制，且與支撐語意無關。
- `Head::penetration_mm` 是**每個 Head 的成員變數**（非每次由 cfg 讀取），這使得 per-head 夾限可行且能讓下游自動保持一致。
- `AABBMesh::query_ray_hit()` 具 `assert(is_approx(dir.norm(), 1.))` 前置條件，且 Release build 下該斷言不存在。
- `SupportPointGenerator.cpp` 與 `SLAPrintSteps.cpp` 與 upstream 相同，修改會增加未來 rebase 的衝突面。

## Goals / Non-Goals

**Goals:**

- 使支撐點的表面投影結果**與切片網格相位無關**——同一模型在任意 elevation 相位下產出一致的支撐點集合。
- 消除 `support_points()` 路徑上的未定義行為與無聲資料遺失。
- 保證支撐頭幾何**不穿透承載面的另一側**，在任意局部壁厚下皆成立。
- 讓失敗與退化路徑**可診斷**：錯誤訊息指向真實成因，靜默丟棄改為有日誌。
- 常態模型（厚度 ÷ 層高 ≥ 2 且局部壁厚 ≥ 2 × penetration）的支撐點數量與位置**逐點不變**。

**Non-Goals:**

- 不改變支撐樹的路由、互連、pillar/bridge 演算法。
- 不改變切片網格的建構方式（`minZ` 綁 elevation 的設計維持原狀）。相位敏感性由下游的方向性約束吸收，而非重新設計網格。
- 不新增任何設定參數、不改變既有參數語意或預設值。
- 不解決「極薄件支撐咬合不足可能脫落」的物理問題——已明確接受此代價（見 Decisions D4）。
- 不修改 `SupportTreeMesher`。防貫穿完全由夾限 `Head::penetration_mm` 達成。
- 不處理 SLA 光柵化階段（既有 `sla-layer-blur` / `sla-support-binary-rasterization` 的範疇）。

## Decisions

### D1｜表面吸附改用「方向性優先」而非「距離優先」（#1）

`move_on_mesh_surface()` 現行邏輯只比較距離：

```
hit = (!down || hit_up.distance() < hit_down.distance()) ? hit_up : hit_down;
```

支撐點的語意是「從下方頂住模型」，因此正確的目標面是**朝下的面**，與距離無關。改為三層決策：

```
1. 優先採用命中面法線 z 分量為負者（朝下面）
2. 若兩者皆朝下 → 取較近者
3. 若兩者皆非朝下（垂直壁、退化幾何）→ 回退至現行「取較近者」
```

第 3 層是刻意保留的相容出口：常態幾何下規則 1 與現行行為的結果本就一致（支撐點產生於零件的**最低**層，向下命中必為朝下面且距離較近），因此常態路徑逐點不變。

判準使用 `AABBMesh::hit_result::normal()`（來自 `normal_by_face_id()`，為幾何面法線）。**面法線不隨射線方向翻轉**，因此「從內側往上命中上表面」回傳的仍是 `(0,0,+1)`，可正確被排除。

*替代方案*：改為只往下打單一射線。**否決**——會喪失現行「點落在模型外側時往上拉回」的能力，屬行為縮減。

### D2｜`allowed_move` 以設定層高作 fallback，且必須排在 D1 之後（#2）

`size() < 2` 時 `m_model_height_levels[1]` 為越界讀取。改為：

```
allowed_move = (size() >= 2) ? (levels[1] − levels[0]) : layer_height   （+ epsilon 維持原樣）
```

**順序相依為硬性要求**。instrumented build 實測：該越界目前讀到 `0.0`，使 `allowed_move = −0.0749999`。負值讓 `hit.distance() <= allowed_move` 恆假，強制走入 `squared_distance` 分支，該分支投影到最近三角形——在薄板上恰好是下表面。**這個 UB 目前是唯一讓 1-level 相位正常運作的原因。**

修復後兩種相位的行為（板厚 0.2、`layer_height` 0.15）：

| 相位 | island 層 `print_z` | 向下命中距離 | `allowed_move` | 結果 |
|---|---|---|---|---|
| 2 levels | `a ∈ [0, 0.05]` | `a` | 0.15 | `a ≤ 0.15` → 位移至 z=0 ✅ |
| 1 level | `a ∈ (0.05, 0.15)` | `a` | 0.15（fallback） | `a ≤ 0.15` → 位移至 z=0 ✅ |

D1 + D2 合併後兩種相位收斂到同一結果，這正是 Goals 第一條的達成方式。

*替代方案*：僅修 D2 而不修 D1。**否決**——會使 2 個 level 的相位仍把點吸到上表面，且 1 個 level 的相位從「歪打正著正確」變成「同樣錯誤」，屬全面回歸。

### D3｜過濾階段以 clamp 取代丟棄，並補可觀測性（#3）

`filter_support_points_by_modifiers()` 的 `lower_bound` 回傳 `end()` 時無 `else` 分支。改為將索引 clamp 至 `slice_grid.size() − 1`，使 blocker/enforcer 遮罩以最後一層為準，點位保留。

**同時新增彙總日誌**：本次呼叫丟棄／clamp 的筆數、點位 z 範圍、`slice_grid` 首末值。此為本次調查耗費最多時間的環節——19 筆點位在此消失且無任何訊息，必須讓下一個人不必再做 instrumented build。

此項**單獨施作無效**：點位若仍在上表面，法線 `(0,0,+1)` 會在 `add_pinheads()` 被 `normal_cutoff_angle` 攔下。定位為防禦與可觀測性，非修復。

### D4｜防貫穿以夾限 `Head::penetration_mm` 達成，不動 Mesher（#5）

由 `pinhead()` 與 `get_mesh(const Head&)` 推導：

```
pinhead() 局部：  pin 球心 z = h = r_back + r_pin + length，球頂 = h + r_pin
get_mesh() 平移： z −= fullwidth() − r_back = 2·r_pin + width + r_back − penetration

平移後 pin 球頂 = length − width + penetration = penetration
                                （get_mesh 傳入 length = h.width_mm，故 length ≡ width）
```

**支撐頭刺入模型的最深點恰好等於 `penetration_mm`**，與 `r_pin`、`r_back`、`width` 皆無關。因此：

```
penetration = min(configured_penetration, local_thickness × 0.5)
```

即為「上表面零凸點」的**必要且充分**條件。`SupportTreeMesher` 完全不需修改。

**政策取捨（已由使用者裁定）**：優先保證幾何上表面完全乾淨，明確接受極薄件因咬合深度不足、列印時可能脫落的物理代價。

*替代方案 A*：在 `get_mesh()` 內夾限。**否決**——網格與邏輯模型脫鉤，`fullwidth()`／`junction()` 仍用未夾限值，主頭會與 pillar 產生最大 `configured_penetration` 的間隙。
*替代方案 B*：在參數驗證層警告使用者而不改幾何。**否決**——使用者已裁定採自動夾限。

### D5｜入模方向由 `−head.dir` 定義，射線沿頭軸量測（#5）

`get_mesh()` 的四元數為 `FromTwoVectors(Vec3f{0,0,−1}, h.dir)`，即局部 `−z` 映至 `h.dir`。故**局部 `+z`（刺入方向）映至 `−h.dir`**：

```
dir_in         = −head.dir                                    （已為單位向量）
local_thickness = query_ray_hit(hp + ε·dir_in, dir_in).distance() + ε
```

起點沿 `+dir_in`（即入模方向）**踏入材料內部** `ε` 以避開自我命中，量得距離後加回 `ε`。`ε` 取遠小於最薄可列印壁厚的量級。

**符號必須為 `+ε`，不可為 `−ε`**：`AABBMesh::query_ray_hit()` 沒有正/反面過濾，射線命中的是路徑上第一個三角形，不論其朝向。若起點沿 `−dir_in` 退到模型**外側**，第一個命中的必然是入射面本身，量得的「厚度」恆為 `2ε` 而與真實壁厚無關，夾限將在所有模型上失準且無聲。踏入內部後第一個命中才是出射面，量得的才是可用深度。此手法與程式庫既有慣用法一致（`SupportTreeUtils.hpp:252` 的 `query_ray_hit(ps + sd * n, n)`）。

（本節符號於實作階段更正，原文誤寫為 `−ε`；實作自始採用正確的 `+ε`，見 tasks.md 7a.1。）

**沿頭軸量測而非沿表面法線量測**是刻意選擇：`penetration` 本身即定義在頭軸上，故軸向距離才是正確的可用深度。此設計自然涵蓋兩種情形：

- **傾斜頭**：斜穿薄板的軸向可用深度大於垂直壁厚，夾限自動放寬，不會過度保守。
- **由側緣出射**：射線從板的側面（rim）出射時，量得的仍是軸向可用深度，夾限依然保證頭部尖端留在材料內。

**量測所用網格**為 `m_sm.emesh`，來源是 `get_mesh_to_print()`，已包含 hollowing 與 drill holes。因此中空件量到的是實際壁厚、鄰近排水孔的點量到的是縮減後的實際厚度——皆為正確語意。

### D6｜「先搜尋、後夾限」，且兩類提交點的語意不同（#5）

夾限**只執行一次**，在所有角度搜尋結束、最終物件提交的當下。角度搜尋全程使用設定值，故搜尋行為完全不變。此決策同時消除了 `w`（`= real_width() − penetration`，即頭部露在模型外的長度）與 optimizer 之間的循環相依——否則夾限依賴 `nn`、`nn` 由 optimizer 決定、optimizer 又以 `w` 為門檻。

四個提交點分為兩類，**語意不同且不可互換**：

```
【主頭 pinhead】提交點在 junction / pillar 計算之前
  DefaultSupportTree.cpp:527-531   （h.dir = nn 的接受區塊）
  SupportTreeUtils.hpp:408-412     （optimize_pinhead_placement 接受區塊）

  → fullwidth() 連帶更新 → junction_point() 外移 → pillar 由新 junction 起算
  → 頭與柱維持相連 ✅
  → 若不連帶更新，兩者將脫開最大 configured_penetration 的距離

【錨點 anchor】提交點在 add_anchor() 呼叫處
  DefaultSupportTree.cpp:742       （Default 樹）
  BranchingTreeSLA.cpp:298         （Branching / Organic 樹）

  → 橋接端點已固定，不受影響 → 拓撲零變化 ✅
```

**Branching / Organic 樹的時序為硬性要求**。`BranchingTreeSLA.cpp:291` 讀取 `anchor->junction_point()` 建立 `Junction toj` 作為橋接端點，而 `junction_point() = pos + (fullwidth() − r_back)·dir` 依賴 `penetration_mm`：

```
BranchingTreeSLA.cpp
  285  auto anchor = calculate_anchor_placement(...)        ← 角度搜尋
  291  sla::Junction toj = {anchor->junction_point(), ...}  ← ✗ 不可在此之前夾限
  292-297  （橋接可行性檢查）
  298  m_builder.add_anchor(*anchor)                        ← ✅ 夾限於此
```

若在 291 行之前夾限，`penetration` 變小 → `fullwidth()` 變大 → 橋接端點外移 → 後續檢查結果改變，「後夾限」所要保住的搜尋穩定度即告失效。

在 298 行夾限的幾何後果是安全的：`fullwidth()` 變大使 anchor 網格比 `toj` **更往外延伸**，橋接端點落在 anchor 罩體**內部**——是重疊而非脫開。

Default 樹另有一項簡化：`m_anchors` 僅被 `SupportTreeBuilder.cpp:131` 的 `get_mesh(anch, steps)` 消費，其 `junction()` **從未被讀取**，故該處夾限對拓撲零影響。但 `add_anchor()` 的第三參數 `w`（即 `length_mm`）由 `dist = ‖hitp − endp‖ + penetration` 導出，夾限時需連帶重算以維持一致。

### D7｜射線無命中採 Fail-safe（#5）

封閉流形網格上，自表面內側向內射出的射線**必定命中**。無命中即代表網格破損、非流形或自交。依「上表面零凸點優先」政策，此時 `penetration = 0`。

**必須輸出彙總警告日誌**（觸發點數量），使此情形可診斷。否則使用者只會看到支撐大量脫落而無從追查——這正是本次調查中 #3 造成的困境，不應複製。

*替代方案*：fail-open（保留設定值）。**否決**——破面模型上仍可能穿透，違反既定政策。

### D8｜`taildir` 正規化獨立列項（#6）

`connect_to_model_body()` 的 `Vec3d taildir = endp - hitp;` 未正規化（upstream 2.9.6 已於「Slicing SLA supports analytically」提交中修正，本 fork 因基底較舊而缺少）。

此為 D5 的**硬性前置**：非單位向量傳入 `query_ray_hit()` 會使回傳的 `m_t` 為參數 t 而非實際長度，夾限計算將失準且無聲（Release 下 `assert` 不存在）。

**獨立列項而非併入 #5**：正規化會改變 `add_anchor()` 收到的 `dir`，進而改變既有 anchor 的朝向與網格——這是既有缺陷的修正，其影響應可獨立審視、獨立驗證、獨立回退，不應被防貫穿的變更掩蓋。

### D9｜施作順序

```
#1 ─────────────────────────► 必須最先
  └─ #2                       （D2 順序相依：先修 #2 會全面回歸）
#3 ─── 獨立
#4 ─── 獨立
#6 ──────────────► #5         （D8：射線需單位向量）
                    └─ #5 的效果需 #1 完成後才觀察得到

建議序：#1 → #2 → #3 → #4 → #6 → #5
```

每一步皆須可獨立建置並通過既有測試，使二分定位（bisect）在回歸發生時仍然可用。

## Risks / Trade-offs

| 風險 | 界限 | 緩解 |
|---|---|---|
| **`w` 以未夾限值驗證**（D6 的必然後果）——提交後頭部向外多伸出 `configured − clamped`，超出淨空檢查所驗證的範圍 | ≤ `configured_penetration`（典型 0.4 mm） | 針對薄壁模型的 anchor 增加幾何回歸測試；於 design 記錄為已知取捨而非缺陷 |
| **Fail-safe 使破面模型支撐零咬合**（D7） | 僅限非流形／破損網格 | 彙總警告日誌，使用者可據以修模；不靜默 |
| **夾限僅在 `局部壁厚 ≥ 2 × penetration` 時失效**——壁厚 < 0.8 mm（@ penetration 0.4）的薄殼件會被靜默調降 | 影響帶為 `(0, 2 × penetration)` | 門檻明寫入 spec 驗收條件；**不得宣稱「常態零影響」**，須表述為「壁厚 ≥ 2 × penetration 時零影響」 |
| **`head_fullwidth()` 全域估算偏樂觀**——`SLAPrint.cpp:733` 的 elevation 驗證使用未夾限的全域值，夾限後實際 `fullwidth()` 更大 | 輕微，僅影響 elevation 下限驗證 | 記錄；若實測出現貼底碰撞再處理 |
| **#6 改變既有 anchor 幾何** | 僅影響 `connect_to_model_body()` 產生的錨點 | 獨立提交、獨立驗證，可單獨回退 |
| **與 upstream 的分歧擴大**——六項修改皆位於與 upstream 相同的檔案 | 4 個檔案 | 每項修改加註來源與意圖註解；未來 rebase 時逐項核對上游是否已修正 |
| **每個支撐點增加一次 raycast**（D5） | 每點 1 次，僅在提交點執行 | 相對於 pinhead GA optimizer 的數十至上百次評估可忽略；**不得**將量測放入 optimizer 的目標函式內 |

## Migration Plan

無資料格式或設定遷移。部署即生效。

**回退策略**：六項修改彼此可分離（除 #2 依賴 #1、#5 依賴 #6 外），採獨立提交，任一項出現回歸可單獨 revert。若需整體回退，還原全部六項即回到現行行為。

**驗證流程**（依施作順序逐項驗證，不累積）：

1. **#1 + #2**：`m020.stl`（0.2 mm）於 `support_object_elevation` 5.00→5.15 逐 0.01 全相位掃描，`layer_height` 取 0.05 / 0.10 / 0.15。判準：全部相位皆產出 19 點，且點位 `zmin = zmax = 0`（下表面）。
2. **#3**：構造使 `slice_grid` 最末值低於支撐點 z 的情境，確認點位保留且日誌輸出丟棄統計。
3. **#4**：`m050.stl` 搭配 `layer_height = 0.6`，確認訊息指向層高而非網格破損。
4. **#6**：對照 `connect_to_model_body()` 產生的錨點，確認方向與網格變化符合預期。
5. **#5**：支撐網格與模型網格布林交集後，**上表面之上無任何幾何**；破面模型確認 fail-safe 觸發並輸出警告。
6. **迴歸**：厚度 ÷ 層高 ≥ 2 且局部壁厚 ≥ 2 × penetration 的既有模型，支撐點數量與位置**逐點一致**。

## Open Questions

無。所有技術決策已於質詢階段收斂（D1～D9）。

實作期間若出現以下情形，需回到本文件重新決議而非就地判斷：

- D1 第 3 層回退分支在實測中被頻繁觸發（代表存在未預期的幾何類別）。
- D5 的 `ε` 選值在極薄件上造成量測誤差超過可接受範圍。
- D6 的 `w` 未夾限驗證在實測中確實產生可見碰撞（而非理論風險）。
