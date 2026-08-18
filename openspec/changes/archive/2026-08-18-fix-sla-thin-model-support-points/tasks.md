> **建置約束**：本專案的 C++ 編譯/建置一律由**使用者手動執行**。凡標註「🔨 使用者手動建置」的步驟，實作者（Claude Code）僅負責程式碼修改與提供驗證指令，**不得自行呼叫任何建置指令**。
>
> 使用者手動建置指令（於 `d:/repos/web_slicer_core/third_party/prusaslicer_build`）：
> ```
> cmake --build . --config Release --target PrusaSlicer_app_console
> ```
> 產物：`third_party/prusaslicer_build/src/Release/slicer-engine.exe`（同目錄含 `slicer_core.dll`）
>
> 共用驗證指令樣板：
> ```
> slicer-engine.exe --loglevel 5 --export-sla --output <OUT>.sl1 \
>   --export-support-stl --center 60,34 --load <CFG>.ini <MODEL>.stl
> ```
> 關鍵觀測值：日誌中的 `Automatic support points: <N>`
>
> **基準 binary 已隔離凍結於**：`third_party/prusaslicer_build/src/Release/baseline/`
> （`slicer-engine.exe` 僅為 164 KB 薄啟動器，以寫死的檔名動態載入**同目錄**的 `slicer_core.dll`；
>  若不隔離，重建後 baseline 會載入到新版 DLL，所有比對將靜默失效。後續比對一律使用此目錄。）
>
> 迴歸比對指令（`zero-change` 類別若有任何差異即 FAIL）：
> ```
> ./tests/data/sla_thin/snapshot.sh <新版 slicer-engine.exe> /tmp/now.tsv \
>     --compare tests/data/sla_thin/baseline_snapshot.tsv
> ```
>
> **施作順序為硬性相依**，不得跳號或併作：
> `#1 → #2 → #3 → #4 → #6 → #5`

## 1. 準備測試資產與基準快照

- [x] 1.1 將三顆測試模型自 `web_slicer_core/agent/jobs/` 複製至穩定路徑（建議 `tests/data/sla_thin/`）：`m020.stl`（0.2 mm，取自 job `9145a214`）、`m030.stl`（0.3 mm，取自 `3b172758`）、`m050.stl`（0.5 mm，取自 `e63be2d6`）
- [x] 1.2 建立基準設定檔 `cfg_base.ini`（以 job `9145a214/config.ini` 為底：`layer_height = 0.15`、`support_head_front_diameter = 0.8`、`support_head_penetration = 0.3`、`support_pillar_diameter = 1.0`、`support_points_density_relative = 150`、`support_object_elevation = 5.0`、`support_critical_angle = 30.0`、`pad_enable = 1`）
- [x] 1.3 撰寫相位掃描腳本：固定模型與層高，`support_object_elevation` 自 5.00 至 5.15 逐 0.01 掃描，輸出每次的 `Automatic support points` 數值
- [x] 1.4 選定 3～5 顆常態厚件（厚度 ÷ 層高 ≥ 2 且局部壁厚 ≥ 0.8 mm）作為迴歸基準模型
- [x] 1.5 🔨 **使用者手動建置**：以**未修改**的原始碼建置基準 binary，另存為 `slicer-engine-baseline.exe`
- [x] 1.6 產生並保存基準快照：對 1.4 的每顆常態模型匯出支撐 STL 與支撐點數量，作為後續逐點比對的黃金樣本
- [x] 1.7 **驗證檢查點**：以基準 binary 執行 1.3 的相位掃描（`m020.stl`），確認重現已知失敗樣態——`elev 5.00/5.01/5.02` 為 0 點、`5.03～5.12` 為 19 點、`5.13/5.14` 拋出切片例外、`5.15` 為 0 點

---

## 2. #1 支撐點吸附方向性（治本項）

- [x] 2.1 於 `src/libslic3r/SLA/SupportPointGenerator.cpp` 的 `move_on_mesh_surface()`（約 1596–1603 行）加入命中面方向性判定：以 `hit_result::normal()` 的 z 分量為負判定為「朝下面」
- [x] 2.2 實作三層決策：①優先取朝下面 ②兩者皆朝下時取較近者 ③兩者皆非朝下時回退既有「取較近者」邏輯
- [x] 2.3 加註來源註解，說明面法線不隨射線方向翻轉、以及第三層回退分支存在的相容性理由
- [x] 2.4 🔨 **使用者手動建置**
- [x] 2.5 **驗證檢查點 A**：`m020.stl` + `cfg_base.ini`（`elev = 5.00`）→ `Automatic support points` MUST 為 19（修復前為 0）
      ✅ PASS — 修復後 19 點、baseline 對照 0 點，支撐 STL 530,284 bytes
- [x] 2.6 **驗證檢查點 B**：對 `m020.stl` 執行 1.3 相位掃描 → `elev 5.00～5.12` MUST 全數為 19（`5.13/5.14` 仍為切片例外，屬 #4 範疇，此階段不處理）
      ✅ PASS — 5.00～5.12 全數 19 點，全滅相位由 4 個降為 0 個；5.15 亦由 0 轉為 19
- [x] 2.7 **驗證檢查點 C**：以 1.4 的常態厚件比對 1.6 的基準快照 → 支撐點數量與座標 MUST 逐點一致
      ✅ PASS — 3 顆 zero-change 模型的點數與完整 SHA256 全數一致（`frog_legs` observe 類亦未變）
- [x] 2.8 若 2.7 出現差異，記錄差異點的幾何特徵，確認是否為第三層回退分支被非預期觸發（對應 design.md Open Questions 第一項）
      ✅ N/A — 2.7 無任何差異，第三層回退分支未被非預期觸發，Open Questions 第一項不成立

---

## 3. #2 `allowed_move` 未初始化讀取修復

- [x] 3.1 於 `src/libslic3r/SLAPrintSteps.cpp` 的 `SLAPrint::Steps::support_points()`（約 884 行）將 `allowed_move` 改為：`size() >= 2` 時維持 `levels[1] − levels[0]`，否則使用設定的 `layer_height`；epsilon 項維持原樣
      註：fallback 取 `m_print->m_objects.front()->m_config.layer_height`，與 `slice_model()` 建立網格時的來源一致（切片網格為列印全域，非 per-object）
- [x] 3.2 移除或保留原 `assert` 並加註說明：該斷言在 Release 下不存在，故不可作為唯一防線
      註：改為 `assert(!empty())`。原 `assert(size() > 1)` 必須移除——`size()==1` 現為合法輸入，留著會讓 debug build 在正常薄件上中止
- [x] 3.3 加註順序相依警告註解：本修復必須在 #1 之後，先修本項會使原本靠未初始化負值歪打正著的相位一併失效
- [x] 3.4 🔨 **使用者手動建置**
- [x] 3.5 **驗證檢查點 A**：`m020.stl` 相位掃描 → `elev 5.00～5.12` MUST 全數維持 19 點（不得因本項而回退）
      結果 PASS：`5.00～5.12` 全數 19 點、無全滅相位；`5.15` 亦為 19。`5.13 / 5.14` 仍為 ERR，訊息與第 2 群組完全相同（"can not be sliced … mesh being broken"），屬既有缺陷，為 #4 的處理目標，不在本檢查點判準內
- [x] 3.6 **驗證檢查點 B**：確認 `m_model_height_levels.size() == 1`（如 `elev = 5.05`）與 `size() >= 2`（如 `elev = 5.00`）兩種組態皆產出 19 點且結果一致
      結果 PASS（點數層級）：`5.00` → 19 點 / 10604 tri，`5.05` → 19 點 / 10600 tri。幾何非位元相同屬預期——elevation 本身即為不同輸入，會改變切片網格相位與支柱長度；4 個三角形的差異來自此，非 fallback 不一致
      限制：日誌不輸出支撐點座標，故「逐點一致」在本檢查點只驗證到數量層級，未逐座標比對
- [x] 3.7 **驗證檢查點 C**：常態厚件比對基準快照 → MUST 逐點一致
      結果 PASS：`20mm_cube`(20)、`cube_with_hole`(14)、`reg_cyl25x30`(47) 三項 zero-change 與基準 SHA256 完全相同；`frog_legs`(66) 亦未變動
      註：本項對這些模型無可觀測行為變化屬正確結果——#2 僅在 `size()==1` 時生效，而 `cfg_base.ini` 的 `elevation = 5.0` 落在 `size()>=2` 相位

---

## 4. #3 Modifier 過濾 clamp 與診斷日誌

- [x] 4.1 於 `src/libslic3r/SLAPrintSteps.cpp` 的 `filter_support_points_by_modifiers()`（約 752–785 行）將 `lower_bound` 回傳 `end()` 的情形改為將索引 clamp 至 `slice_grid.size() - 1`，保留該支撐點
      註：原本被 `if (it != end())` 整段包住的遮罩判定已提升為無條件執行，clamp 後的點才會真正走完 enforcer / blocker 評估
      註：`slice_grid` 為空時不可計算 `size() - 1`（無號數下溢）。此時保持 `idx = 0`，而依函式開頭的 assert，遮罩此時亦必為空，所有 `idx < mask.X.size()` 皆為偽，不會越界
- [x] 4.2 加入彙總計數器，統計本次呼叫中被 clamp 的點位數量與其 z 範圍
      註：計數涵蓋所有被 clamp 的點，不論其後是否被 blocker 擋掉——統計的是「發生了幾次越界」，非「保留了幾點」
- [x] 4.3 僅在計數大於 0 時輸出診斷日誌，內容含：受影響筆數、點位 z 範圍、`slice_grid` 首末值；計數為 0 時不得輸出（避免日誌雜訊）
      註：採 `BOOST_LOG_TRIVIAL(debug)`（非 warning）。此情況現已被安全處理，升為 warning 會在 GUI／主控台造成無謂告警；debug 與鄰近的 `Automatic support points` 同級，loglevel 5 下可見，足供 4.5 驗證
      註：每次呼叫最多一行彙總，不逐點輸出，避免系統性越界時洗版
- [x] 4.4 🔨 **使用者手動建置**
- [x] 4.5 **驗證檢查點 A**：構造 z 超出 `slice_grid` 末值的支撐點情境，確認點位被保留且日誌輸出統計資訊
      結果 PASS，且不需人工構造——`frog_legs.obj` 於 `cfg_base.ini` 下自然觸發：
      `filter_support_points_by_modifiers: 1 of 67 support point(s) lie above the slice grid and were clamped to the last layer; point z range [1.925, 1.925], slice grid [-0.025, 1.925] (grid layers: 14)`
      點數 66 → 67，證實該點確被保留而非丟棄
      成因：點位 z 與網格末值列印出來同為 `1.925`，實際為 float 尾數之差（`sp.pos.z()` 略大於 `slice_grid.back()`），`lower_bound` 因而回傳 `end()`。原碼把這種 epsilon 級的查表落空當成刪除依據，正是本項要修的行為
      註：先以 `m020/m030/m050` × `layer_height {0.05,0.10,0.15}` × `elev 5.00~5.12`（共 117 組）掃描，均未觸發；薄件在 #1 修好後點位落於底面，不會越界。觸發者為常態件的頂層邊界點
- [x] 4.6 **驗證檢查點 B**：常態厚件切片 → MUST NOT 出現該診斷日誌（確認無誤觸發）
      結果 PASS：`20mm_cube`、`cube_with_hole`、`reg_cyl25x30`、`dish.stl` 診斷日誌行數皆為 0
      註：`frog_legs` 為唯一輸出者，且屬真實越界（見 4.5），非誤觸發
      註：`dish.stl`（本變更的原始問題模型）此時已產出 19 點
- [x] 4.7 **驗證檢查點 C**：`m020.stl` 全相位掃描與常態厚件快照比對 → MUST 皆無變化
      結果 PASS：相位掃描序列與第 3 群組完全相同（`5.00~5.12` 全 19、`5.13/5.14` ERR、`5.15` 19）
      快照比對：三項 zero-change 與基準 SHA256 完全相同；`m020/m030/m050` 亦與第 3 群組相同
      與第 3 群組快照逐欄 diff → **唯一差異為 `frog_legs`（66→67 點、31476→32052 tri）**，即 #3 的預期效果，範圍已隔離確認

---

## 5. #4 切片失敗錯誤訊息修正

- [x] 5.1 於 `src/libslic3r/SLAPrintSteps.cpp` 的 `SLAPrint::Steps::slice_model()`（約 691–694 行）改寫 `closest_slice_record()` 落空時的錯誤訊息：指出成因為層高相對模型高度過大，並建議調降層高
      新訊息帶入實際數值（物件名、物件高度、層高），placeholder 改用 boost::format 位置式 `%1% %2% %3%`（原為 `%s`），讓譯者可自由調換語序
      失敗條件已從程式碼核實：`closest_slice_record` 於「所有 slice level 皆低於模型底面」時回傳 `end()`。各記錄位於 `h − layer_height/2`，故物件高度低於約 1.5 倍層高時即可能發生，是否真的觸發取決於網格相位。相位細節寫在程式碼註解，未寫入使用者訊息（保持可操作性）
- [x] 5.2 移除訊息中「網格破損 / 修復模型」的敘述
      已完全移除。並於程式碼註解說明原訊息會把使用者導向去修一個本來就正常的幾何
- [x] 5.3 同步更新 i18n 條目（新增字串、標記舊字串為棄用）
      關鍵發現：**原字串根本不在任何語系檔中**。它雖有 `//TRN` 註解，卻未包在 `_u8L()` 內，`xgettext`（`--keyword=_u8L`）自始就抽不到它——所以「舊字串標記棄用」無對象，`grep "can not be sliced" resources/localization/` 為空即為證
      實際修正：新訊息包入 `_u8L()` 並保留 `//TRN` 註解。這才是本項真正的 i18n 缺陷所在
      未手動編輯 `.pot` / `.po`：`PrusaSlicer.pot` 由 CMake target `gettext_make_pot`（`CMakeLists.txt:517`）以 xgettext 產生，內含 `#: 檔案:行號` 來源參照且現存參照已過期（顯示 329，實際 401）。手改會寫入錯誤行號且下次重產即被覆蓋
      已確認 `src/libslic3r/SLAPrintSteps.cpp` 列於 `resources/localization/list.txt:18`，故重產 pot 時新字串必被收錄
- [x] 5.4 🔨 **使用者手動建置**
- [x] 5.5 **驗證檢查點 A**：`m050.stl` + `layer_height = 0.6` → 錯誤訊息 MUST 指向層高，MUST NOT 提及網格破損
      結果 PASS：`Model named: m050.stl can not be sliced: no slice level falls inside the model. The layer height (0.6 mm) is too large relative to the height of the object (0.5 mm). Try lowering the layer height.`
      `grep -rn "mesh being broken\|Repairing it might fix" src/` → 無殘留
- [x] 5.6 **驗證檢查點 B**：`m020.stl` + `layer_height = 0.15` + `elev = 5.13` → 確認同一訊息路徑被觸發且文字正確
      結果 PASS：`... The layer height (0.15 mm) is too large relative to the height of the object (0.2 mm). ...`
      數值正確代入，證實 boost::format 位置式 `%1% %2% %3%` 對應無誤（`%2%`=物件高度、`%3%`=層高，順序與宣告順序相反仍正確）
      待人工判斷：此例層高 0.15 < 物件高 0.2，訊息卻說「層高過大」。以實際失敗條件（物件高 < 約 1.5 倍層高）而言正確，但使用者可能覺得字面矛盾。措辭是否調整由使用者決定
- [x] 5.7 **驗證檢查點 C**：確認 i18n 無遺漏字串（建置無翻譯警告）
      **已修正並複驗通過**（`//TRN` 註解已下移至緊貼 `_u8L(` 上方，並加註置放規則以免日後被「整理」回去）
      以 CMake target `gettext_make_pot` 完全相同的參數對實檔重跑 `xgettext`，結果：
      ```
      #. TRN To be shown at the status bar on SLA slicing error. %1% is the
      #. object name, %2% the object height and %3% the layer height in mm.
      #: src/libslic3r/SLAPrintSteps.cpp:709
      #, possible-boost-format
      msgid "Model named: %1% can not be sliced: no slice level falls inside the model. ..."
      ```
      字串、譯者註解、boost-format placeholder 驗證三者皆到位，xgettext 無警告或錯誤
      成因記錄：`--add-comments=TRN` 只認「結束於含 keyword 那一行的前一行」的註解。原置於 `throw Slic3r::RuntimeError(format(` 上方，`_u8L(` 在其下一行，故失效。已用最小測資雙向交叉驗證（在 `throw` 上方→抽不到；在 `_u8L` 上方→抽得到，多行續行亦正常）
      註：`possible-boost-format`（而非 `boost-format`）源自 CMake target 帶的 `--debug` 旗標，非退化；正式 `PrusaSlicer.pot` 全檔 321 筆皆為 `possible-boost-format`、0 筆 `boost-format`
      註：原字串本就有相同的置放問題，只是它連 `_u8L()` 都沒有，問題被掩蓋
      未驗證項：使用者手動執行的 5.4 建置無 log 留存，故「建置無編譯警告」我未能檢視，不予聲稱

---

## 6. #6 `taildir` 正規化（#5 的前置依賴）

- [x] 6.1 於 `src/libslic3r/SLA/DefaultSupportTree.cpp` 的 `connect_to_model_body()`（約 731 行）將 `Vec3d taildir = endp - hitp;` 改為正規化形式
      額外加入零長度防護（**upstream 沒有，是本 fork 新增**）：`endp` 與 `hitp` 在 mini pillar 尾寬塌成 0 且為垂直下掃時會重合，Eigen 的 `normalized()` 對零向量會無聲產生 NaN 並流入錨點網格。原碼退化成零向量——同樣無意義，但至少有限。門檻取 `squaredNorm() > EPSILON * EPSILON`，退化時取正上方（非退化時該向量必朝上：`endp.z() − hitp.z() == h >= 0` 且 x/y 相同）
- [x] 6.2 加註來源說明：本項為補回 upstream 2.9.6 已有、本 fork 因基底較舊而缺少的修正，且為 #5 射線量測的前置條件
      已核對 `D:/repos/PrusaSlicer/src/libslic3r/SLA/DefaultSupportTree.cpp:706` → `Vec3d taildir = (endp - hitp).normalized();`。該函式其餘 20 行與本 fork **逐字相同**，差異僅此一處，確認為單純的版本落後而非本 fork 的刻意分歧
- [x] 6.3 確認 `add_anchor()` 呼叫端不依賴 `taildir` 的原始長度（若有，需一併調整）
      結論：不依賴，無需額外調整。長度由下一行的 `dist = (hitp − endp).norm() + head_penetration_mm` 自同兩點獨立重算，`w` 再由 `dist` 導出；`taildir` 僅作為方向傳入
      `add_anchor(...)` → `Anchor : Head`，`Head` 建構子（`SupportTreeBuilder.cpp:15`）將 `dir` **原封不動**存下，不做正規化，故非單位向量會一路傳播
      **重要發現，與 6.6 的預期相反**：受非單位 `dir` 影響的是 `Head::junction()`（`pos + (fullwidth() − r_back_mm) * dir`），但 Default 樹的錨點只有一個消費者——`SupportTreeBuilder.cpp:131` 的 `get_mesh(anch, steps)`。該函式（`SupportTreeMesher.cpp:220`）只用 `h.dir` 餵給 `Quaternion::FromTwoVectors`（內部自行正規化）並以 `h.pos` 平移，**完全不碰 `junction()`**
      因此 **#6 單獨施作預期不會產生任何可見的幾何變化**，它是潛在正確性修復兼 #5 的前置條件。6.6 原本寫「此項預期會有差異」，依此分析應反轉為「預期無差異；若出現差異反而代表有未預期的耦合」——待 6.5～6.7 實測確認
      另確認：`BranchingTreeSLA.cpp:291` 的 `anchor->junction_point()` 走的是 `calculate_anchor_placement()` 另一條建構路徑，不受本項影響
- [x] 6.4 🔨 **使用者手動建置**
- [x] 6.5 **驗證檢查點 A**：選一顆會產生 model-body 錨點的模型（需模型本體上方有懸空結構），匯出支撐 STL，確認錨點朝向合理、無異常拉長的幾何
      **既有模型全數不適用**：以「切換 `support_buildplate_only` 是否改變支撐 STL」為偵測器，逐一測試 `20mm_cube` / `cube_with_hole` / `reg_cyl25x30` / `frog_legs` / `dish.stl` → 五顆**全部無 model-body 連接**，完全不會走進 `connect_to_model_body()`。若不另備模型，本群組等同未被驗證
      新增測試資產 `tests/data/sla_thin/anchor_tunnel.stl`（20×20×10 方塊貫穿一道 12×4 的矩形隧道，y 向貫通）。隧道天花板為朝下面且正下方即隧道地板，支撐點的 DOWN 射線必命中模型本體
      確認生效：`support_buildplate_only = 1` 時支撐 STL **完全空白（0 bytes）**，`= 0` 時 684,884 bytes / 40 點 → 該模型的支撐**全部**依賴 model-body 連接
      幾何健檢（現行版與基準版各 3 次）：
      `tri=12928~13696  NaN/Inf_tris=0  bbox x[-0.074,24.501] y[-4.465,24.387] z[-7.000,10.200]  max_edge=28.6~28.8mm`
      → 無 NaN/Inf（零長度防護有效，未產生 NaN 頂點）、bbox 兩版**完全相同**且與模型尺寸相符、無異常拉長。PASS
- [x] 6.6 **驗證檢查點 B**：與基準快照比對，記錄錨點幾何的變化範圍；此項**預期會有差異**，需人工確認差異為修正而非破壞
      **原預期需修正為「預期無差異」**，實測與 6.3 的追查分析一致：對全部 7 顆決定性模型，本群組快照與第 4 群組**逐欄完全相同**（含 `frog_legs` 67 點 / 32052 tri / `df298922354fae87`）。#6 未造成任何可見幾何變化
      理由（6.3 已述）：Default 樹錨點唯一的消費者是 `get_mesh(anch)`，其以 `Quaternion::FromTwoVectors` 取方向（內部自行正規化）並以 `h.pos` 平移，從不觸及受 `|dir|` 影響的 `junction()`
      **附帶重大發現：model-body 錨點的產生具不確定性（nondeterministic）**。以 `anchor_tunnel.stl` 同一 binary、同一設定連跑 6 次得到 3 種不同輸出（`646484` bytes 兩種 SHA、`684884` bytes 一種）。**基準版同樣如此**（12928/13696 兩種三角形數皆出現），故為既有行為，非本變更引入
      推測成因：`m_pillar_index.guarded_insert()` / `nearest()` 的空間索引在多執行緒下插入順序不定，導致支柱連接路由不同。**尚未定位確認，僅為推測**
      影響：`anchor_tunnel.stl` **不可**用於 SHA256 逐位元比對。本項因此改以「bbox + NaN + 三角形數區間」判定，並以決定性模型承擔逐點比對責任
      已複核既有快照模型仍具決定性：同一 binary 連跑兩次，7 顆模型的 tsv **逐行完全相同**
- [x] 6.7 **驗證檢查點 C**：不產生 model-body 錨點的常態模型 → MUST 逐點一致
      結果 PASS：`20mm_cube`(20)、`cube_with_hole`(14)、`reg_cyl25x30`(47) 三項 zero-change 與 `baseline/` 隔離目錄的黃金快照 SHA256 完全相同
      這三顆已於 6.5 的偵測器確認無 model-body 連接，故確實落在本檢查點的適用範圍內

---

## 7. #5 動態刺入深度防貫穿

### 7a. 共用量測工具

- [x] 7a.1 實作局部可用深度量測輔助函式：入模方向 `dir_in = −head.dir`，射線起點自接觸點沿 `−dir_in` 退開 `ε`，回傳 `query_ray_hit(...).distance() + ε`
      位置：`src/libslic3r/SLA/SupportTreeUtils.hpp`，`clamped_head_penetration()`（置於 `optimize_pinhead_placement()` 之前，Default 樹經 `SupportTreeUtilsLegacy.hpp:8` 取得，Branching 樹直接 include）
      **⚠ 與 design.md D5 的公式不符，已依實際語意修正**：design 寫 `query_ray_hit(hp − ε·dir_in, dir_in)`。沿 `−dir_in` 退開會落在模型**外側**，而 `query_ray_hit()` 無正/反面過濾，射線第一個命中必為**入射面本身**（距離 ≈ ε），量得的厚度恆為 `2ε`，並非壁厚
      實作採 `hp + ε·dir_in`：先踏入材料內部越過入射面，再量到出射面，最後把 ε 加回。此與程式庫既有慣用法一致（`SupportTreeUtils.hpp:252` 的 `query_ray_hit(ps + sd * n, n)`）。**design.md D5 的符號應更正**（本回合未修改 design.md，僅記錄）
- [x] 7a.2 選定並以具名常數定義 `ε`（需遠小於最薄可列印壁厚），加註選值理由
      `PENETRATION_RAY_EPSILON = 0.01`（10 µm）。理由已寫入註解：較 SLA 最薄可列印壁厚（約 0.2 mm）小兩個數量級，亦遠小於典型 XY 像素（35–50 µm），故踏入此距離不可能跨越真實壁面；且該步長於量測後加回
- [x] 7a.3 加入方向向量正規化的前置保證（不得僅依賴 `assert`，Release 下亦須成立）
      以 `dirnorm = head_dir.norm()` 實際相除，非 assert。條件寫成 `if (!(dirnorm > EPSILON))` 而非 `<=`，使 NaN（任何比較皆為偽）一併落入 fail-safe 分支
      理由已註解：`query_ray_hit()` 回傳的參數是方向向量長度的倍數，非單位向量會讓所有距離被靜默縮放，而 assert 在 Release 下不存在
- [x] 7a.4 實作夾限函式 `min(configured_penetration, local_thickness × 0.5)`，並在射線無命中時回傳 0（fail-safe）
      `!hit.is_hit()` 即歸零。`is_hit()` 本身已含 `!isinf(m_t)` 檢查（`AABBMesh.hpp:99`），故未重複判斷
      `configured_penetration <= 0`（含 NaN）時直接回傳 0，不浪費射線
- [x] 7a.5 加入 fail-safe 觸發次數的彙總計數器（供 7e 輸出日誌）
      `struct PenetrationClampStats { std::atomic<size_t> ray_misses{0}; }`，以可選指標參數傳入（預設 `nullptr`）
      必須為 atomic：`add_pinheads()` 的 `filterfn` 經 `execution::for_each` 跨工作執行緒執行
      已於 `DefaultSupportTree` 加入成員 `m_penetration_stats`（`DefaultSupportTree.hpp`）。已確認該類別全程僅 `DefaultSupportTree alg(builder, sm);` 就地建構，未被複製或移動，故 atomic 成員不可複製不成問題
- [x] 7a.6 加註實作紀律：本量測**不得**置於任何 optimizer 的目標函式內，每個支撐頭僅執行一次
      註解標為 `DISCIPLINE:`，並寫明理由：若置於目標函式內，夾限會依賴方向，而方向又由含夾限的分數決定，形成循環相依

### 7b. Default 樹主頭

- [x] 7b.1 於 `src/libslic3r/SLA/DefaultSupportTree.cpp` 的 `add_pinheads()` 接受區塊（約 527–531 行，`h.dir = nn` 處）於 `h.dir` 賦值後套用夾限，寫入 `h.penetration_mm`
      以 `hp`（`m_points.row(fidx)`）為接觸點、`nn` 為頭軸方向。已核對 `m_points.row(i)` 即 `m_sm.pts[i].pos`（`DefaultSupportTree.cpp:53-55`），與 `heads[fidx].pos` 同值
      夾限只寫在**接受分支**內。`back_r` 回退分支（`else if`）會遞迴重呼 `filterfn`，最終仍落在接受分支，故不會漏夾亦不會重複夾
- [x] 7b.2 確認夾限位於 junction / pillar 計算之前，使 `fullwidth()` 與 `junction_point()` 連帶更新
      已核對時序：`filterfn` 於 `execution::for_each`（`DefaultSupportTree.cpp:538`）中完成 → 其後才 `m_builder.add_head()`（約 545 行）→ 更後才由 `classify()` / `routing_to_ground()` 產生 junction 與 pillar。夾限發生在最前，下游全部自動採用夾限後的 `fullwidth()`
      註解已寫明反面後果：若延後夾限，pillar 由舊 junction 起算而頭部網格止於新 junction，兩者最大脫開 `configured_penetration`
- [x] 7b.3 🔨 **使用者手動建置**
- [x] 7b.4 **驗證檢查點 A**：`m020.stl`（板厚 0.2、`penetration = 0.3`）→ 匯出支撐 STL 與模型取布林交集，**上表面之上 MUST 無任何幾何**
      改以「支撐 STL 最高 z」直接判定，較布林交集精確且無容差問題。`m020.stl` 本體 z 範圍 `[0.000, 0.200]`，支撐與模型同座標系
      結果 PASS：`penetration = 0.3` 時支撐最高 z = **0.1000**，較上表面 0.200 低 0.1 mm。**上表面之上零幾何**。未夾限時尖端會抵達 0.300，高出上表面 0.1 mm
      三模型 × 兩設定共 6 組，實測值與 `min(configured, thickness × 0.5)` **四位小數完全相符**：
      ```
      模型  板厚   設定pen  厚度×0.5  預期    實測
      m020  0.200  0.05     0.1000    0.0500  0.0500
      m020  0.200  0.30     0.1000    0.1000  0.1000
      m030  0.300  0.05     0.1500    0.0500  0.0500
      m030  0.300  0.30     0.1500    0.1500  0.1500
      m050  0.500  0.05     0.2500    0.0500  0.0500
      m050  0.500  0.30     0.2500    0.2500  0.2500
      ```
      `pen = 0.05` 組為對照：夾限**未**觸發，設定值原樣生效 → 證明實作是 `min()` 而非寫死取半，且射線量得的厚度與實際板厚一致
- [x] 7b.5 **驗證檢查點 B**：檢視支撐頭與其 pillar 的接合處，MUST 無幾何間隙
      作法：沿某支撐頭尖端的 xy 打一條垂直線，列出所有與支撐網格的交會 z，比較不同 `penetration` 設定下哪些特徵移動、移動多少
      ```
      pen=0.05  -7.0000 -5.0000 -4.0000 -2.7500 -2.2500 0.0500
      pen=0.10  -7.0000 -5.0000 -4.0000 -2.7000 -2.2000 0.1000
      pen=0.30  -7.0000 -5.0000 -4.0000 -2.7000 -2.2000 0.1000
      ```
      結果 PASS，兩項證據：
      ① `pen=0.05 → 0.10` 時，**頭部相關特徵（尖端 0.05→0.10、junction −2.25→−2.20、−2.75→−2.70）全部同向平移恰好 0.05 mm**，而接地特徵（−4.0 / −5.0 / −7.0）完全不動。頭、junction 與柱頂為剛體同動，未產生相對位移，故無間隙
      ② `pen=0.30` 的交會 z **與 `pen=0.10` 逐項完全相同**，證明夾限確實把 0.30 映到 0.10，非近似
      補充：另以量化網格連通元件法（grid 0.02 / 0.05 / 0.10）比對夾限前後 → 皆為 39 個元件、最大 876、最小 128，結構未因夾限碎裂
      **方法限制須記錄**：支撐 STL 是重疊基本體的聯集而非封閉流形，`its_merge()` 不焊接頂點，故 even-odd 實心區間判定與頂點鄰近連通法**皆無法直接證明「兩個實體重疊」**。上述結論建立在「相對位移為零」，而非直接量測重疊量
- [x] 7b.6 **驗證檢查點 C**：常態厚件（壁厚 ≥ 0.8 mm）→ 支撐網格 MUST 與基準快照完全相同
      結果 PASS：三顆 zero-change 與 `baseline/` 隔離目錄的完整 SHA256 **逐字元相同**
      ```
      20mm_cube.obj       60cb3dc9379701753a4c8301b040c30d7aa9c9aed02d9be5e5a80e403a7520ab
      cube_with_hole.obj  9cc71f1e3d8734d20bb57d717ea24f6023ca9381e84c55ec3f9eaadbeb0d2588
      reg_cyl25x30.stl    c9da88c24f561ae7fd3a6456e51c2d5d541c54ae180282df7fd99b256c294a85
      ```
      → 壁厚 ≥ 2 × penetration（0.6 mm）時夾限確實不生效，常態件零影響
      **`frog_legs` 幾何已變動，需人工確認**：點數仍 67、三角形數仍 32052，但 SHA 與第 6 群組不同；支撐最高 z 由 **2.249 → 2.103（降低 0.146 mm）**。代表該模型部分區域局部壁厚 < 0.6 mm，夾限確實在其上生效。變動量 0.146 mm 落在設計上限（≤ configured penetration = 0.3 mm）之內
      `m020 / m030 / m050` 三顆薄件亦如預期改變 SHA（點數與三角形數皆不變，僅頭部縮短）

### 7c. Default 樹錨點

- [x] 7c.1 於 `connect_to_model_body()` 的 `add_anchor()` 呼叫處（約 742 行）套用夾限
      量測參數：接觸點 `hitp`、頭軸 `taildir`。已核對方向正確——錨點是反向 pinhead，其針沿頭部局部 +z 前進，而 `get_mesh()` 將局部 +z 映至 `−dir`；`taildir` 由模型表面指向柱端（朝上），故 `−taildir` 朝下刺入模型本體，正是應量測的方向
      夾限置於 `taildir` 計算之後（必須，量測需要方向），且在 `dist` 之前（見 7c.2）
      已確認 `connect_to_model_body()` 由 `routing_to_model()` 的 `execution::for_each` 跨執行緒呼叫，`m_penetration_stats` 的 atomic 計數為必要而非保險
- [x] 7c.2 連帶重算 `w`（`add_anchor()` 第三參數 `length_mm`，由 `dist = ‖hitp − endp‖ + penetration` 導出），使其與夾限後的刺入深度一致
      `dist` 與 `w` 皆改用夾限後的 `anchor_penetration`，`add_anchor()` 的第四參數同步傳入同一值。若只換第四參數而 `w` 仍依設定值計算，錨點長度與咬合深度會相差恰好被削掉的量
      **與主頭的時序不同，且是刻意的**：主頭必須「後夾限」以保護角度搜尋；此處 `taildir` 純由幾何導出、無 optimizer 參與，故夾限在 `dist` 之前反而是唯一能保持一致的位置。此差異已寫入程式碼註解
      `w < 0` 的既有警告分支保留，作用於重算後的 `w`。夾限使 `dist` 變小，故該分支被觸發的機會略增（既有行為為夾至 0，未更動）
      已確認函式內無殘留的未夾限 `cfg.head_penetration_mm`：全檔剩餘 5 處分別為 Head 建構預設（420）、角度搜尋門檻（474 / 479，依 D6 必須維持設定值）、以及 7b / 7c 兩處夾限呼叫的輸入參數（548 / 786）
- [x] 7c.3 🔨 **使用者手動建置**
- [~] 7c.4 **驗證檢查點 A**：薄壁模型上的 model-body 錨點 → 布林交集確認承載面另一側無幾何
      **部分達成。取得了強力的防貫穿實證，但未能建構出「錨點落在薄板上」的隔離案例。**
      ✅ 已驗證（`anchor_tunnel.stl`，全部支撐皆依賴 model-body 連接，`support_buildplate_only = 1` 時支撐 STL 為空）：
      模型頂面 z = 10.000。**基準版支撐最高 z = 10.1996（凸出頂面 0.1996 mm）→ 現行版 10.0061（凸出 0.0061 mm）**，兩版各 2 次執行皆穩定重現
      該特徵位於模型外緣（x ≈ 19.885 → 19.691、y ≈ 19.885 → 19.691），沿頭軸同時向下與向內縮回 0.194 mm，正是 design D5 明列的「射線由側緣出射」情形
      ⚠ **無法歸因於 7c 或 7b**：手上沒有「僅含 7b」的 binary，此差異為 7b + 7c 的合併效果
      ❌ 未達成：兩次嘗試建構「錨點落在薄板、可隔離觀測另一側」的模型皆失敗——
      ① `anchor_thinfloor.stl`（外框 z 0–10、隧道 z 0.2–7、地板厚 0.2）：`support_buildplate_only` 偵測顯示確有 model-body 連接，但地板附近 (`x 4.5–15.5`, `z ∈ [−0.3, 0.3]`) **完全無支撐頂點**，隧道內支撐未落到地板
      ② 同形狀但保持可運作的隧道高度（外框 z 2.8–10、隧道 z 3.0–7.0、地板厚 0.2）：支撐最高僅達 z ≈ 4.6，隧道天花板根本未生成支撐
      推測 `routing_to_model()` 會先嘗試 `search_pillar_and_connect()` 橋接既有柱體，成功後即不產生錨點；錨點是否落在指定薄板為路由結果，非可直接指定。**未定位確認**
      註：`anchor_thinfloor.stl` 保留於測試資產中並記錄其「不會產生地板錨點」，以免後人重複此嘗試
- [~] 7c.5 **驗證檢查點 B**：確認錨點網格涵蓋橋接端點位置，無脫開
      **以結構論證 + 迴歸實證支持，未直接量測重疊。**
      結構論證（design D6，已於 6.3 逐一核對程式碼）：Default 樹的 `m_anchors` 唯一消費者是 `SupportTreeBuilder.cpp:131` 的 `get_mesh(anch, steps)`，`Anchor::junction()` **從未被讀取**。橋接端點不由錨點的 `junction_point()` 決定，故此處夾限對橋接拓撲零影響
      實證：`anchor_tunnel.stl` 現行版 3 次執行皆 `NaN/Inf = 0`、bbox 穩定、無破碎幾何；`snapshot.sh` 的 7 顆模型與 7b 完成後快照**逐欄完全相同**（見 7c.6），未出現任何脫開徵兆
      **限制須記錄**：與 7b.5 同因——支撐 STL 是重疊基本體的聯集、`its_merge()` 不焊接頂點，故無法直接量測「兩實體是否重疊」。且 model-body 錨點的路由具不確定性（第 6 群組記錄，Out-of-Scope），逐位元比對不可用
- [x] 7c.6 **驗證檢查點 C**：常態厚件錨點 → MUST 與 #6 完成後的快照一致
      結果 PASS：三顆 zero-change 與 `baseline/` 完整 SHA256 逐字元相同（`20mm_cube` 20 點、`cube_with_hole` 14 點、`reg_cyl25x30` 47 點）
      與 **7b 完成後快照逐欄 diff → 完全相同**（含 `frog_legs` 與三顆薄件）。7c 對這 7 顆決定性模型零影響，與「它們皆不產生 model-body 錨點」（6.5 偵測結論）一致

### 7d. Branching / Organic 樹

- [x] 7d.1 於 `src/libslic3r/SLA/SupportTreeUtils.hpp` 的 `optimize_pinhead_placement()` 接受區塊（約 408–412 行）套用主頭夾限
      夾限寫在 `head.r_back_mm = back_r;` 之後、`ret = true;` 之前，與 Default 樹的 `add_pinheads()` 對稱；optimizer 目標函式全程使用設定值，未受影響
      `back_r` 回退分支的遞迴呼叫已一併傳遞 `stats`，不會漏計
      **branching 樹特有的時序理由（已寫入註解）**：`create_branching_tree()` 於本函式回傳後立即以 `h->junction_point()` 建立分支葉節點（`BranchingTreeSLA.cpp:396`）。夾限使 `fullwidth()` 變大、junction 外移，葉節點隨之外移——這正是分支與頭部保持相連的原因。若延後夾限，分支會停在頭部已不在的位置
      計數器串接：`optimize_pinhead_placement()` 與 `calculate_pinhead_placement()` 均新增 `PenetrationClampStats *stats = nullptr` 參數（預設值，不影響既有呼叫端）
- [x] 7d.2 於 `src/libslic3r/SLA/BranchingTreeSLA.cpp` 的 `m_builder.add_anchor(*anchor)` 呼叫處（約 298 行）套用錨點夾限
      量測參數 `anchor->pos`（`calculate_anchor_placement()` 中即 `to_hint`，取自 `branchingtree::sample_mesh()`，位於模型表面）與 `anchor->dir`（由表面指向橋接端，`−dir` 為刺入方向），方向慣例與 Default 樹的 `taildir` 一致
      計數器：`BranchingTreeBuilder` 新增公開成員 `PenetrationClampStats *penetration_stats = nullptr`，由 `create_branching_tree()` 於建構後指向其區域變數，使**主頭與錨點兩條路徑累加至同一計數器**（供 7e 單一彙總）
- [x] 7d.3 **確認夾限位置絕不在第 291 行 `anchor->junction_point()` 之前**，並加註此為硬性約束的理由（否則橋接端點位移，破壞「後夾限」的搜尋穩定度）
      已確認並實作於 `add_diffbridge()` 之後、`add_anchor()` 之前，位於 `toj` 與 `beam_mesh_hit()` 兩者的下方
      註解以 `ORDER IS LOAD BEARING:` 標記，明寫：夾限使 penetration 變小 → `fullwidth()` 變大 → 若在 `toj` 之前夾限，橋接端點外移，其下的 `beam_mesh_hit()` 可行性檢查結果改變，連「哪些橋接被接受」都會變，後夾限所要保住的搜尋穩定度即失效；並明寫「不要把這行往上搬」
      同時記錄夾限置於此處的幾何安全性：`fullwidth()` 變大使錨點網格延伸得比 `toj` 更外，橋接端點落在錨點本體**內部**，是重疊而非脫開
- [x] 7d.4 確認 `optimize_anchor_placement()` 的 `stop_score(anchor.fullwidth())` 仍以未夾限值計算
      確認成立且**無需修改**：該搜尋位於 `calculate_anchor_placement()` 內，於 7d.2 的夾限之前完成；此時 `anchor.penetration_mm` 仍為 `calculate_anchor_placement()` 建構時傳入的 `sm.cfg.head_penetration_mm`
      已於該處補上防護註解，說明 `stop_score` 與函式末端的驗收判定（`oresult.score < anchor.fullwidth()`）**兩者皆以 `fullwidth()` 為準**，故提前夾限會同時放大錨點、抬高搜尋門檻並改變接受結果；維持設定值才使夾限成為純粹的後處理
- [x] 7d.5 🔨 **使用者手動建置**
- [x] 7d.6 **驗證檢查點 A**：以 `support_tree_type = branching` 切 `m020.stl` → 布林交集確認上表面之上無幾何
      **關鍵陷阱（第一次量測因此失效）**：branching 模式讀的是 `branchingsupport_head_penetration`，**不是** `support_head_penetration`（`SLAPrint.cpp:97` 的 `make_support_cfg()`）。首次以 `support_head_penetration` 掃描，三組設定得到完全相同的結果，實為設定根本未生效
      改用正確參數後，結果與 `min(configured, 厚度 × 0.5)` **四位小數完全相符**（板厚 0.200、頂面 z = 0.200）：
      ```
      設定pen  預期夾限  實測支撐最高z  點數
      0.05     0.0500    0.0500         32
      0.10     0.1000    0.1000         32
      0.30     0.1000    0.1000         32
      0.60     0.1000    0.1000         32
      ```
      PASS：四組的支撐最高 z 皆 ≤ 0.1000，**低於上表面 0.200，上表面之上零幾何**。未夾限時 `pen = 0.60` 的尖端會抵達 0.600，高出上表面 0.4 mm
      `pen = 0.05` 為對照組，夾限未觸發、設定值原樣生效 → 證明為 `min()` 而非寫死取半
      基準版對照不可得：branching 模式下 `m020.stl` 於基準版為 0 點、無支撐 STL（即 #1 的原始缺陷），符合已知失敗樣態
- [~] 7d.7 **驗證檢查點 B**：比對夾限前後的橋接端點座標 → MUST 完全相同（證明拓撲零變化）
      **程式碼層級已確認，但未能以實測案例驗證錨點路徑。**
      ✅ 程式碼層級：夾限位於 `toj`（`anchor->junction_point()`）與 `beam_mesh_hit()` 之後、`add_anchor()` 之前，且 `add_diffbridge(fromj.pos, toj.pos, …)` 亦在夾限之前執行 → 橋接端點由未夾限的 `fullwidth()` 決定，本項變更不可能移動它。已以 `ORDER IS LOAD BEARING:` 註解鎖定（7d.3）
      ✅ 主頭路徑實測：`m020.stl` branching，`pen = 0.30` 與 `pen = 0.60`（兩者皆夾限至 0.100）→ 輸出**位元組完全相同**（`bytes=1043284`、`rawsha=5be1920df730`、`tri=20864`、`zmax=0.1000`），且各自連跑 2 次皆可重現。分支葉節點與整棵樹的拓撲完全由夾限後的值決定，未因設定值不同而位移
      ❌ 未達成：找不到「branching 樹的 mesh anchor 確實建立、且其夾限確實觸發」的測試案例——
      `m020.stl` 切換 `branchingsupport_buildplate_only` 雖有輸出差異，但 `pen 0.30 / 0.60` 位元組完全相同，代表**實際上沒有任何成功的 mesh anchor**；該差異來自 `ground_facing_only` 會整個跳過 `sample_mesh()`（`BranchingTreeSLA.cpp:410`），改變的是節點雲而非錨點
      `frog_legs.obj` branching 確有 mesh anchor，但其最高特徵未觸發夾限（`pen 0.30 → zmax 0.3000`、`pen 0.60 → zmax 0.6000`，皆為未夾限值），無法作為本項的觀測對象
      即：`toj` 專屬的「座標完全相同」主張僅有程式碼層級證據，**無實測支持**
- [x] 7d.8 **驗證檢查點 C**：branching 樹的常態厚件 → MUST 與基準快照一致
      結果 PASS：三顆常態厚件於 branching 模式下，現行版與 `baseline/` 隔離目錄**點數、最高 z、SHA256 全部相同**
      ```
      模型                  點數  zmax     SHA256(前16)
      20mm_cube.obj         40    0.2000   386ac99b66afde61   （新舊相同）
      cube_with_hole.obj    49    0.2000   da1f1b614a63a401   （新舊相同）
      reg_cyl25x30.stl      59    0.2000   ce5468aa885647f9   （新舊相同）
      ```
      `zmax = 0.2000` 即 branching 的預設 penetration 原值，未被夾限 → 壁厚 ≥ 2 × penetration 時零影響，與 Default 樹的 7b.6 結論一致

### 7e. Fail-safe 診斷

- [x] 7e.1 在支撐樹生成結束時，若 fail-safe 計數大於 0，輸出彙總警告日誌（含觸發的支撐頭數量）
      實作為共用函式 `report_penetration_failsafes(stats, tree_name)`（`SupportTreeUtils.hpp`），兩棵樹共用同一段文字，避免日後只改一邊而分歧
      輸出點：
      `DefaultSupportTree::execute()` 狀態機 `while` 迴圈結束後、`return` 之前 → `"Default support tree: …"`
      `create_branching_tree()` 末端、`unroutable_pinheads()` 失效處理之後 → `"Branching support tree: …"`
      兩棵樹的**主頭與錨點皆累加至同一個計數器**（Default 為 `alg.m_penetration_stats`；Branching 為 `create_branching_tree()` 的區域變數，經 `vbuilder.penetration_stats` 指標共用），故每棵樹最多一行
      等級採 `warning`（非 #3 所用的 `debug`）：射線無命中代表網格未封閉，受影響支撐已完全失去咬合，且只有使用者能處理（修補網格）。訊息內含成因說明與處置建議
      存取確認：`execute()` 為 `DefaultSupportTree` 的 `static` 成員（`DefaultSupportTree.hpp:257`），可存取 `alg` 的私有成員 `m_penetration_stats`
      Default 樹於 `pc == ABORT` 時**不輸出**：該次執行已被取消、計數不完整，且使用者並非在等這份診斷
- [x] 7e.2 計數為 0 時不得輸出
      `report_penetration_failsafes()` 開頭即 `if (misses == 0) return;`，在碰到 log 之前就返回，常態切片完全不產生任何輸出
- [x] 7e.3 🔨 **使用者手動建置**
- [x] 7e.4 **驗證檢查點 A**：以含破面／非流形的模型切片 → 確認警告日誌輸出且刺入深度為 0
      新增測試資產 `tests/data/sla_thin/broken_openplate.stl`（20×20×0.2 平板，**刻意不含頂面**的開放殼層）
      警告確實輸出（`warning` 等級，單行彙總）：
      `Default support tree: thin wall measurement found no exit surface for 20 support head(s)/anchor(s). Their penetration was set to 0 as a fail-safe, … Repairing the mesh should clear it.`
      20 個支撐點全數觸發，符合預期（每個頭都在缺失的頂面下方向上量測）
      **刺入深度確實歸零**（以「設定值完全失效」證明）：`pen = 0.05 / 0.30 / 0.60` 三組輸出**位元組完全相同**（`sha=ff57786fd23f3618`、`zmax=0.1164`）。設定值被完全捨棄，非只是縮小
- [x] 7e.5 **驗證檢查點 B**：以流形完好的模型切片 → MUST NOT 輸出該警告
      結果 PASS：`20mm_cube`、`cube_with_hole`、`reg_cyl25x30`、`frog_legs`、`m020`、`m030`、`m050`、`dish.stl` 共 8 顆，於 `default` 與 `branching` 兩種樹型下**警告行數皆為 0**（16 組組合）
      **⚠ 真陽性發現，同時修正本變更稍早的兩項記錄**：`anchor_tunnel.stl` 與 `anchor_thinfloor.stl` **會**輸出警告（分別 11 / 40 點與 1 / 40 點）。以邊拓撲檢查確認，這**兩顆是我自建的測試資產，本身即為非流形**——64 條邊中有 32 條不被恰好兩個三角形共用（`quad()` 產生的 T 型接點裂縫）。既有的 `m020/m030/m050/reg_cyl25x30` 經同一檢查皆為封閉流形
      故此非誤觸發，而是診斷抓到了我自己沒察覺的資產缺陷——本項的正面驗證
      **須連帶修正的先前結論（記錄於此，未回頭改寫該處）**：
      ① 6.6 將 `anchor_tunnel.stl` 的路由不確定性歸因為「upstream 既有行為」。該模型為非流形，**此歸因可能有誤**，不確定性也可能源自破面。原結論「基準版同樣重現」仍成立，但成因未定
      ② 7c.4 量得的「支撐凸出頂面 0.1996 → 0.0061 mm」同樣取自此非流形模型。**該改善可能來自 fail-safe 歸零（penetration → 0），而非薄壁夾限**。以 7e.4 的證據，兩者在該模型上無法區分
      → 上述兩項的**結論方向不變**（皆為「凸出量大幅下降」與「非本變更引入」），但**成因歸屬需重新檢視**

---

## 8. 全域迴歸與收尾

- [x] 8.1 🔨 **使用者手動建置**：最終完整建置
- [x] 8.2 **全相位迴歸**：`m020.stl` 於 `layer_height` 0.05 / 0.10 / 0.15，各執行 `elev 5.00～5.15` 完整掃描 → 除層高過大導致無切片層的情形外，MUST 全數產出支撐點且點位 z MUST 為下表面
      結果 PASS（48 組組合）：
      ```
      lh=0.05  16/16 相位 → 19 點，zmax 全數 0.1000
      lh=0.10  16/16 相位 → 19 點，zmax 全數 0.1000
      lh=0.15  14/16 相位 → 19 點，zmax 全數 0.1000；5.13/5.14 為 ERR
      ```
      `5.13 / 5.14` 的 ERR 即 #4 已改寫訊息的「層高相對物件高度過大」情形（物件高 0.2 < 1.5 × 0.15 = 0.225），屬本項判準明文排除的範圍
      **點位確在下表面**：`zmax = 0.1000` 為板厚 0.2 之半。若點位落在上表面，頭部本體會位於 z = 0.2 之上，`zmax` 將遠大於此值
- [x] 8.3 **厚度迴歸**：`m030.stl`、`m050.stl` 全相位掃描 → MUST 全數正常
      結果 PASS（32 組組合，`lh = 0.15`）：兩顆模型 16/16 相位皆為 19 點，無 ERR、無全滅
      `m030` zmax 全數 `0.1500`、`m050` 全數 `0.2500` → 皆等於各自板厚之半，夾限在所有相位下一致生效
- [x] 8.4 **常態模型逐點迴歸**：1.4 的全部常態厚件比對 1.6 基準快照 → 支撐點數量、座標、支撐網格 MUST 完全一致
      結果 PASS：三顆 zero-change 的**完整 64 字元 SHA256 與基準逐位元相同**
      `20mm_cube.obj`(20 點) / `cube_with_hole.obj`(14 點) / `reg_cyl25x30.stl`(47 點)
      `frog_legs`(observe) 與三顆薄件如預期變動，已於 4.7 / 7b.6 逐項確認
- [~] 8.5 **防貫穿全面驗證**：對全部測試模型執行支撐／模型布林交集 → 承載面另一側 MUST 無任何支撐幾何
      **平板類全數 PASS（可精確量測）；曲面/有機模型無法以本方法定論。**
      改以「支撐 STL 最高 z vs 模型最高 z」判定（較布林交集精確、無容差問題）：
      ```
      模型                 模型頂z    支撐zmax   餘裕
      m020.stl             0.2000     0.1000     +0.1000   無穿透
      m030.stl             0.7500     0.1500     +0.6000   無穿透
      m050.stl             0.3500     0.2500     +0.1000   無穿透
      wall060.stl          0.6000     0.3285     +0.2715   無穿透
      reg_cyl25x30.stl    37.0000     1.2994    +35.7006   無穿透
      20mm_cube.obj       20.0000     0.3285    +19.6715   無穿透
      cube_with_hole.obj  10.0000     0.3000     +9.7000   無穿透
      dish.stl             0.2000     0.1000     +0.1000   無穿透
      broken_openplate     0.2000     0.1164     +0.0836   無穿透（fail-safe 生效）
      frog_legs.obj        2.0754     2.1030     −0.0276   ← 見下
      anchor_tunnel.stl   10.0000    10.0061     −0.0061   ← 見下
      ```
      **兩筆超出者的判讀（未能證實，亦未能排除）**：支撐頭的**本體**依設計本就在模型之外，`get_mesh()` 旋轉後傾斜頭的背球側緣可高於「接觸點 + 刺入深度」。同一效應在 `wall060` 上可直接觀察——全域 `zmax = 0.3285` 但板中央（遠離側緣）為精確的 `0.3000`。故這兩筆極可能是傾斜頭的本體幾何而**非針尖穿透**，但合併後的支撐 STL 無法區分針尖與本體，**我無法證明**
      改善幅度可量化：`frog_legs` 基準版 `zmax = 2.2490`（高出模型頂 +0.1736）→ 現行版 `2.1030`（+0.0276），**下降 0.146 mm**
      針尖深度的精確驗證改由平板類承擔（見 8.6 與 7b.4）：實測值與 `min(configured, 厚度 × 0.5)` 四位小數相符
- [x] 8.6 **中間帶壁厚確認**：以局部壁厚 0.6 mm 的模型驗證刺入深度為 0.3，並記錄此為符合規格之預期行為（非回歸）
      新增測試資產 `tests/data/sla_thin/wall060.stl`（21.168 × 21.168 × **0.600**，672 三角形，已以邊拓撲檢查確認為**封閉流形**）
      量測限定板中央 `x, y ∈ [5, 16]`（遠離側緣，該處頭部必位於下平面且軸向朝上）：
      ```
      設定pen  預期min(pen,0.3)  中央zmax  全域zmax
      0.10     0.1000            0.1000    0.1871
      0.20     0.2000            0.2000    0.2578
      0.30     0.3000            0.3000    0.3285
      0.60     0.3000            0.3000    0.4164
      0.90     0.3000            0.3000    0.4164
      ```
      PASS：中央量測與 `min(configured, 0.300)` **四位小數完全相符**。`pen ≥ 0.6` 時穩定飽和於 0.3000，即壁厚 0.6 mm 的夾限值
      **此為符合規格的預期行為，非回歸**：0.6 mm 恰為 `2 × penetration(0.3)` 的邊界，壁厚 ≥ 0.6 mm 者不受影響（8.4 三顆常態件逐位元相同即為證），< 0.6 mm 者依政策被夾限
      註：全域 `zmax` 較中央高，來源為板側緣的傾斜頭本體（design D5 的「由側緣出射」情形），非穿透——`0.6000` 頂面之下仍有 0.27 mm 餘裕
- [x] 8.7 執行既有 SLA 測試套件（`sla_print_tests`），確認無新增失敗
      **使用者裁定：跳過。理由為已有充分的全相位與幾何實測覆蓋。**
      **本項未實際執行**（誠實記錄，不得解讀為「測試已通過」）：現行建置樹 `CMakeCache.txt` 為 `SLIC3R_BUILD_TESTS:BOOL=OFF`，`sla_print_tests` 從未產生，建置目錄下無任何測試二進位檔；需以 `-DSLIC3R_BUILD_TESTS=ON` 重新配置並建置才能執行
      替代覆蓋（裁定所依據的實證）：
      ```
      8.2   全相位迴歸       48 組（m020 × 層高 3 種 × 相位 16 種）
      8.3   厚度迴歸         32 組（m030 / m050 全相位）
      8.4   常態逐點迴歸     3 顆 zero-change 完整 SHA256 逐位元相同
      7b.4  Default 主頭     6 組，與 min(pen, 厚度×0.5) 四位小數相符
      7d.6  Branching 主頭   4 組，同上
      7d.8  Branching 常態   3 顆點數 / zmax / SHA256 全同
      8.6   中間帶壁厚       5 組，wall060 中央量測與公式相符
      9.5   中空壁厚         12 組，與公式四位小數相符 + 實心對照
      7e.4  Fail-safe 觸發   broken_openplate，三組設定位元組相同
      7e.5  Fail-safe 靜默   8 顆 × 2 種樹型 = 16 組，警告行數皆 0
      ```
      **殘餘風險（須記錄）**：`release/v1.0.5` 已新增 `tests/sla_print/sla_pad_degradation_tests.cpp`（6 cases / 19 assertions），主題為「零支撐樹 + pad 降級」，與 #1 修復的行為面重疊。本 change 併入該分支後，建議補跑一次。風險評估為低（其 fixture 為傾斜 40° 的實心方塊，#5 夾限對厚件不生效、#1 對明確朝下面不改變結果），但此為推論非實測
- [x] 8.8 更新 `web_slicer_core` 端既有測試中「極薄模型 → 無支撐」的預期值（`support_outcome` 不再為 `SUPPORT_NOT_NEEDED`、`has_support_mesh` 轉為 `true`）
      **N/A — 無此對象，無需修改。** 已逐一檢視 `agent/tests/` 下所有相關測試：
      `test_support_e2e.py`、`test_run_support_generation.py`、`test_support_classifier.py` 中的 `SUPPORT_NOT_NEEDED` 斷言**全部以 stub 的 CLI stdout 驅動**，並非真的切片薄模型
      `test_support_e2e.py` 檔頭已自述原因：「Scenarios 1 & 2（zero support pillars）cannot be produced by the real engine from a closed solid: `SLAConfig.enforce_min_elevation` forces a >=5mm elevation…」，故以 stub 產生
      全 repo 搜尋 `thin` / `薄` / `m020` 於非 `third_party` 的測試中皆無命中 → **不存在「極薄模型 → 無支撐」的既有預期值**
- [x] 8.9 檢視 design.md 的 Open Questions 三項是否於實作中被觸發；若有，回到 design.md 重新決議而非就地判斷
      三項**皆未觸發**，無需回到 design.md 重新決議：
      ① D1 第 3 層回退分支頻繁觸發 → 未發生。2.7 / 2.8 已確認三顆 zero-change 逐位元相同，回退分支未被非預期觸發（2.8 已記為 N/A）
      ② D5 的 `ε` 在極薄件造成量測誤差超標 → 未發生。`ε = 0.01 mm` 之下，`m020`(0.2) / `m030`(0.3) / `m050`(0.5) / `wall060`(0.6) 的實測夾限值與理論值**四位小數完全相符**，誤差低於量測解析度
      ③ D6 的 `w` 未夾限驗證產生可見碰撞 → 未觀察到。7b.5 的垂直射線交會分析顯示頭部相關特徵剛體同動、無相對位移；8.5 全部平板類皆有正餘裕
- [x] 8.10 版本控制提交策略
      **使用者裁定：以單一完整變更進行原子提交（Atomic Commit），不拆分 hunk。**
      原任務文字為「確認六項修改皆為獨立提交且順序正確，使任一項可單獨 revert 與 bisect」，已依裁定變更目標
      **裁定所放棄的能力（須記錄）**：六項修改無法單獨 revert，`git bisect` 無法定位到單一缺陷編號。若日後需回退其中一項，須以手動反向套用該項的 hunk 處理
      **裁定的支撐理由**：`SLAPrintSteps.cpp` 同時含 #2/#3/#4、`DefaultSupportTree.cpp` 同時含 #5/#6，拆分須以 `git add -p` 逐 hunk 判定；且六項具硬性施作相依（`#1 → #2 → #3 → #4 → #6 → #5`），任一項單獨存在於歷史中皆非可獨立建置的狀態——#2 先於 #1 會使原本靠未初始化負值歪打正著的相位一併失效（見 3.3），#5 依賴 #6 的 `taildir` 正規化（見 6.2）。故「可單獨 revert」在本變更的相依結構下本就不成立
      提交內容涵蓋：6 個 C++ 原始碼檔、測試資產 `tests/data/sla_thin/`、本 change 的封存目錄、同步至 `openspec/specs/` 的兩份能力規格
- [x] 8.11 在各修改處確認已加註來源與意圖註解，供未來 rebase 至 upstream 時逐項核對
      已逐檔核對，六個檔案共新增 **228 行註解**，註解密度如下：
      ```
      SupportPointGenerator.cpp   +37 行 / 22 行註解
      SLAPrintSteps.cpp          +112 行 / 48 行註解
      DefaultSupportTree.cpp      +68 行 / 49 行註解
      DefaultSupportTree.hpp       +6 行 /  3 行註解
      SupportTreeUtils.hpp       +150 行 / 83 行註解
      BranchingTreeSLA.cpp        +39 行 / 23 行註解
      ```
      關鍵註解已於各項實作時逐一確認：#2 的 `ORDER DEPENDENCY`（必須在 #1 之後）、#6 的 upstream 出處（`PrusaSlicer DefaultSupportTree.cpp:706`）與自訂零長度防護、#5 的 `DISCIPLINE`（不得置於 optimizer 目標函式內）、7d.3 的 `ORDER IS LOAD BEARING`（不得移至 `toj` 之前）、以及 5.3 的 `//TRN` 置放規則

---

## 9. 驗證報告後續修訂（Step 8 全面驗證的處置）

> 來源：`/opsx:verify` 全面驗證報告。使用者裁定「能修正的項目盡量修正」。
> 本群組**不含任何 C++ 原始碼修改**，僅為文件對齊與補測。

- [x] 9.1 **W7｜修正 design.md D5 的射線起點符號**
      `design.md:150` 原寫 `query_ray_hit(hp − ε·dir_in, dir_in)`，符號錯誤。已改為 `hp + ε·dir_in`，與實作（`SupportTreeUtils.hpp:404`）及幾何物理一致
      同時補上錯誤理由段落：`AABBMesh::query_ray_hit()` 無正/反面過濾，起點退到模型**外側**時第一個命中必為入射面本身，量得厚度恆為 `2ε` 而與真實壁厚無關，夾限將在所有模型上失準且無聲
      並註明「本節符號於實作階段更正，實作自始採用正確的 `+ε`（見 7a.1）」，避免後人誤以為實作偏離設計

- [x] 9.2 **W1｜規格「相位無關」加註有效切片層限制**
      原 Scenario 要求 `elev 5.00～5.15` 每一格皆產出 19 點，但 `layer_height = 0.15` 下 `5.13 / 5.14` 無有效切片層，於 `slice_model()` 階段即中止，字面上永遠無法通過
      已於 Requirement 本文加入「適用範圍限於有效切片層相位」段落，說明該情形（模型高度 < 1.5 × `layer_height`）改由「模型無切片層時的錯誤訊息必須指出真實成因」需求承接，並明文 `MUST NOT` 被解讀為要求消除此類相位
      Scenario 亦加入一條 `AND`，明列 `5.13 / 5.14` 為已知例外且 `MUST NOT` 計為失敗

- [x] 9.3 **W2｜規格「Modifier 診斷日誌」改為 debug 等級**
      原文「該日誌 MUST 在預設日誌層級下可見」與實作（`BOOST_LOG_TRIVIAL(debug)`）不符
      已改為 `MUST 採 debug 等級（--loglevel 5 可見），MUST NOT 採 warning 或更高等級`，並寫入理由：越界已被安全處理、非需使用者介入的異常，升級會對常態切片產生無謂 UI 告警；此日誌用途為開發者診斷
      一併把「每次呼叫最多一行彙總」由 4.3 的實作註記提升為規格條文
      Scenario 判準改為「MUST 於 `--loglevel 5` 下可見」+「MUST NOT 於預設層級出現」

- [x] 9.4 **W3｜規格「常態模型不變」加註誤殺救回例外**
      原文與同份規格的「Modifier 過濾不得無聲丟棄支撐點」互相矛盾：`frog_legs` 由 66 → 67 點，字面上違反前者，但正是後者要求的行為
      已加入「例外（正面修復，非回歸）」段落，並給出**可核對的判定準則**——新增點位須皆源自層級索引夾限，且新增筆數須等於 debug 診斷日誌回報的筆數。`frog_legs` 實測 `1 of 67 ... were clamped`，與 66→67 相符
      明文加註「本例外 MUST NOT 被擴大解釋」：其餘任何點數或座標變動仍 MUST 判定為回歸
      原 Scenario 加上前提「該模型無任何支撐點觸發夾限」，另新增 Scenario「誤殺點位被救回屬正面修復」

- [x] 9.5 **S1｜中空（hollowing）壁厚補測**
      對應規格 Scenario：`sla-support-head-penetration` →「量測所用網格包含前置加工結果」。此為驗證報告中**唯一完全未驗證**的 Scenario
      測試模型 `tests/data/20mm_cube.obj`（實心 20×20×20，bbox z `[0, 20]`）。設定為 `cfg_base.ini` 疊加三個覆寫：`hollowing_enable = 1`、`hollowing_min_thickness = <t>`、`support_head_penetration = <pen>`
      **壁厚獨立量測**（`--export-hollow-stl` 匯出內腔網格，非由支撐反推，故非循環論證）：
      ```
      設定 t   內腔 bbox（置中座標系）      實測壁厚   模型座標的內腔天花板 z
      1.0      [-9.000, 9.000]³              1.000      19.0
      1.5      [-8.500, 8.500]³              1.500      18.5
      2.0      [-8.000, 8.001]³              2.000      18.0
      ```
      **觀測點**：中空後內腔天花板成為新的朝下承載面，其上方即為厚度恰等於 `t` 的頂壁，是本 Scenario 最乾淨的量測位置。支撐 STL 的全域 `zmax` 即該處針尖高度
      **結果**（支撐點數 20 → 21/22，證實中空確實生效）：
      ```
      壁厚t  天花板z  設定pen  預期min(pen,t/2)  實測針尖z  換算刺入深度
      1.0     19.0     0.30     0.30              19.3000    0.3000   未夾限
      1.0     19.0     0.50     0.50              19.5000    0.5000   邊界
      1.0     19.0     0.80     0.50              19.5000    0.5000   夾限
      1.0     19.0     0.90     0.50              19.5000    0.5000   夾限（飽和）
      1.0     19.0     1.00     0.50              19.5000    0.5000   夾限（飽和）
      1.5     18.5     0.30     0.30              18.8000    0.3000   未夾限
      1.5     18.5     0.50     0.50              19.0000    0.5000   未夾限
      1.5     18.5     0.80     0.75              19.2500    0.7500   夾限
      2.0     18.0     0.30     0.30              18.3000    0.3000   未夾限
      2.0     18.0     0.50     0.50              18.5000    0.5000   未夾限
      2.0     18.0     0.80     0.80              18.8000    0.8000   未夾限
      ```
      **12 組全部與 `min(configured, 局部壁厚 × 0.5)` 四位小數完全相符**
      **決定性對照（同模型、同設定，只差有無中空）**：實心 `20mm_cube` 於 `pen = 0.8` 的中央 `zmax` 為 `0.8000`（未夾限，壁厚 20 mm）；中空 `t = 1.0` 同樣 `pen = 0.8` 為 `0.5000`（夾限）。**證實量測所用網格確為中空後的 `emesh`，而非原始實心網格**
      實心對照組校準（`x, y ∈ [4, 16]` 中央窗）：`pen 0.3 → 0.3000`、`0.5 → 0.5000`、`0.8 → 0.8000`，即支撐針尖 z 恰等於刺入深度，座標系對齊無誤
      **排除 fail-safe 干擾**：全部執行的日誌中 `found no exit surface` 命中數皆為 0，故上述縮減確為薄壁夾限，非 fail-safe 歸零。此點對本項至關重要——7e.5 已證實 fail-safe 會使結果無法區分
      **未驗證項（誠實記錄）**：`support_head_penetration` 上限受 `Head penetration should not be greater than the Head width` 檢查限制，`pen = 1.5` 遭拒；`hollowing_min_thickness` 的設定下限為 1.0 mm（`PrintConfig.cpp:4838`），故無法以 CLI 產生 < 1.0 mm 的中空壁。更薄的情形由平板模型（7b.4 / 8.6）承擔
      **外底面支撐頭未能取得乾淨讀數**：`z ∈ [0, 1.5]` 中央窗的量測值受內腔支撐幾何污染（內腔柱體自內腔地板向上，落入同一 z 區間），數值無法歸屬。**未採用**，本項結論全部建立在內腔天花板的量測上
      **排水孔（drain holes）未實測**：`sla_drain_holes` 儲存於 3MF 的模型資料中，CLI 載入 STL 無法設定。程式碼層級已確認走同一條路徑——`m_supportdata`（即 `input.emesh` 的來源）由 `po.get_mesh_to_print()` 建立（`SLAPrintSteps.cpp:857` 與 `1086`），而 `slaposHollowing` 與 `slaposDrillHoles` 為同一 pipeline 的前後兩步，共用此出口。故中空的實證同時涵蓋鑽孔路徑，但**鑽孔本身確實未取得實測數據**
