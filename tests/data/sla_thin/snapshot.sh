#!/usr/bin/env bash
# 支撐生成基準快照 / 比對 —— openspec/changes/fix-sla-thin-model-support-points 任務 1.6 與 8.4
#
# 對 regression_models.txt 列出的模型（以及三顆薄件）逐一切片，記錄：
#   支撐點數、支撐 STL 的 SHA256、支撐 STL 三角形數
# 支撐輸出已驗證具決定性（solver.seed(0)），故 SHA256 可作為「幾何逐點一致」的判準。
#
# 用法：
#   產生快照： ./snapshot.sh <slicer-engine.exe> [輸出 tsv]
#   比對快照： ./snapshot.sh <slicer-engine.exe> <輸出 tsv> --compare <基準 tsv>
#
# 注意：slicer-engine.exe 是 164KB 的薄啟動器，切片邏輯在同目錄的 slicer_core.dll。
#       比較 baseline 與修改後版本時，兩者 MUST 位於各自獨立的目錄，
#       否則 baseline 的 exe 會載入到新版 DLL，比對將靜默失效。

set -u

BIN="${1:?用法: $0 <slicer-engine.exe> [out.tsv] [--compare <base.tsv>]}"
OUT="${2:-baseline_snapshot.tsv}"
MODE="${3:-}"
BASE="${4:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
CFG="$SCRIPT_DIR/cfg_base.ini"
LIST="$SCRIPT_DIR/regression_models.txt"

[ -f "$BIN" ] || { echo "找不到切片器 $BIN" >&2; exit 1; }
[ -f "$CFG" ] || { echo "找不到 $CFG" >&2; exit 1; }
[ -f "$LIST" ] || { echo "找不到 $LIST" >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# 待測模型：regression_models.txt 的項目 + 三顆薄件（薄件於修復前部分相位為 0，屬預期）
MODELS=""
while IFS='|' read -r path cls pts desc; do
    case "$path" in ''|\#*) continue ;; esac
    MODELS="$MODELS $path|$cls"
done < "$LIST"
for m in m020 m030 m050; do
    MODELS="$MODELS tests/data/sla_thin/$m.stl|thin"
done

printf "model\tclass\tpoints\ttriangles\tsha256\n" > "$OUT"

for entry in $MODELS; do
    path="${entry%%|*}"; cls="${entry##*|}"
    full="$REPO_ROOT/$path"
    [ -f "$full" ] || { echo "跳過（找不到）: $path" >&2; continue; }

    log=$("$BIN" --loglevel 5 --export-sla --output "$TMP/o.sl1" --export-support-stl \
              --center 60,34 --load "$CFG" "$full" 2>&1)
    pts=$(printf '%s' "$log" | grep -oE "Automatic support points: [0-9]+" | grep -oE "[0-9]+$")
    pts="${pts:-ERR}"

    sup="$TMP/o_support.stl"
    if [ -f "$sup" ]; then
        sha=$(sha256sum "$sup" | cut -d' ' -f1)
        # Windows 原生 python 讀不到 Git Bash 的 POSIX 路徑，需轉為 Windows 路徑
        supw=$(cygpath -w "$sup" 2>/dev/null || printf '%s' "$sup")
        tri=$(SUPPATH="$supw" python -c "
import struct, os
f = open(os.environ['SUPPATH'], 'rb'); f.read(80)
print(struct.unpack('<I', f.read(4))[0])
" 2>/dev/null || echo "?")
    else
        sha="-"; tri="0"
    fi

    printf "%s\t%s\t%s\t%s\t%s\n" "$path" "$cls" "$pts" "$tri" "$sha" >> "$OUT"
    printf "%-45s %-12s pts=%-6s tri=%-8s %s\n" "$(basename "$path")" "$cls" "$pts" "$tri" "${sha:0:16}"
    rm -f "$TMP/o.sl1" "$sup"
done

echo "快照已寫入: $OUT"

if [ "$MODE" = "--compare" ]; then
    [ -f "$BASE" ] || { echo "找不到基準快照 $BASE" >&2; exit 1; }
    echo "---------------------------------------------"
    echo "與基準比對: $BASE"
    fail=0
    while IFS=$'\t' read -r path cls pts tri sha; do
        [ "$path" = "model" ] && continue
        # 以 awk 逐欄比對；grep -P 在非 UTF-8 locale 下不可用，且會讓比對靜默失效
        bline=$(awk -F'\t' -v k="$path" '$1==k{print; exit}' "$BASE")
        [ -z "$bline" ] && { echo "  新增       $path"; continue; }
        bpts=$(printf '%s' "$bline" | cut -f3); bsha=$(printf '%s' "$bline" | cut -f5)
        if [ "$cls" = "zero-change" ]; then
            if [ "$pts" = "$bpts" ] && [ "$sha" = "$bsha" ]; then
                echo "  PASS       $path (pts=$pts)"
            else
                echo "  ✗ FAIL     $path  pts $bpts→$pts  sha $( [ "$sha" = "$bsha" ] && echo same || echo CHANGED )"
                fail=1
            fi
        else
            [ "$pts" = "$bpts" ] && [ "$sha" = "$bsha" ] \
                && echo "  (unchanged) $path (pts=$pts)" \
                || echo "  (changed)   $path  pts $bpts→$pts  — $cls 類別，變化屬預期，需人工確認"
        fi
    done < "$OUT"
    echo "---------------------------------------------"
    [ "$fail" -eq 0 ] && { echo "結果: PASS（zero-change 類別全數逐點一致）"; exit 0; } \
                      || { echo "結果: FAIL（zero-change 類別出現差異）"; exit 1; }
fi
