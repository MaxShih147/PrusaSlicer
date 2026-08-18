#!/usr/bin/env bash
# 切片網格相位掃描 —— openspec/changes/fix-sla-thin-model-support-points
#
# 固定模型與層高，將 support_object_elevation 自 5.00 掃至 5.15（逐 0.01），
# 逐次輸出 "Automatic support points" 的數值。elevation 的小數部分決定切片
# 網格相位（minZ = bb.min(Z) - get_elevation()），故一個完整週期 = layer_height。
#
# 用法：
#   ./sweep_phase.sh <slicer-engine.exe> <model.stl> [layer_height] [elev_from] [elev_to] [step]
#
# 範例：
#   ./sweep_phase.sh ../../../../prusaslicer_build/src/Release/slicer-engine.exe m020.stl 0.15
#
# 輸出：每行一個 "elev=<值> pts=<數量|ERR>"，末尾附摘要。
#   pts=<N>  正常產出 N 個支撐點
#   pts=0    支撐點全滅
#   pts=ERR  切片拋出例外（通常是網格內無模型切片層）

set -u

BIN="${1:?用法: $0 <slicer-engine.exe> <model.stl> [layer_height] [from] [to] [step]}"
MODEL="${2:?缺少模型路徑}"
LH="${3:-0.15}"
FROM="${4:-5.00}"
TO="${5:-5.15}"
STEP="${6:-0.01}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG_BASE="$SCRIPT_DIR/cfg_base.ini"
[ -f "$CFG_BASE" ] || { echo "找不到 $CFG_BASE" >&2; exit 1; }
[ -x "$BIN" ] || [ -f "$BIN" ] || { echo "找不到切片器 $BIN" >&2; exit 1; }
[ -f "$MODEL" ] || { echo "找不到模型 $MODEL" >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "模型      : $MODEL"
echo "層高      : $LH"
echo "elevation : $FROM → $TO (step $STEP)"
echo "切片器    : $BIN"
echo "---------------------------------------------"

ok=0; zero=0; err=0
results=""

for ev in $(awk -v a="$FROM" -v b="$TO" -v s="$STEP" 'BEGIN{for(x=a;x<=b+1e-9;x+=s) printf "%.2f\n", x}'); do
    sed -e "s/^layer_height = .*/layer_height = $LH/" \
        -e "s/^initial_layer_height = .*/initial_layer_height = $LH/" \
        -e "s/^support_object_elevation = .*/support_object_elevation = $ev/" \
        "$CFG_BASE" > "$TMP/cfg.ini"

    n=$("$BIN" --loglevel 5 --export-sla --output "$TMP/out.sl1" --export-support-stl \
            --center 60,34 --load "$TMP/cfg.ini" "$MODEL" 2>&1 \
        | grep -oE "Automatic support points: [0-9]+" | grep -oE "[0-9]+$")

    if   [ -z "${n:-}" ]; then n="ERR"; err=$((err+1))
    elif [ "$n" = "0" ];  then           zero=$((zero+1))
    else                                 ok=$((ok+1)); fi

    printf "elev=%s  pts=%s\n" "$ev" "$n"
    results="$results $n"
    rm -f "$TMP/out.sl1" "$TMP/out_support.stl"
done

echo "---------------------------------------------"
echo "摘要: 正常 $ok / 全滅 $zero / 例外 $err"
echo "序列:$results"

# 全部相位皆產出支撐點才視為通過
if [ "$zero" -eq 0 ] && [ "$err" -eq 0 ]; then
    echo "結果: PASS（所有相位皆產出支撐點）"
    exit 0
else
    echo "結果: FAIL（存在支撐點全滅或切片例外的相位）"
    exit 1
fi
