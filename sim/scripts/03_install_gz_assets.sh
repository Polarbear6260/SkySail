#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 03 · 把 X650 模型 / 世界 / airframe 装进 PX4 源码树
#
#   ./scripts/03_install_gz_assets.sh          # 安装（覆盖同名文件）
#   ./scripts/03_install_gz_assets.sh --dry    # 只打印将要做什么
#
# 关键点：SITL airframe 必须在 ROMFS/.../init.d-posix/airframes/CMakeLists.txt
# 的 px4_add_romfs_files(...) 列表里登记，否则 ROMFS 里没有这个 airframe，
# `make px4_sitl gz_x650` 会找不到目标。上游把 22000–22999 预留给自定义机型，
# 本套用 22000 / 22001。
#
# 装完必须重新编译（ROMFS 已嵌入 airframe 列表）：
#   ./scripts/02_build_sitl.sh
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DRY=0
[ "${1:-}" = "--dry" ] && DRY=1

GZ_DIR="$PX4_DIR/Tools/simulation/gz"
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
AF_LIST=("22000_gz_x650" "22001_gz_x650_depth")
MODELS=(x650_base x650 x650_depth gemini_335l)

[ -d "$GZ_DIR/models" ] || { echo "找不到 $GZ_DIR/models，先跑 01_fetch_sources.sh" >&2; exit 1; }
[ -f "$AF_DIR/CMakeLists.txt" ] || { echo "找不到 $AF_DIR/CMakeLists.txt，PX4 源码不完整" >&2; exit 1; }

run() { if [ "$DRY" -eq 1 ]; then echo "  [dry] $*"; else eval "$@"; fi; }

echo "== 模型 → $GZ_DIR/models =="
for m in "${MODELS[@]}"; do
  src="$ROOT/gz_extras/models/$m"
  [ -d "$src" ] || { echo "缺少 $src" >&2; exit 1; }
  run "rm -rf '$GZ_DIR/models/$m'"
  run "cp -r '$src' '$GZ_DIR/models/$m'"
  echo "  ✔ $m"
done

echo "== 世界 → $GZ_DIR/worlds =="
run "cp -f '$ROOT/gz_extras/worlds/x650_obstacles.sdf' '$GZ_DIR/worlds/'"
echo "  ✔ x650_obstacles.sdf"

echo "== airframe → $AF_DIR =="
for f in "$ROOT"/gz_extras/airframes/*_gz_x650*; do
  base="$(basename "$f")"
  run "cp -f '$f' '$AF_DIR/$base'"
  run "chmod 755 '$AF_DIR/$base'"
  echo "  ✔ $base"
done

echo "== 登记到 CMakeLists.txt =="
if [ "$DRY" -eq 1 ]; then
  echo "  [dry] 在 px4_add_romfs_files(...) 末尾追加 ${AF_LIST[*]}"
else
  python3 - "$AF_DIR/CMakeLists.txt" "${AF_LIST[@]}" <<'PY'
import sys
path, names = sys.argv[1], sys.argv[2:]
lines = open(path).read().splitlines()
missing = [n for n in names if not any(n in ln for ln in lines)]
if not missing:
    print("  ✔ 已登记，跳过")
else:
    idx = max(i for i, ln in enumerate(lines) if ln.strip() == ")")
    lines[idx:idx] = [f"\t{n}" for n in missing]
    open(path, "w").write("\n".join(lines) + "\n")
    print("  ✔ 新登记：" + "、".join(missing))
PY
fi

if [ "$DRY" -eq 0 ]; then
  echo
  echo "== 校验 =="
  python3 "$ROOT/scripts/05_check_model_math.py" >/dev/null && echo "  ✔ 模型自检通过"
  for m in "${MODELS[@]}"; do
    [ -f "$GZ_DIR/models/$m/model.sdf" ] && echo "  ✔ model://$m 就位"
  done
  [ -f "$GZ_DIR/worlds/x650_obstacles.sdf" ] && echo "  ✔ 世界 x650_obstacles 就位"
  for a in "${AF_LIST[@]}"; do
    grep -q "$a" "$AF_DIR/CMakeLists.txt" && echo "  ✔ airframe $a 已登记"
  done
  echo
  echo "下一步：./scripts/02_build_sitl.sh   （airframe 变更必须重编）"
  echo "然后：  ./scripts/04_run_sitl.sh --smoke"
fi
