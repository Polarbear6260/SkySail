#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 02 · 编译 px4_sitl（普通用户执行，不要 sudo）
#
#   ./scripts/02_build_sitl.sh
#
# 做法：
#   1. 在源码树内建独立 venv（Ubuntu 24.04 有 PEP 668，不能直接 pip 装到系统）
#   2. 装 PX4 的 Python 依赖（Tools/setup/requirements.txt）
#   3. make px4_sitl —— 只编译，不启动仿真
# ---------------------------------------------------------------------------
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="$PX4_DIR/.venv-px4"

[ -d "$PX4_DIR" ] || { echo "找不到 $PX4_DIR，先跑 01_fetch_sources.sh" >&2; exit 1; }

echo "== Python 依赖 =="
if [ ! -d "$VENV" ]; then
  python3 -m venv "$VENV"
fi
# shellcheck disable=SC1091
. "$VENV/bin/activate"
# PyPI 直连在这台机器上很慢，默认走清华源；可用 PIP_INDEX_URL 覆盖
PIP_INDEX="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
python -m pip install -q -i "$PIP_INDEX" --upgrade pip wheel setuptools

REQ="$PX4_DIR/Tools/setup/requirements.txt"
if [ -f "$REQ" ]; then
  echo "  pip install -r Tools/setup/requirements.txt"
  pip install -q -i "$PIP_INDEX" -r "$REQ"
else
  echo "  未找到 requirements.txt，安装最小依赖集"
  pip install -q -i "$PIP_INDEX" "empy==3.3.4" kconfiglib jinja2 numpy packaging pyros-genmsg toml jsonschema
fi

echo "== 版本 =="
cmake --version | head -1
ninja --version | sed 's/^/ninja /'
g++ --version | head -1

echo "== 编译 px4_sitl（首次约 3–8 分钟，20 核）=="
cd "$PX4_DIR"

# PX4 的 Tools/check_submodules.sh 在 CI=true 或检测到 VSCode/CLion 时会对每个
# 子模块执行 `git submodule update --recursive`，会把 NuttX / tflite / zenoh 等
# 300–600 MB 的东西全拉下来。这里显式跳过（SITL 编译不需要它们）。
unset VSCODE_PID CI CLION_IDE
export GIT_SUBMODULES_ARE_EVIL=1
if ! time make px4_sitl; then
  cat >&2 <<EOF

编译失败。若错误与缺失头文件/子模块有关（NuttX、mavlink 等），说明 01 用了精简模式，
补齐全部子模块后重跑本脚本：
    git -C "$PX4_DIR" submodule update --init --recursive --depth 1
EOF
  exit 1
fi

BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [ ! -x "$BIN" ]; then
  echo "错误：没有生成 $BIN" >&2
  exit 1
fi
echo
echo "✔ 编译完成：$BIN"
echo "  下一步：./scripts/03_install_gz_assets.sh"
