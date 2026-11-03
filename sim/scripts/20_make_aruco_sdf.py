#!/usr/bin/env python3
"""20 · 生成落点板上的 ArUco 标记（几何体版本，不依赖贴图管线）

为什么用几何体而不是纹理：
  相机渲染在本机只能走 ogre1（ogre2 需要 EGL，此环境建不出上下文），
  而 ogre1 对 PBR 贴图支持有限、经典 .material 脚本又要额外资源路径，
  所以直接把 4×4 的 ArUco 图案用黑白小方块拼出来——渲染代价极低（每块一个平面），
  而且 OpenCV 检测到的就是真实图案。

用法：
  python3 scripts/20_make_aruco_sdf.py            # 重写 x650_sea.sdf 里的标记块
  python3 scripts/20_make_aruco_sdf.py --marker-size 0.12 --per-side 2

标记为 DICT_4X4_50 的 4 个 id，2×2 排布在落点板中心周围。
"""

from __future__ import annotations

import argparse
import re
import sys

# 4x4 字典中 id 0..3 的比特图案（1=黑块）。用 OpenCV 生成后固化，避免运行期依赖。
BITMAPS = {
    0: [[0, 1, 0, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 1, 0, 1]],
    1: [[1, 1, 1, 1], [0, 0, 0, 0], [0, 1, 1, 0], [0, 1, 0, 1]],
    2: [[1, 1, 0, 0], [1, 1, 0, 0], [1, 1, 0, 1], [0, 0, 1, 0]],
    3: [[0, 1, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [1, 0, 0, 1]],
}

BEGIN = "        <!-- ARUCO_BEGIN（由 scripts/20_make_aruco_sdf.py 生成） -->"
END = "        <!-- ARUCO_END -->"


def marker_block(mid: int, cx: float, cy: float, size: float, z: float,
                 white: str, black: str) -> list[str]:
    """生成单个 6×6 ArUco 标记：黑色底板（含 1 格边框）+ 白色数据格（bit=0）。"""
    cell = size / 6.0
    out = [f'        <visual name="aruco{mid}_plate">',
           f'          <pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>',
           f'          <geometry><plane><normal>0 0 1</normal><size>{size:.4f} {size:.4f}</size></plane></geometry>',
           f'          <material><ambient>{black}</ambient><diffuse>{black}</diffuse></material>',
           '        </visual>']
    bits = BITMAPS[mid % len(BITMAPS)]
    for r in range(4):
        for c in range(4):
            if bits[r][c]:          # 黑格与底板同色，不必画
                continue
            px = cx + (c - 1.5) * cell
            py = cy - (r - 1.5) * cell
            out += [f'        <visual name="aruco{mid}_w{r}{c}">',
                    f'          <pose>{px:.4f} {py:.4f} {z + 0.0006:.4f} 0 0 0</pose>',
                    f'          <geometry><plane><normal>0 0 1</normal><size>{cell:.4f} {cell:.4f}</size></plane></geometry>',
                    f'          <material><ambient>{white}</ambient><diffuse>{white}</diffuse></material>',
                    '        </visual>']
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="gz_extras/worlds/x650_sea.sdf")
    ap.add_argument("--marker-size", type=float, default=0.10)
    ap.add_argument("--spacing", type=float, default=0.17, help="标记中心间距 m")
    ap.add_argument("--pad-center", default="0.30,0.0", help="落点板中心（船体系）")
    ap.add_argument("--pad-z", type=float, default=0.472, help="板面高度（船体系）")
    args = ap.parse_args()

    cx0, cy0 = (float(v) for v in args.pad_center.split(","))
    blocks: list[str] = []
    # 2×2 排布，避开板中心（留给机体）
    positions = [(-1, -1), (+1, -1), (-1, +1), (+1, +1)]
    for mid, (sx, sy) in enumerate(positions):
        cx = cx0 + sx * args.spacing / 2.0
        cy = cy0 + sy * args.spacing / 2.0
        blocks += marker_block(mid, cx, cy, args.marker_size, args.pad_z,
                               "0.95 0.95 0.95 1", "0.05 0.05 0.05 1")

    text = "\n".join([BEGIN] + blocks + [END])
    src = open(args.world).read()
    if BEGIN in src and END in src:
        src = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END), text, src, flags=re.S)
    else:
        # 插到 pad_visual 之前
        anchor = '        <visual name="pad_visual">'
        src = src.replace(anchor, text + "\n\n" + anchor, 1)
    open(args.world, "w").write(src)
    print(f"已写入 {len(positions)} 个 ArUco 标记（边长 {args.marker_size} m）到 {args.world}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
