#!/usr/bin/env python3
"""
X650 仿真模型自检（离线，不需要 PX4 / Gazebo）

把 BOM 的台架数据与 model.sdf / airframe 里的参数做交叉验证，确认：
  1. 模型总质量 = BOM 起飞重量 3.5767 kg
  2. 单电机最大推力 = BOM 的 1920 g
  3. 推重比 = BOM 的 2.147
  4. 悬停油门 ≈ BOM 的 53%
  5. 推力曲线在 50%–100% 油门区间与台架数据的偏差
  6. EC_MIN/EC_MAX 与 maxRotVelocity 一致（不一致会导致推力曲线整体缩放错）
  7. 惯量为正定、Izz > Ixx

用法：  ./scripts/05_check_model_math.py [-v]
退出码：0 = 全部通过；1 = 有项目超差
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MODELS = ROOT / "gz_extras" / "models"
AIRFRAMES = ROOT / "gz_extras" / "airframes"

G = 9.81

# ---- BOM 实测/台架数据（「重量与续航评估」表）-------------------------------
BOM_TAKEOFF_MASS_KG = 3.5767
BOM_HOVER_THROTTLE = 0.5301  # 53.008 %
BOM_THRUST_G = [  # 油门 -> 单电机拉力 (g)
    (0.50, 830),
    (0.65, 1150),
    (0.75, 1430),
    (0.85, 1690),
    (1.00, 1920),
]
TOL_MASS = 0.01       # kg
TOL_THRUST = 0.02     # 相对
TOL_HOVER = 0.03      # 绝对（油门）

FAILURES: list[str] = []


def fail(msg: str) -> None:
    FAILURES.append(msg)


def note(cond: bool, msg: str) -> None:
    print(f"  {'✔' if cond else '✘'} {msg}")
    if not cond:
        fail(msg)


@dataclass
class Model:
    base_mass: float
    rotor_masses: list[float]
    inertia: tuple[float, float, float]
    max_rot_velocity: float
    motor_constant: float
    ec_min: float
    ec_max: float

    @property
    def total_mass(self) -> float:
        return self.base_mass + sum(self.rotor_masses)

    def omega(self, throttle: float) -> float:
        """PX4 归一化油门 -> 电机角速度（affine 映射）。"""
        return self.ec_min + throttle * (self.ec_max - self.ec_min)

    def thrust(self, throttle: float) -> float:
        """单电机推力 (N)。"""
        return self.motor_constant * self.omega(throttle) ** 2

    def hover_throttle(self) -> float:
        """使 4·T = m·g 的油门。"""
        need = self.total_mass * G / 4.0
        omega = math.sqrt(need / self.motor_constant)
        return (omega - self.ec_min) / (self.ec_max - self.ec_min)


def validate_assets() -> None:
    """XML 可解析 + airframe 脚本语法正确 + model.config 与 model.sdf 名称一致。"""
    xml_files = sorted(list((ROOT / "gz_extras").rglob("*.sdf"))
                       + list((ROOT / "gz_extras").rglob("*.config")))
    for f in xml_files:
        rel = f.relative_to(ROOT)
        try:
            tree = ET.parse(f)
        except ET.ParseError as e:
            fail(f"{rel}: XML 解析失败 — {e}")
            continue
        if f.suffix != ".config":
            continue
        cfg = tree.getroot()
        sdf = f.parent / (cfg.findtext("sdf") or "model.sdf")
        if not sdf.is_file():
            fail(f"{rel}: 引用的 {sdf.name} 不存在")
            continue
        sdf_root = ET.parse(sdf).getroot()
        model = sdf_root.find("./model")
        name_in_sdf = model.get("name") if model is not None else None
        if name_in_sdf != cfg.findtext("name"):
            fail(f"{rel}: name={cfg.findtext('name')} 与 {sdf.name} 中 "
                 f"model name={name_in_sdf} 不一致")
    for f in sorted(AIRFRAMES.glob("*_gz_x650*")):
        rel = f.relative_to(ROOT)
        r = subprocess.run(["sh", "-n", str(f)], capture_output=True, text=True)
        if r.returncode != 0:
            fail(f"{rel}: shell 语法错误 — {r.stderr.strip()}")
        if not f.stat().st_mode & 0o111:
            fail(f"{rel}: 缺少可执行权限（chmod +x）")
    print(f"  解析 {len(xml_files)} 个 XML 文件、"
          f"{len(list(AIRFRAMES.glob('*_gz_x650*')))} 个 airframe 脚本")


def _load(path: Path) -> ET.Element:
    return ET.parse(path).getroot().find("./model")


def validate_references() -> None:
    """include 目标存在、插件引用的 joint/link 在合并后的模型里确实存在。"""
    base = _load(MODELS / "x650_base" / "model.sdf")
    base_links = {l.get("name") for l in base.findall("link")}
    base_joints = {j.get("name") for j in base.findall("joint")}

    top = _load(MODELS / "x650" / "model.sdf")
    for inc in top.findall("include"):
        uri = (inc.findtext("uri") or "").replace("model://", "")
        if not (MODELS / uri / "model.sdf").is_file():
            fail(f"x650/model.sdf 的 include 指向不存在的 model://{uri}")
        elif uri != "x650_base":
            fail(f"x650/model.sdf 只应 include x650_base，实际 {uri}")

    motors = [p for p in top.findall("plugin")
              if (p.get("name") or "").endswith("MulticopterMotorModel")]
    seen_numbers = set()
    for p in motors:
        num = p.findtext("motorNumber")
        if num is None or num in seen_numbers:
            fail(f"x650: motorNumber 缺失或重复（{num}）")
        seen_numbers.add(num)
        for tag, pool, what in (("jointName", base_joints, "joint"),
                                ("linkName", base_links, "link")):
            ref = p.findtext(tag)
            if ref not in pool:
                fail(f"x650: 插件引用的 {what} '{ref}' 在 x650_base 中不存在")
        if p.findtext("turningDirection") not in ("cw", "ccw"):
            fail(f"x650: motorNumber={num} 的 turningDirection 非法")

    depth = _load(MODELS / "x650_depth" / "model.sdf")
    inc_uris = {(i.findtext("uri") or "").replace("model://", "")
                for i in depth.findall("include")}
    for uri in inc_uris:
        if not (MODELS / uri / "model.sdf").is_file():
            fail(f"x650_depth/model.sdf 的 include 指向不存在的 model://{uri}")
    cam = _load(MODELS / "gemini_335l" / "model.sdf")
    cam_links = {l.get("name") for l in cam.findall("link")}
    for j in depth.findall("joint"):
        child, parent = j.findtext("child"), j.findtext("parent")
        if child not in cam_links:
            fail(f"x650_depth: 相机关节的 child '{child}' 不存在于 gemini_335l")
        if parent not in base_links:
            fail(f"x650_depth: 相机关节的 parent '{parent}' 不存在于 x650")
    print(f"  引用检查：{len(motors)} 个电机插件、"
          f"{len(depth.findall('joint'))} 个相机关节")


def load_model() -> Model:
    base = ET.parse(MODELS / "x650_base" / "model.sdf").getroot()
    base_link = base.find("./model/link[@name='base_link']")
    if base_link is None:
        raise SystemExit("x650_base/model.sdf 中找不到 base_link")
    base_mass = float(base_link.find("inertial/mass").text)
    inertia = tuple(
        float(base_link.find(f"inertial/inertia/{k}").text)
        for k in ("ixx", "iyy", "izz")
    )
    rotor_masses = [
        float(link.find("inertial/mass").text)
        for link in base.findall("./model/link")
        if link.get("name", "").startswith("rotor_")
    ]

    top = ET.parse(MODELS / "x650" / "model.sdf").getroot()
    plugins = top.findall("./model/plugin")
    motors = [
        p for p in plugins
        if p.get("name", "").endswith("MulticopterMotorModel")
    ]
    if len(motors) != 4:
        raise SystemExit(f"x650/model.sdf 应有 4 个电机插件，实际 {len(motors)}")
    max_rot = {float(m.findtext("maxRotVelocity")) for m in motors}
    k = {float(m.findtext("motorConstant")) for m in motors}
    if len(max_rot) != 1 or len(k) != 1:
        raise SystemExit(f"4 个电机参数不一致：maxRotVelocity={max_rot} motorConstant={k}")

    frames = sorted(AIRFRAMES.glob("*_gz_x650"))
    if not frames:
        raise SystemExit("找不到 airframe 文件（gz_extras/airframes/*_gz_x650）")
    text = frames[0].read_text()
    mins, maxs = set(), set()
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "param" and parts[1] == "set-default":
            if parts[2].startswith("SIM_GZ_EC_MIN"):
                mins.add(float(parts[3]))
            elif parts[2].startswith("SIM_GZ_EC_MAX"):
                maxs.add(float(parts[3]))
    if len(mins) != 1 or len(maxs) != 1:
        raise SystemExit(f"airframe 中 EC_MIN/EC_MAX 必须各有一个统一值：{mins} {maxs}")

    return Model(
        base_mass=base_mass,
        rotor_masses=rotor_masses,
        inertia=inertia,          # type: ignore[arg-type]
        max_rot_velocity=max_rot.pop(),
        motor_constant=k.pop(),
        ec_min=mins.pop(),
        ec_max=maxs.pop(),
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.parse_args()

    print("=" * 74)
    print("X650 模型自检")
    print("=" * 74)
    print("[0] 资源文件")
    validate_assets()
    validate_references()
    print()

    m = load_model()
    max_thrust = m.thrust(1.0)
    twr = 4 * max_thrust / (m.total_mass * G)
    hover_u = m.hover_throttle()

    print(f"  机体质量 {m.base_mass:.4f} kg + {len(m.rotor_masses)} × "
          f"{m.rotor_masses[0]:.4f} kg 旋翼 = {m.total_mass:.4f} kg")
    print(f"  电机：ω_max={m.max_rot_velocity:.1f} rad/s  k={m.motor_constant:.6g}  "
          f"EC_MIN={m.ec_min:.0f}  EC_MAX={m.ec_max:.0f}")
    print(f"  惯量 Ixx=Iyy={m.inertia[0]:.4f}  Izz={m.inertia[2]:.4f} kg·m²")
    print()

    print("[1] 质量")
    note(abs(m.total_mass - BOM_TAKEOFF_MASS_KG) <= TOL_MASS,
         f"仿真起飞重量 {m.total_mass:.4f} kg vs BOM {BOM_TAKEOFF_MASS_KG} kg "
         f"(容差 ±{TOL_MASS} kg)")

    print("[2] 推力映射一致性")
    note(abs(m.ec_max - m.max_rot_velocity) < 1e-6,
         f"EC_MAX={m.ec_max:.1f} 与 maxRotVelocity={m.max_rot_velocity:.1f} 一致")
    note(m.ec_min < m.ec_max, f"EC_MIN={m.ec_min:.0f} < EC_MAX={m.ec_max:.0f}")

    print("[3] 最大推力 / 推重比")
    bom_max = BOM_THRUST_G[-1][1] / 1000 * G
    note(abs(max_thrust - bom_max) / bom_max <= TOL_THRUST,
         f"单电机满油门 {max_thrust:.3f} N ({max_thrust / G * 1000:.0f} g) vs "
         f"BOM 1920 g ({bom_max:.3f} N)")
    note(abs(twr - 2.147) < 0.02, f"推重比 {twr:.3f} vs BOM 2.147")

    print("[4] 悬停点")
    note(abs(hover_u - BOM_HOVER_THROTTLE) <= TOL_HOVER,
         f"悬停油门 {hover_u * 100:.2f}% vs BOM {BOM_HOVER_THROTTLE * 100:.1f}% "
         f"(容差 ±{TOL_HOVER * 100:.0f}%)")

    print("[5] 推力曲线 vs 台架数据（油门 u → 单电机 g）")
    worst = 0.0
    for u, g_bom in BOM_THRUST_G:
        g_sim = m.thrust(u) / G * 1000
        err = (g_sim - g_bom) / g_bom * 100
        worst = max(worst, abs(err))
        flag = "✔" if abs(err) <= 10 else "✘"
        print(f"  {flag} u={u * 100:5.0f}%  仿真 {g_sim:7.1f} g   台架 {g_bom:5.0f} g   "
              f"偏差 {err:+6.1f}%")
    note(worst <= 10.0, f"全曲线最大偏差 {worst:.1f}%（口径：≤10%，仅作趋势参考）")

    print("[6] 惯量合理性")
    ixx, iyy, izz = m.inertia
    note(ixx > 0 and iyy > 0 and izz > 0, "惯量均为正")
    note(abs(ixx - iyy) < 0.02 * ixx, f"Ixx≈Iyy（{ixx:.4f} / {iyy:.4f}）")
    note(izz > ixx, f"Izz > Ixx（{izz:.4f} > {ixx:.4f}），符合四旋翼布局")

    print("=" * 74)
    if FAILURES:
        print(f"结果：{len(FAILURES)} 项不通过")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("结果：全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
