#!/usr/bin/env python3
"""22 · ArUco 视觉检测节点：相机图像 → 落点板世界位姿

真实视觉链路：
  相机图像 → cv2.aruco 检测 4 个标记 → 每个标记 solvePnP 求相机系位姿
          → 用相机世界位姿合成 → 求板中心世界位置 → 发布 + 统计延迟

发布：
  /ship/pad_pose_vis   (PoseStamped, 世界 ENU)  —— 板中心
  /ship/pad_twist_vis  (TwistStamped, ENU)      —— 差分的板速度
  /aruco/stats         (String, JSON)           —— 检测数/耗时/延迟

说明：
  · 相机外参由 camrig 模型的真实位姿给出（真机上相当于飞控的姿态估计）
  · 内参由水平 FOV 与图像尺寸推算（无畸变，仿真相机是理想针孔）
  · 时间戳用图像到达时刻，便于上层做延迟补偿

用法：
  python3 scripts/22_aruco_detector.py --world x650_sea \
      --image-topic /world/x650_sea/model/camrig/link/link/sensor/downcam/image
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.image_pb2 as image_pb2
import gz.msgs10.pose_pb2 as pose_single_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.transport13 as gz_transport

# 标记在板上（船体系）的布局，与 20_make_aruco_sdf.py 一致
SPACING = 0.30
MARKER_SIZE = 0.22
PAD_LOCAL = (0.30, 0.0)          # 板中心（船体系 xy）
# OpenCV 相机系(x右,y下,z前) → gz 相机系(x前,y左,z上)
CV_TO_GZ = np.array([[0.0, 0.0, 1.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0]])
MARKER_OFFSETS = {
    0: (-SPACING / 2, -SPACING / 2),
    1: (+SPACING / 2, -SPACING / 2),
    2: (-SPACING / 2, +SPACING / 2),
    3: (+SPACING / 2, +SPACING / 2),
}


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ArucoDetector(Node):
    def __init__(self, args) -> None:
        super().__init__("aruco_detector")
        self.args = args
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params = params

        self.cam_pose = None          # (p, R) 相机世界位姿
        self.last_pub = None
        self.history = []             # (t, pos) 用于差分速度

        self.pub_pose = self.create_publisher(PoseStamped, "/ship/pad_pose_vis", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "/ship/pad_twist_vis", 10)
        self.pub_stats = self.create_publisher(String, "/aruco/stats", 10)

        self.gz_node = gz_transport.Node()
        world = args.world
        # 单独的 Node 负责服务请求：在订阅回调里用同一个 Node 发 request 会失效
        self.pose_client = gz_transport.Node()
        self.gz_node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self.on_gz_pose)
        self.gz_node.subscribe(image_pb2.Image, args.image_topic, self.on_image)
        # 自己把跟随相机贴到无人机上（省掉一个独立脚本，减少故障点）
        self.set_pose_pub = self.gz_node.advertise(f"/world/{world}/set_pose", pose_single_pb2.Pose)
        self.drone_pose = None
        self._last_follow = 0.0
        self._last_detect = 0.0

        self.create_timer(2.0, self.report)
        self.n_frames = 0
        self.n_detect = 0
        self.get_logger().info(f"ArUco 检测：{args.image_topic}")

    # ---------- 输入 ----------
    def on_gz_pose(self, msg) -> None:
        for p in msg.pose:
            if p.name == self.args.drone:
                self.drone_pose = p
                # 相机安装在机体上：机体系偏移 (0,0,-0.05)，绕 y 转 +90° 朝下
                R_body = quat_to_matrix((p.orientation.x, p.orientation.y,
                                         p.orientation.z, p.orientation.w))
                cam_local = np.array([0.0, 0.0, -0.05])
                R_cam_local = np.array([[0.0, 0.0, 1.0],
                                        [0.0, 1.0, 0.0],
                                        [-1.0, 0.0, 0.0]])
                self.cam_pose = (np.array([p.position.x, p.position.y, p.position.z])
                                 + R_body @ cam_local, R_body @ R_cam_local)
            elif self.args.camrig and p.name == self.args.camrig:
                self.cam_pose = (
                    np.array([p.position.x, p.position.y, p.position.z]),
                    quat_to_matrix((p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w)),
                )

    def follow_drone(self) -> None:
        if self.drone_pose is None or not self.args.follow:
            return
        now = time.time()
        if now - self._last_follow < 0.05:      # 最多 20 Hz，避免把 set_pose 服务打满
            return
        self._last_follow = now
        p = pose_single_pb2.Pose()
        p.name = self.args.camrig
        p.position.x = self.drone_pose.position.x
        p.position.y = self.drone_pose.position.y
        p.position.z = self.drone_pose.position.z - 0.10
        p.orientation.y, p.orientation.w = 0.7071068, 0.7071068
        self.pose_client.request(f"/world/{self.args.world}/set_pose", p,
                                 pose_single_pb2.Pose, boolean_pb2.Boolean, 100)

    def on_image(self, msg) -> None:
        t0 = time.time()
        self.follow_drone()
        # 图像可能以数百 Hz 到达（update_rate 未必生效），检测限速到 30 Hz
        if t0 - self._last_detect < 1.0 / 30.0:
            return
        self._last_detect = t0
        self.n_frames += 1
        if self.cam_pose is None:
            return
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        except ValueError:
            return
        gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return
        self.n_detect += 1

        # 内参：由水平 FOV 推算（理想针孔）
        h, w = gray.shape
        fx = (w / 2.0) / math.tan(self.args.hfov / 2.0)
        K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]], dtype=float)
        half = MARKER_SIZE / 2.0
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                       dtype=np.float32)

        p_c, R_c = self.cam_pose
        world_pts = []
        for i, mid in enumerate(ids.ravel()):
            ok, rvec, tvec = cv2.solvePnP(obj, corners[i].reshape(-1, 2), K, None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            R_m, _ = cv2.Rodrigues(rvec)
            # 标记中心在相机系
            c_cam = tvec.reshape(3)
            # 板中心 = 标记中心 + R_marker * (-offset)（把标记偏移反推到板中心）
            ox, oy = MARKER_OFFSETS.get(int(mid), (0.0, 0.0))
            c_cam = c_cam + R_m @ np.array([-ox, oy, 0.0])   # 图像 y 向下，故 y 取反
            world_pts.append(p_c + R_c @ (CV_TO_GZ @ c_cam))

        if not world_pts:
            return
        pad = np.mean(np.array(world_pts), axis=0)

        # 速度：差分（只保留最近 1 s）
        now = time.time()
        self.history.append((now, pad))
        self.history = [(tt, pp) for tt, pp in self.history if now - tt <= 1.0]
        vel = np.zeros(3)
        if len(self.history) >= 2:
            dt = self.history[-1][0] - self.history[0][0]
            if dt > 1e-3:
                vel = (self.history[-1][1] - self.history[0][1]) / dt

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in pad)
        ps.pose.orientation.w = 1.0
        self.pub_pose.publish(ps)

        ts = TwistStamped()
        ts.header = ps.header
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = (float(v) for v in vel)
        self.pub_twist.publish(ts)

        st = String()
        st.data = json.dumps({
            "n_markers": int(len(world_pts)),
            "proc_ms": round((time.time() - t0) * 1000, 1),
            "pad": [round(float(v), 3) for v in pad],
        })
        self.pub_stats.publish(st)

    def report(self) -> None:
        if self.n_frames:
            self.get_logger().info(
                f"ArUco: {self.n_frames} 帧, {self.n_detect} 帧检出 "
                f"({100.0*self.n_detect/self.n_frames:.0f}%)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="x650_sea")
    ap.add_argument("--camrig", default="")      # 空=直接用机体位姿（相机装在机体上）
    ap.add_argument("--drone", default="x650_0")
    ap.add_argument("--follow", type=int, default=1)
    ap.add_argument("--hfov", type=float, default=1.4)
    ap.add_argument("--image-topic",
                    default="/world/x650_sea/model/x650_0/link/base_link/sensor/downcam/image")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ArucoDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
