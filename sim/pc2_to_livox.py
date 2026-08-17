#!/usr/bin/env python3
"""Gazebo 3D 激光 PointCloud2 → Livox CustomMsg 桥 (给 FAST-LIO 喂数据)。

sub  /lidar3d/points  (sensor_msgs/PointCloud2, x/y/z/intensity)
pub  /livox/lidar_front (livox_interfaces/CustomMsg)

每点填:offset_time(按方位角推算, ns) / x,y,z / reflectivity(=intensity) / tag=0 / line=0。
FAST-LIO2 用原始点(feature_en=false), line 不敏感; offset_time 用于去畸变(低速影响小)。
numpy 向量化读点; CustomPoint 列表构造是瓶颈, 已把点数降到 ~180×16。

    python3 pc2_to_livox.py            # 默认 topic
"""
import argparse, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from livox_interfaces.msg import CustomMsg, CustomPoint


class Bridge(Node):
    def __init__(self, in_topic, out_topic, period_ns):
        super().__init__("pc2_to_livox")
        self.period_ns = period_ns
        self.pub = self.create_publisher(CustomMsg, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, qos_profile_sensor_data)
        self.n = 0
        self.get_logger().info(f"bridge {in_topic} (PointCloud2) -> {out_topic} (CustomMsg)")

    def cb(self, msg: PointCloud2):
        # read x,y,z,intensity as numpy (point_step 16 = 4 float32)
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        xyz = arr[:, 0:3]
        inten = arr[:, 3] if arr.shape[1] > 3 else np.zeros(len(arr), np.float32)
        # drop NaN/inf and zero points
        good = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
        xyz = xyz[good]; inten = inten[good]
        if len(xyz) == 0:
            return
        az = np.arctan2(xyz[:, 1], xyz[:, 0])                    # [-pi,pi]
        off = ((az + math.pi) / (2 * math.pi) * self.period_ns).astype(np.uint32)  # ns within one sweep
        refl = np.clip(inten, 0, 255).astype(np.uint8)

        out = CustomMsg()
        out.header = msg.header
        out.timebase = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        out.point_num = len(xyz)
        out.lidar_id = 0
        pts = []
        xf = xyz[:, 0].tolist(); yf = xyz[:, 1].tolist(); zf = xyz[:, 2].tolist()
        offl = off.tolist(); rfl = refl.tolist()
        for i in range(len(xf)):
            cp = CustomPoint()
            cp.offset_time = offl[i]
            cp.x = xf[i]; cp.y = yf[i]; cp.z = zf[i]
            cp.reflectivity = rfl[i]; cp.tag = 0; cp.line = 0
            pts.append(cp)
        out.points = pts
        self.pub.publish(out)
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-topic", default="/lidar3d/points")
    ap.add_argument("--out-topic", default="/livox/lidar_front")
    ap.add_argument("--rate-hz", type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = Bridge(args.in_topic, args.out_topic, int(1e9 / args.rate_hz))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
