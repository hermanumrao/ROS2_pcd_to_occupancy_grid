#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2

import numpy as np
import os
import cv2

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QVBoxLayout,
    QPushButton,
    QLabel,
    QDoubleSpinBox,
)
from std_msgs.msg import Header


class MapGUI(Node):
    def __init__(self):
        super().__init__("map_gui")

        self.cloud_np = None
        self.original_cloud = None
        self.map2d = None
        self.frame_id = "map"

        self.sub_cloud = self.create_subscription(
            PointCloud2, "/saved_map", self.cloud_cb, 10
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map2d", self.map_cb, 10
        )

        self.pub = self.create_publisher(PointCloud2, "/aligned_map", 10)

        # Transform params
        self.tx = 0.0
        self.ty = 0.0
        self.yaw = 0.0
        self.tz = 0.0
        self.roll = 0.0
        self.pitch = 0.0

    # -------------------------------
    # Cloud callback (FIXED)
    # -------------------------------
    def cloud_cb(self, msg):
        self.frame_id = msg.header.frame_id

        points = point_cloud2.read_points_numpy(msg, skip_nans=True)

        if points.size == 0:
            return

        # Handle both structured and plain arrays
        if hasattr(points.dtype, "names") and points.dtype.names is not None:
            xyz = np.zeros((points.shape[0], 3), dtype=np.float32)
            xyz[:, 0] = points["x"]
            xyz[:, 1] = points["y"]
            xyz[:, 2] = points["z"]
        else:
            xyz = points[:, :3].astype(np.float32)

        # Store original only once
        if self.original_cloud is None:
            self.original_cloud = xyz.copy()
            self.get_logger().info("Original cloud stored")

        self.cloud_np = xyz

    def map_cb(self, msg):
        self.map2d = msg

    def transform_cloud(self):
        if self.original_cloud is None:
            return

        pts = self.original_cloud.copy()

        # Rotation matrices
        cr = np.cos(self.roll)
        sr = np.sin(self.roll)
        cp = np.cos(self.pitch)
        sp = np.sin(self.pitch)
        cy = np.cos(self.yaw)
        sy = np.sin(self.yaw)

        # Roll (X)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

        # Pitch (Y)
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

        # Yaw (Z)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        # Combined rotation (Z * Y * X)
        R = Rz @ Ry @ Rx

        pts = pts @ R.T

        # Translation
        pts[:, 0] += self.tx
        pts[:, 1] += self.ty
        pts[:, 2] += self.tz

        self.cloud_np = pts
        self.publish_cloud()

    # -------------------------------
    # Publish aligned cloud
    # -------------------------------
    def publish_cloud(self):
        if self.cloud_np is None:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        msg = point_cloud2.create_cloud_xyz32(header, self.cloud_np.tolist())

        self.pub.publish(msg)

    # -------------------------------
    # Save 3D map
    # -------------------------------
    def save_pcd(self):
        if self.original_cloud is None:
            return

        # Recompute transformed cloud (always fresh)
        pts = self.original_cloud.copy()

        cr = np.cos(self.roll)
        sr = np.sin(self.roll)
        cp = np.cos(self.pitch)
        sp = np.sin(self.pitch)
        cy = np.cos(self.yaw)
        sy = np.sin(self.yaw)

        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        R = Rz @ Ry @ Rx

        pts = pts @ R.T

        # Translation
        pts[:, 0] += self.tx
        pts[:, 1] += self.ty
        pts[:, 2] += self.tz

        # Save this transformed version
        os.makedirs(os.path.expanduser("~/maps"), exist_ok=True)
        path = os.path.expanduser("~/maps/aligned.pcd")

        with open(path, "w") as f:
            f.write("# .PCD v0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\n")
            f.write("DATA ascii\n")

            for p in pts:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")

        self.get_logger().info("Saved aligned.pcd (transformed)")

    # -------------------------------
    # Save 2D map
    # -------------------------------
    def save_pgm(self):
        if self.map2d is None:
            return

        width = self.map2d.info.width
        height = self.map2d.info.height

        data = np.array(self.map2d.data).reshape((height, width))

        img = np.zeros((height, width), dtype=np.uint8)

        img[data == 100] = 0
        img[data == 0] = 254
        img[data == -1] = 205

        os.makedirs(os.path.expanduser("~/maps"), exist_ok=True)
        cv2.imwrite(os.path.expanduser("~/maps/aligned2d.pgm"), img)

        self.get_logger().info("Saved aligned2d.pgm")


# ---------------- GUI ----------------


class App(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.node = ros_node

        layout = QVBoxLayout()

        # -------------------------------
        # X
        # -------------------------------
        layout.addWidget(QLabel("X"))
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-100.0, 100.0)
        self.spin_x.setSingleStep(0.1)
        self.spin_x.setDecimals(3)
        self.spin_x.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_x)

        # -------------------------------
        # Y
        # -------------------------------
        layout.addWidget(QLabel("Y"))
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-100.0, 100.0)
        self.spin_y.setSingleStep(0.1)
        self.spin_y.setDecimals(3)
        self.spin_y.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_y)

        # -------------------------------
        # Z
        # -------------------------------
        layout.addWidget(QLabel("Z"))
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-100.0, 100.0)
        self.spin_z.setSingleStep(0.1)
        self.spin_z.setDecimals(3)
        self.spin_z.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_z)

        # -------------------------------
        # Roll
        # -------------------------------
        layout.addWidget(QLabel("Roll (rad)"))
        self.spin_roll = QDoubleSpinBox()
        self.spin_roll.setRange(-3.14, 3.14)
        self.spin_roll.setSingleStep(0.01)
        self.spin_roll.setDecimals(4)
        self.spin_roll.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_roll)

        # -------------------------------
        # Pitch
        # -------------------------------
        layout.addWidget(QLabel("Pitch (rad)"))
        self.spin_pitch = QDoubleSpinBox()
        self.spin_pitch.setRange(-3.14, 3.14)
        self.spin_pitch.setSingleStep(0.01)
        self.spin_pitch.setDecimals(4)
        self.spin_pitch.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_pitch)

        # -------------------------------
        # Yaw
        # -------------------------------
        layout.addWidget(QLabel("Yaw (rad)"))
        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-3.14, 3.14)
        self.spin_yaw.setSingleStep(0.01)
        self.spin_yaw.setDecimals(4)
        self.spin_yaw.valueChanged.connect(self.update_values)
        layout.addWidget(self.spin_yaw)

        # -------------------------------
        # Apply Transform
        # -------------------------------
        btn_transform = QPushButton("Apply Transform")
        btn_transform.clicked.connect(self.node.transform_cloud)
        layout.addWidget(btn_transform)

        # -------------------------------
        # Reset Button
        # -------------------------------
        btn_reset = QPushButton("Reset")
        btn_reset.clicked.connect(self.reset_values)
        layout.addWidget(btn_reset)

        # -------------------------------
        # Save Button
        # -------------------------------
        btn_save = QPushButton("Save Maps")
        btn_save.clicked.connect(self.save_maps)
        layout.addWidget(btn_save)

        self.setLayout(layout)

    # -------------------------------
    # Update node params
    # -------------------------------
    def update_values(self):
        self.node.tx = self.spin_x.value()
        self.node.ty = self.spin_y.value()
        self.node.tz = self.spin_z.value()

        self.node.roll = self.spin_roll.value()
        self.node.pitch = self.spin_pitch.value()
        self.node.yaw = self.spin_yaw.value()

    # -------------------------------
    # Reset everything
    # -------------------------------
    def reset_values(self):
        self.spin_x.setValue(0.0)
        self.spin_y.setValue(0.0)
        self.spin_z.setValue(0.0)

        self.spin_roll.setValue(0.0)
        self.spin_pitch.setValue(0.0)
        self.spin_yaw.setValue(0.0)

        self.update_values()

        # Reset preview cloud too
        if self.node.original_cloud is not None:
            self.node.cloud_np = self.node.original_cloud.copy()
            self.node.publish_cloud()

    # -------------------------------
    # Save maps
    # -------------------------------
    def save_maps(self):
        self.node.save_pcd()
        self.node.save_pgm()


# ---------------- MAIN ----------------


def main():
    rclpy.init()
    node = MapGUI()

    app = QtWidgets.QApplication([])
    gui = App(node)
    gui.show()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        app.processEvents()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
