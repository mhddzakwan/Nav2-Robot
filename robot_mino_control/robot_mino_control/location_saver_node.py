#!/usr/bin/env python3
"""
Location Saver Node (Fase 1 - opsi A: drive-and-mark, versi service + TF)

Cara kerja:
1. Drive robot manual (teleop) ke lokasi fisik yang mau ditandai.
2. Panggil service /save_location dengan nama lokasi -> node lookup TF
   map -> base_link SAAT ITU JUGA (bukan cache dari topic), convert
   quaternion -> yaw, tulis/update ke file YAML.
3. Service /list_locations tersedia untuk query nama lokasi yang sudah
   tersimpan (dipakai nanti di Fase 4, buat system prompt LLM & validasi).

Kenapa TF, bukan subscribe /amcl_pose (versi sebelumnya):
- /amcl_pose HANYA dipublish AMCL saat filter update (robot bergerak
  melewati threshold update_min_d/update_min_a). Kalau robot diam saat
  command disave, pesan topic itu jadi "basi" walau localization-nya
  sendiri sebenarnya tetap valid.
- TF map->base_link selalu bisa di-lookup kapan saja: map->odom
  di-broadcast AMCL berkala, odom->base_link dari odometry kontinu.
  lookup_transform() otomatis kasih estimasi TERKINI, tanpa isu "basi".

Kenapa service (bukan topic seperti versi paling awal):
- Butuh feedback sinkron (success/message) langsung ke caller, penting
  nanti saat orchestrator LLM (Fase 5) perlu tahu instan hasil simpan
  lokasi untuk kasih respons TTS yang tepat.
"""

import math
import os
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from robot_mino_interfaces.srv import SaveLocation, ListLocations


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion (robot planar, hanya perlu yaw) ke radian."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class LocationSaverNode(Node):
    def __init__(self):
        super().__init__('location_saver_node')

        default_path = os.path.expanduser('~/ros2_ws/src/robot_mino_control/location/robot_mino_locations.yaml')
        
        self.declare_parameter(
            'locations_file',
            os.path.normpath(default_path)
        )
        self.declare_parameter('overwrite_existing', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 1.0)

        self.locations_file = self.get_parameter(
            'locations_file').get_parameter_value().string_value
        self.overwrite_existing = self.get_parameter(
            'overwrite_existing').get_parameter_value().bool_value
        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter(
            'robot_frame').get_parameter_value().string_value
        self.tf_timeout_sec = self.get_parameter(
            'tf_timeout_sec').get_parameter_value().double_value

        # --- TF buffer & listener: sumber pose "live", bukan cache topic ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- File lock sederhana, cegah korup kalau ada 2 write nyaris bersamaan ---
        self._file_lock = threading.Lock()

        # --- Service: simpan lokasi ---
        self.save_srv = self.create_service(
            SaveLocation,
            'save_location',
            self._save_location_callback
        )

        # --- Service: list lokasi tersimpan (buat Fase 4) ---
        self.list_srv = self.create_service(
            ListLocations,
            'list_locations',
            self._list_locations_callback
        )

        self.get_logger().info(
            f'Location Saver siap. File target: {self.locations_file}'
        )
        self.get_logger().info(
            f'Lookup TF: {self.map_frame} -> {self.robot_frame}'
        )
        self.get_logger().info(
            'Drive robot ke lokasi, lalu panggil: '
            "ros2 service call /save_location robot_mino_interfaces/srv/SaveLocation "
            '"{name: \'nama_lokasi\'}"'
        )

    def _save_location_callback(self, request, response):
        name = request.name.strip()

        if not name:
            response.success = False
            response.message = 'Nama lokasi tidak boleh kosong.'
            return response

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),  # Time() kosong = ambil transform TERBARU yg tersedia
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
        except TransformException as ex:
            response.success = False
            response.message = (
                f'Gagal lookup TF {self.map_frame}->{self.robot_frame}: {ex}. '
                'Pastikan AMCL & TF tree (map->odom->base_link) sudah aktif.'
            )
            self.get_logger().error(response.message)
            return response

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        with self._file_lock:
            data = self._load_locations()

            if name in data and not self.overwrite_existing:
                response.success = False
                response.message = (
                    f"Lokasi '{name}' sudah ada dan overwrite_existing=False."
                )
                self.get_logger().warn(response.message)
                return response

            data[name] = {
                'x': round(t.x, 4),
                'y': round(t.y, 4),
                'yaw': round(yaw, 4),
                'frame_id': self.map_frame,
            }
            self._save_locations(data)

        response.success = True
        response.message = (
            f"Tersimpan '{name}': x={data[name]['x']}, "
            f"y={data[name]['y']}, yaw={data[name]['yaw']} rad "
            f"(frame: {self.map_frame})"
        )
        self.get_logger().info(response.message)
        return response

    def _list_locations_callback(self, request, response):
        with self._file_lock:
            data = self._load_locations()
        response.names = sorted(data.keys())
        return response

    def _load_locations(self) -> dict:
        if os.path.exists(self.locations_file):
            with open(self.locations_file, 'r') as f:
                loaded = yaml.safe_load(f)
                return loaded if loaded else {}
        return {}

    def _save_locations(self, data: dict):
        target_dir = os.path.dirname(self.locations_file)
        if target_dir:
            os.makedirs(target_dir, exist_ok=True)

        # Tulis ke file sementara dulu lalu rename (atomic), cegah file YAML
        # korup/kepotong kalau proses mati di tengah proses tulis.
        tmp_path = self.locations_file + '.tmp'
        with open(tmp_path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
        os.replace(tmp_path, self.locations_file)


def main(args=None):
    rclpy.init(args=args)
    node = LocationSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()