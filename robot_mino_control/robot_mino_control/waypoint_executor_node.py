#!/usr/bin/env python3
"""
Waypoint Executor Node (Fase 2)

Cara kerja:
1. Terima goal ExecuteWaypoints berisi list nama lokasi, misal
   ["dapur", "ruang_keluarga"].
2. Validasi semua nama ke ~/robot_mino_locations.yaml (file yang sama
   dipakai location_saver_node di Fase 1). Kalau ada nama tidak dikenal,
   goal ditolak SEBELUM robot bergerak sama sekali (fail fast).
3. Convert tiap lokasi jadi geometry_msgs/PoseStamped (frame map),
   kirim sebagai satu goal ke action Nav2 "follow_waypoints"
   (nav2_msgs/action/FollowWaypoints).
4. Teruskan feedback dari Nav2 (index waypoint) ke pemanggil kita,
   tapi ditranslate balik jadi NAMA lokasi (lebih berguna utk Fase 5,
   misal buat TTS "menuju dapur...").
5. Kalau goal kita di-cancel, otomatis cancel juga goal Nav2 yang lagi
   jalan -> robot berhenti, bukan lanjut ke waypoint berikutnya.

Kenapa Action, bukan Service (beda dari Fase 1):
- Navigasi itu long-running & butuh feedback berkala + harus bisa
  dibatalkan di tengah jalan. Service itu blocking request-response,
  gak natural untuk kasus ini dan bakal jadi masalah pas Fase 5 butuh
  jalur "stop darurat" yang cepat.

Catatan arsitektur penting:
- Node ini jadi ActionServer SEKALIGUS ActionClient (relay ke Nav2).
  Supaya gak DEADLOCK saat action client menunggu balasan dari action
  server Nav2 (yang notabene proses/node lain, jadi sebenarnya aman),
  best practice tetap pakai ReentrantCallbackGroup + MultiThreadedExecutor,
  karena pola relay-action ini sering dipakai bersama timer/callback lain
  yang perlu tetap responsif (misal nanti butuh cek status tambahan).
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

from robot_mino_interfaces.action import ExecuteWaypoints


def yaw_to_quaternion(yaw: float):
    """Robot planar -> quaternion cuma butuh komponen z & w."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointExecutorNode(Node):
    def __init__(self):
        super().__init__('waypoint_executor_node')

        # Mendapatkan path absolut dari folder tempat file python ini berada,
        # naik satu level ke atas (..), lalu masuk ke folder 'location'
        default_path = os.path.expanduser('~/ros2_ws/src/robot_mino_control/location/robot_mino_locations.yaml')

        self.declare_parameter(
            'locations_file',
            os.path.normpath(default_path)
        )
        self.locations_file = self.get_parameter(
            'locations_file').get_parameter_value().string_value

        # Reentrant: action server ini bakal await action client internal,
        # butuh callback group yang boleh diproses concurrent.
        self._cb_group = ReentrantCallbackGroup()

        self._nav_goal_handle = None  # goal Nav2 yang lagi aktif, buat cancel

        self._action_server = ActionServer(
            self,
            ExecuteWaypoints,
            'execute_waypoints',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self._nav_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints',
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'Waypoint Executor siap. Action: /execute_waypoints'
        )

    # ------------------------------------------------------------------
    # Goal / cancel handling
    # ------------------------------------------------------------------
    def _goal_callback(self, goal_request):
        if not goal_request.location_names:
            self.get_logger().warn('Goal ditolak: location_names kosong.')
            return GoalResponse.REJECT
        if self._nav_goal_handle is not None:
            self.get_logger().warn(
                'Goal ditolak: masih ada waypoint execution yang berjalan.'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Menerima permintaan cancel.')
        if self._nav_goal_handle is not None:
            self.get_logger().warn(
                'Meneruskan cancel ke Nav2 follow_waypoints...'
            )
            # Fire-and-forget: request cancel langsung terkirim saat dipanggil,
            # gak perlu await future-nya di sini (cancel_callback itu sync,
            # bukan coroutine). Hasil akhirnya nanti muncul lewat
            # get_result_future yang di-await di execute_callback.
            self._nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Eksekusi utama
    # ------------------------------------------------------------------
    def _load_locations(self) -> dict:
        if not os.path.exists(self.locations_file):
            return {}
        with open(self.locations_file, 'r') as f:
            data = yaml.safe_load(f)
            return data if data else {}

    def _names_to_poses(self, names, locations: dict):
        """Return (poses, invalid_names). poses cuma dibuat kalau semua nama valid."""
        invalid = [n for n in names if n not in locations]
        if invalid:
            return None, invalid

        poses = []
        stamp = self.get_clock().now().to_msg()
        for name in names:
            entry = locations[name]
            pose = PoseStamped()
            pose.header.frame_id = entry.get('frame_id', 'map')
            pose.header.stamp = stamp
            pose.pose.position.x = float(entry['x'])
            pose.pose.position.y = float(entry['y'])
            pose.pose.position.z = 0.0
            qz, qw = yaw_to_quaternion(float(entry['yaw']))
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)
        return poses, []

    async def _execute_callback(self, goal_handle):
        names = list(goal_handle.request.location_names)
        result = ExecuteWaypoints.Result()

        locations = self._load_locations()
        poses, invalid_names = self._names_to_poses(names, locations)

        if invalid_names:
            msg = f"Lokasi tidak dikenal: {', '.join(invalid_names)}"
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            result.failed_location_names = invalid_names
            return result

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            msg = "Action server 'follow_waypoints' tidak tersedia."
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            result.failed_location_names = names
            return result

        nav_goal = FollowWaypoints.Goal()
        nav_goal.poses = poses

        self.get_logger().info(f'Mengirim {len(poses)} waypoint ke Nav2: {names}')

        send_goal_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda fb: self._on_nav_feedback(
                goal_handle, names, fb
            )
        )
        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:
            msg = 'Goal ditolak oleh Nav2 follow_waypoints.'
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            result.failed_location_names = names
            return result

        self._nav_goal_handle = nav_goal_handle

        # --- Tunggu hasil Nav2 langsung (bukan polling asyncio.sleep) ---
        # await ke rclpy Future terbukti kompatibel dgn cara rclpy men-drive
        # coroutine (lihat send_goal_future di atas). asyncio.sleep() TIDAK
        # kompatibel -> itu penyebab bug ABORTED sebelumnya.
        # try/finally: _nav_goal_handle GARANSI reset apapun hasilnya, supaya
        # goal berikutnya gak pernah nyangkut ditolak selamanya.
        try:
            get_result_future = nav_goal_handle.get_result_async()
            nav_result = await get_result_future
        finally:
            self._nav_goal_handle = None

        if goal_handle.is_cancel_requested:
            self.get_logger().warn('Goal dibatalkan oleh pengguna.')
            goal_handle.canceled()
            result.success = False
            result.message = 'Dibatalkan oleh pengguna.'
            result.failed_location_names = names
            return result

        missed_idx = list(nav_result.result.missed_waypoints)
        failed_names = [names[i] for i in missed_idx if 0 <= i < len(names)]

        if failed_names:
            goal_handle.succeed()  # request tetap "selesai diproses", cuma sebagian gagal
            result.success = False
            result.message = (
                f"Selesai dengan {len(failed_names)} lokasi gagal: "
                f"{', '.join(failed_names)}"
            )
            result.failed_location_names = failed_names
            self.get_logger().warn(result.message)
        else:
            goal_handle.succeed()
            result.success = True
            result.message = f"Berhasil mengunjungi semua lokasi: {', '.join(names)}"
            result.failed_location_names = []
            self.get_logger().info(result.message)

        return result

    def _on_nav_feedback(self, goal_handle, names, nav_feedback_msg):
        idx = nav_feedback_msg.feedback.current_waypoint
        if 0 <= idx < len(names):
            fb = ExecuteWaypoints.Feedback()
            fb.current_location = names[idx]
            fb.waypoint_index = idx
            fb.total_waypoints = len(names)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(
                f'Menuju waypoint {idx + 1}/{len(names)}: {names[idx]}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()