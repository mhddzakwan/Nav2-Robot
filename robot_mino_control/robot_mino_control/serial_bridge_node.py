#!/usr/bin/env python3
"""
serial_bridge_node.py
Jembatan antara firmware Arduino robot_mino dan topic_based_ros2_control.

Tanggung jawab node ini:
  1. Subscribe /topic_based_joint_commands (sensor_msgs/JointState, velocity[])
     -> kirim "wL,wR\n" ke Arduino lewat serial.
  2. Baca telemetri Arduino (#ts,ticksL,ticksR,rpmL,rpmR,ax,ay,az,gx,gy,gz)
     -> publish /topic_based_joint_states (posisi rad + velocity rad/s)
     -> publish /imu/data_raw (terpisah dari ros2_control, karena
        topic_based_ros2_control TIDAK mendukung <sensor> tag)

PENTING: urutan nama joint di sini (left_wheel_joint, right_wheel_joint)
HARUS sama persis dengan urutan <joint> di URDF dan urutan
left_wheel_names/right_wheel_names di controllers.yaml.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu

try:
    import serial
except ImportError:
    serial = None


JOINT_NAMES = ['left_wheel_joint', 'right_wheel_joint']


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('counts_per_rev', 1012.0)
        self.declare_parameter('joint_commands_topic', '/topic_based_joint_commands')
        self.declare_parameter('joint_states_topic', '/topic_based_joint_states')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_frame_id', 'imu_link')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value
        cmd_topic = self.get_parameter('joint_commands_topic').value
        state_topic = self.get_parameter('joint_states_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        if serial is None:
            raise RuntimeError('pyserial belum terinstall. jalankan: pip install pyserial --break-system-packages')

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()

        self.create_subscription(JointState, cmd_topic, self.joint_command_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, state_topic, 10)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)

        # Posisi kumulatif (rad), diakumulasi dari delta ticks tiap paket
        self.pos_left = 0.0
        self.pos_right = 0.0
        self.prev_ticks_l = None
        self.prev_ticks_r = None

        # Baca serial di timer terpisah dari command callback
        self.create_timer(0.01, self.read_serial)  # polling 100Hz, paket Arduino datang ~50Hz

        self.get_logger().info(f'Serial bridge aktif: {port} @ {baud} | cmd={cmd_topic} state={state_topic}')

    def joint_command_callback(self, msg: JointState):
        """Terima command velocity dari diff_drive_controller via topic_based_ros2_control."""
        if len(msg.velocity) < 2:
            return

        # Cocokkan index berdasarkan nama joint, jangan asumsikan urutan
        cmd_map = dict(zip(msg.name, msg.velocity)) if msg.name else None
        if cmd_map and JOINT_NAMES[0] in cmd_map and JOINT_NAMES[1] in cmd_map:
            w_left = cmd_map[JOINT_NAMES[0]]
            w_right = cmd_map[JOINT_NAMES[1]]
        else:
            # Fallback kalau message tidak mengisi field 'name'
            w_left, w_right = msg.velocity[0], msg.velocity[1]

        line = f'{w_left:.4f},{w_right:.4f}\n'
        with self.lock:
            self.ser.write(line.encode('utf-8'))

    def read_serial(self):
        with self.lock:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if not raw.startswith('#') or raw == '#ROBOT_READY':
            return

        fields = raw[1:].split(',')
        if len(fields) != 11:
            return

        try:
            (_, ticks_l, ticks_r, rpm_l, rpm_r,
             ax, ay, az, gx, gy, gz) = fields
            ticks_l, ticks_r = int(ticks_l), int(ticks_r)
            rpm_l, rpm_r = float(rpm_l), float(rpm_r)
            ax, ay, az = float(ax), float(ay), float(az)
            gx, gy, gz = float(gx), float(gy), float(gz)
        except ValueError:
            return

        now = self.get_clock().now()
        self.publish_joint_states(ticks_l, ticks_r, rpm_l, rpm_r, now)
        self.publish_imu(ax, ay, az, gx, gy, gz, now)

    def publish_joint_states(self, ticks_l, ticks_r, rpm_l, rpm_r, stamp):
        if self.prev_ticks_l is None:
            self.prev_ticks_l, self.prev_ticks_r = ticks_l, ticks_r
            return

        delta_l = ticks_l - self.prev_ticks_l
        delta_r = ticks_r - self.prev_ticks_r
        self.prev_ticks_l, self.prev_ticks_r = ticks_l, ticks_r

        rad_per_tick = (2.0 * math.pi) / self.counts_per_rev
        self.pos_left += delta_l * rad_per_tick
        self.pos_right += delta_r * rad_per_tick

        RPM_TO_RADPS = (2.0 * math.pi) / 60.0
        vel_left = rpm_l * RPM_TO_RADPS
        vel_right = rpm_r * RPM_TO_RADPS

        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = JOINT_NAMES
        js.position = [self.pos_left, self.pos_right]
        js.velocity = [vel_left, vel_right]
        self.joint_state_pub.publish(js)

    def publish_imu(self, ax, ay, az, gx, gy, gz, stamp):
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.imu_frame_id
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.orientation_covariance[0] = -1.0  # tidak ada data orientasi absolut

        accel_var = 0.05
        gyro_var = 0.01
        imu.linear_acceleration_covariance[0] = accel_var
        imu.linear_acceleration_covariance[4] = accel_var
        imu.linear_acceleration_covariance[8] = accel_var
        imu.angular_velocity_covariance[0] = gyro_var
        imu.angular_velocity_covariance[4] = gyro_var
        imu.angular_velocity_covariance[8] = gyro_var

        self.imu_pub.publish(imu)

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
