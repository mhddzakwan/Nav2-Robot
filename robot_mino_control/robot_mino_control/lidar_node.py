#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        # DISESUAIKAN: default diubah dari 'laser_link' menjadi 'lidar_link' agar pas dengan URDF
        self.declare_parameter('frame_id', 'lidar_link') 
        self.declare_parameter('angle_resolution_deg', 1.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_res = self.get_parameter('angle_resolution_deg').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.num_bins = int(360.0 / self.angle_res)
        self.ranges = [float('nan')] * self.num_bins
        self.last_fsa = 0.0

        # Proteksi jika port serial gagal dibuka
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Lidar node sukses terkoneksi di {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal membuka port serial {port}: {e}')
            raise e

        self.buffer = bytearray()
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer pembacaan data serial
        self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        while len(self.buffer) >= 10:
            if self.buffer[0] == 0xAA and self.buffer[1] == 0x55:
                lsn = self.buffer[3]
                packet_size = 10 + (lsn * 2)
                if len(self.buffer) < packet_size:
                    break
                packet = bytes(self.buffer[:packet_size])
                del self.buffer[:packet_size]

                if packet[2] == 0x00:
                    self.process_packet(packet)
            else:
                del self.buffer[0]

    def process_packet(self, packet):
        lsn = packet[3]
        if lsn == 0:
            return

        fsa = (packet[5] << 8 | packet[4]) >> 1
        lsa = (packet[7] << 8 | packet[6]) >> 1
        fsa_angle = fsa / 64.0
        lsa_angle = lsa / 64.0

        if lsa_angle >= fsa_angle:
            angle_diff = lsa_angle - fsa_angle
        else:
            angle_diff = (lsa_angle + 360.0) - fsa_angle

        if fsa_angle < self.last_fsa - 180.0:
            self.publish_scan()
        self.last_fsa = fsa_angle

        data_index = 10
        for i in range(lsn):
            if data_index + 1 >= len(packet):
                break

            distance_raw = packet[data_index + 1] << 8 | packet[data_index]
            distance_m = (distance_raw / 4.0) / 1000.0

            if distance_m > 0.0:
                angle = (angle_diff / (lsn - 1 if lsn > 1 else 1)) * i + fsa_angle
                angle = (360.0 - angle) % 360.0
                bin_idx = int(angle / self.angle_res) % self.num_bins
                self.ranges[bin_idx] = distance_m

            data_index += 2

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi - math.radians(self.angle_res)
        scan.angle_increment = math.radians(self.angle_res)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        clean = []
        for r in self.ranges:
            if math.isnan(r) or r < self.range_min or r > self.range_max:
                clean.append(float('inf'))
            else:
                clean.append(r)
        scan.ranges = clean

        self.scan_pub.publish(scan)
        self.ranges = [float('nan')] * self.num_bins

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node berhenti karena error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()