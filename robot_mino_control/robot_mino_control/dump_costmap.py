#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CostmapDump(Node):
    def __init__(self):
        super().__init__('costmap_dump')
        self.sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.cb, 1)

    def cb(self, msg):
        w, h = msg.info.width, msg.info.height
        data = msg.data
        chars = " .,:-=+*#%@"  # 0(kosong) -> 100(lethal)
        print(f"Grid {w}x{h}, resolusi {msg.info.resolution:.3f} m/sel\n")
        for row in range(h - 1, -1, -1):  # baris atas = y terbesar
            line = ""
            for col in range(w):
                v = data[row * w + col]
                if v < 0:
                    line += "?"
                else:
                    idx = min(int(v / 100 * (len(chars) - 1)), len(chars) - 1)
                    line += chars[idx]
            print(line)
        lethal_count = sum(1 for v in data if v >= 99)
        print(f"\nJumlah sel lethal (>=99): {lethal_count} / {len(data)}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CostmapDump()
    rclpy.spin(node)

if __name__ == '__main__':
    main()