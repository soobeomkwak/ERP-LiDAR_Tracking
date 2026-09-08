#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class RawHexInspector(Node):
    def __init__(self):
        super().__init__('raw_hex_inspector')
        self.sub = self.create_subscription(PointCloud2, '/pointcloud/clustered', self.cb, 10)
        self.get_logger().info('RawHexInspector started')

    def cb(self, msg: PointCloud2):
        self.get_logger().info(f'Received width={msg.width} point_step={msg.point_step} fields={[ (f.name,f.offset,f.datatype) for f in msg.fields ]}')
        data = bytes(msg.data)
        ps = msg.point_step
        cnt = min(20, msg.width)
        for i in range(cnt):
            off = i * ps
            try:
                x = struct.unpack_from('<f', data, off + 0)[0]
                y = struct.unpack_from('<f', data, off + 4)[0]
                z = struct.unpack_from('<f', data, off + 8)[0]
                cluster_int = struct.unpack_from('<i', data, off + 12)[0]
                rgb_f = struct.unpack_from('<f', data, off + 16)[0]
                rgb_i = struct.unpack_from('<I', data, off + 16)[0]
            except struct.error:
                self.get_logger().error('struct error at i=%d' % i)
                break
            self.get_logger().info(f'pt[{i}]: x={x:.6f} y={y:.6f} z={z:.6f} cluster_int={cluster_int} rgb_f={rgb_f:.6f} rgb_hex=0x{rgb_i:08X}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RawHexInspector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
