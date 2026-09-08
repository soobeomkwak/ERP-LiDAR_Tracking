#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class Inspector(Node):
    def __init__(self):
        super().__init__('pc_inspector')
        self.sub = self.create_subscription(PointCloud2, '/pointcloud/clustered', self.cb, 10)
        self.get_logger().info('Inspector started, waiting for one message...')
        self.received = False

    def cb(self, msg: PointCloud2):
        self.get_logger().info(f'Received PointCloud2: width={msg.width} height={msg.height} fields={[f.name for f in msg.fields]}')
        count = 0
        for p in pc2.read_points(msg, field_names=["x", "y", "z", "cluster_id"], skip_nans=False):
            print(p)
            count += 1
            if count >= 50:
                break
        self.received = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Inspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.received:
            node.get_logger().info('No message received')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
