#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from collections import Counter

class CounterNode(Node):
    def __init__(self):
        super().__init__('cluster_counter')
        self.sub = self.create_subscription(PointCloud2, '/pointcloud/clustered', self.cb, 10)
        self.get_logger().info('cluster_counter started')

    def cb(self, msg: PointCloud2):
        cnt = Counter()
        total=0
        for p in pc2.read_points(msg, field_names=['cluster_id'], skip_nans=False):
            lbl = p[0]
            cnt[lbl]+=1
            total+=1
        self.get_logger().info(f'clusters count total={total} unique={len(cnt)} sample={list(cnt.items())[:10]}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
