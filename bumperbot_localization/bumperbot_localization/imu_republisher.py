#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

imu_pub = None

# Adjust the QoS profile for compatibility
qos_profile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    depth=10
)

def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub.publish(imu)


def main(args=None):
    global imu_pub
    rclpy.init(args=args)
    node = Node('imu_republisher_node')
    time.sleep(1)
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub = node.create_subscription(Imu, "imu/out", imuCallback, qos_profile)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
