#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

joy_pub = None

def joyCallback(joy_msg):
    global joy_pub
    # Create a TwistStamped message
    stamped_msg = TwistStamped()
    
    # Set timestamp and frame ID
    stamped_msg.header.stamp = rclpy.clock.Clock().now().to_msg()  # Current time
    # stamped_msg.header.frame_id = "base_footprint"  # Modify frame_id if needed
    
    # Copy Twist data into TwistStamped
    stamped_msg.twist = joy_msg
    
    # Publish the TwistStamped message
    joy_pub.publish(stamped_msg)


def main(args=None):
    global joy_pub
    rclpy.init(args=args)
    node = Node('joy_republisher_node')
    time.sleep(1)
    
    # Publisher for TwistStamped
    joy_pub = node.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
    
    # Subscriber for Twist messages
    joy_sub = node.create_subscription(Twist, "/cmd_vel", joyCallback, 10)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
