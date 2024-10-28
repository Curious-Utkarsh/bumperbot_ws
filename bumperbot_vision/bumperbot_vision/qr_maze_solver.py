import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import time


class CmaraLidarSub(Node):

    def __init__(self):
        super().__init__("qr_maze_solver_node")
        self.bridge = CvBridge()
        self.frame = None  # Initialize to None
        self.linear = 0.0
        self.angular = 0.0
        self.last_turn_data = "Drive Forward"
        self.flag = False
        self.error = 0.0
        self.velocity_pub = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.sub_camera = self.create_subscription(Image, "/camera/image_raw", self.cameraCallback, 10) 
        self.sub_lidar = self.create_subscription(LaserScan, "/scan", self.lidarCallback, 10)
        self.timer = self.create_timer(0.01, self.vel_pub)

    def cameraCallback(self, img):
        self.frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')

        if self.frame is not None:  # Ensure frame is not None
            # Ensure frame is in 8-bit unsigned format
            if self.frame.dtype != 'uint8':
                self.frame = cv2.convertScaleAbs(self.frame)

            # Initialize QR Code Detector
            decoder = cv2.QRCodeDetector()
            data, points, _ = decoder.detectAndDecode(self.frame)

            # Draw bounding box if QR code is detected
            if points is not None:
                points = points[0].astype(int)  # Convert points to integer type
                # Draw bounding box around the QR code
                for i in range(4):
                    start_point = tuple(points[i])
                    end_point = tuple(points[(i + 1) % 4])
                    cv2.line(self.frame, start_point, end_point, color=(0, 255, 0), thickness=2)

            # Update turn data on the frame
            if self.last_turn_data == "left":
                cv2.putText(self.frame, "Left Turn", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)
            elif self.last_turn_data == "right":
                cv2.putText(self.frame, "Right Turn", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)
            elif self.last_turn_data == "stop":
                cv2.putText(self.frame, "STOP", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 3, cv2.LINE_AA)
            elif self.last_turn_data == "Drive Forward":
                cv2.putText(self.frame, "Drive Forward", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 0), 3, cv2.LINE_AA)

        # Display the frame
        cv2.imshow('Frame', self.frame)
        cv2.waitKey(1)


    def lidarCallback(self, data):
        front_ray = min(data.ranges[179], 100 )
        right_ray = min(data.ranges[0], 100 )
        left_ray = min(data.ranges[359], 100)  
        
        print("Left Ray = " + str(left_ray))
        print("Front Ray = " + str(front_ray))
        print("Right Ray = " + str(right_ray))
        print(" - - - ")

        print("Movement To Do = " + self.last_turn_data)
        print("Flag = " + str(self.flag))
        print(" - - - - - ")


        if front_ray <= 0.66 and self.flag == False:
            # print(self.last_turn_data)
            # print("Front Ray = " + str(front_ray))
            if(self.last_turn_data == "left" or self.last_turn_data == "right" or self.last_turn_data == "stop"):
                self.linear = 0.1
                self.angular = 0.0
            else:
                self.linear = 0.0
                self.angular = 0.0
                self.qr_detector()

        if front_ray <= 0.30 and self.flag == False:
            self.flag = True
            self.linear = 0.0
            self.angular = 0.0

        if self.last_turn_data == "left" and self.flag == True:
            self.linear = 0.0
            self.angular = 0.2
            if(right_ray <= 0.358 and front_ray == 100):
                self.flag = False
                self.last_turn_data = "Drive Forward"
                self.linear = 0.0
                self.angular = 0.0

        if self.last_turn_data == "right" and self.flag == True:
            self.linear = 0.0
            self.angular = -0.2
            if(left_ray >= 0.35 and front_ray == 100):
                self.flag = False
                self.last_turn_data = "Drive Forward"
                self.linear = 0.0
                self.angular = 0.0

        if self.last_turn_data == "stop" and self.flag == True:
            self.linear = 0.0
            self.angular = 0.0
                
        if self.flag == False and self.last_turn_data == "Drive Forward":
            self.linear = 0.1
            error = left_ray - right_ray
            error = round(error, 3)
            if(abs(error) <= 0.20):
                self.angular = error
            else:
                error = 0.0
                self.angular = 0.0
            print("Error = " + str(error))
            print(" - - - - - - ")

            if self.frame is not None:  # Check if frame is valid before using it
                self.last_turn_data = "Drive Forward"

    def qr_detector(self):
        if self.frame is not None:  # Ensure frame is not None
            # Ensure frame is in 8-bit unsigned format
            if self.frame.dtype != 'uint8':
                self.frame = cv2.convertScaleAbs(self.frame)

            decoder = cv2.QRCodeDetector()
            data, self.points, _ = decoder.detectAndDecode(self.frame)
            self.last_turn_data = data

    def vel_pub(self):
        # Create the TwistStamped message
        twist_stamped = TwistStamped()
        
        # Set the header with timestamp and frame ID
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        # twist_stamped.header.frame_id = 'base_link'  # Replace with your frame ID
        
        twist_stamped.twist.linear.x = self.linear  
        twist_stamped.twist.angular.z = self.angular 
        
        # Publish the message
        self.velocity_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)

    camera_lidar_sub = CmaraLidarSub()
    rclpy.spin(camera_lidar_sub)
    
    camera_lidar_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
