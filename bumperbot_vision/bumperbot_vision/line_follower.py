import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import time


class LineFollowerNode(Node):

    def __init__(self):
        super().__init__("qr_maze_solver_node")
        self.bridge = CvBridge()
        self.frame = None  # Initialize to None
        self.error = 0.0
        self.midpoint = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.sub_camera = self.create_subscription(Image, "/camera/image_raw", self.cameraCallback, 10) 
        self.velocity_pub = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        
    def cameraCallback(self, img):
        self.frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        frame_cropped = self.frame[200:475, 100:545] #First Y then X
        edges_frame = cv2.Canny(frame_cropped, 60, 100)

        white_index = []
        for index,value in enumerate(edges_frame[:][200]): #here we get all values of x about column 200 in y.
            if(value == 255):
                white_index.append(index)
        # print(white_index)

        if(len(white_index) == 2):
            cv2.circle(img = edges_frame, center = (white_index[0], 200), radius = 2, color= (255,255,255), thickness = 4)
            cv2.circle(img = edges_frame, center = (white_index[1], 200), radius = 2, color= (255,255,255), thickness = 4)
            self.midpoint = int((white_index[0] + white_index[1]) / 2)
            cv2.circle(img = edges_frame, center = (self.midpoint, 200), radius = 6, color= (255,0,0), thickness = 2)

        goal_point = [222, 200] #midpoint of canny edge frame along column 200
        cv2.circle(img = edges_frame, center = (goal_point[0], goal_point[1]), radius = 2, color= (255,255,255), thickness = 4)

        self.error = goal_point[0] - self.midpoint
        print("Error = " + str(self.error))
        print(" - - - - - ")
        if(len(white_index) == 2):
            self.linear = 0.1
            self.angular = 0.01 * self.error
            # print(self.angular)

        if(len(white_index) == 0):
            self.linear = 0.0
            self.angular = 0.0

        self.vel_pub()
        

        # cv2.imshow('Frame', self.frame)
        # cv2.imshow('Cropped Frame', frame_cropped)
        cv2.imshow('Canny Edge Frame', edges_frame)
        cv2.waitKey(1)
    
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

    line_follower = LineFollowerNode()
    rclpy.spin(line_follower)
    
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
