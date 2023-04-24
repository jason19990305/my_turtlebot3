import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from cv_bridge import CvBridge

class MyPublisher(Node):

    def __init__(self):
        super().__init__('turtlebot3_control')
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_canny = self.create_publisher(Image, '/camera/canny/image_raw', 1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.ScanObstacle,10)
        self.sub_image = self.create_subscription(Image,'/camera/image_raw',self.ImageProcess,1)
        self.cvBridge = CvBridge()
        self.i = 0
    def ImageProcess(self,image_msg):
        cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg)#convert image to numpy format
        gray = cv2.cvtColor(cv_image_input, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(gray, 30, 150)
        
        #image = np.zeros((480,320))
        
        self.pub_canny.publish(self.cvBridge.cv2_to_imgmsg(canny))#publish image 
    
    def Move(self,linear_vel,angular_vel):#-0.22~0.22  , -2.84~2.84
        twist = Twist()

        twist.linear.x = float(linear_vel )#+forward  -backward  -0.22~0.22
        twist.linear.y = 0.0 
        twist.linear.z = 0.0

        twist.angular.x = 0.0 
        twist.angular.y = 0.0 
        twist.angular.z = float(angular_vel)#+left  -right    -2.84~2.84

        print('linear velocity %0.2f	 angular velocity %0.2f'%(linear_vel,angular_vel))

        self.pub_twist.publish(twist)

    def ScanObstacle(self,scan):
        angle_range = 40
        scan_start_left = 0
        scan_end_left = 0 + angle_range

        scan_start_right = 360 - angle_range
        scan_end_right = 360
        
        threshold_distance = 0.4# m
        is_obstacle_L = False
        is_obstacle_R = False
        for degree in range(scan_start_left, scan_end_left):
            if scan.ranges[degree] < threshold_distance:#left side have obstacle
                is_obstacle_L = True

        for degree in range(scan_start_right, scan_end_right):
            if scan.ranges[degree] < threshold_distance:#right side have obstacle
                is_obstacle_R = True
                
        if is_obstacle_L:
            self.Move(0.02,-0.5)#turn right
        elif is_obstacle_R:
            self.Move(0.02,0.5)#turn left
        else:
            self.Move(0.1,0)#go forward


def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
