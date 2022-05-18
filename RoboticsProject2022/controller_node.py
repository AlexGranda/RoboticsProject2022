import rclpy
from rclpy.node import Node
import tf_transformations
import cv2, math, time
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.br = CvBridge()

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        #self.camera_subscriber = self.create_subscription(Image,'/RM0001/camera/image_raw',self.camera_callback,10)
        self.camera_subscriber = self.create_subscription(Image,'/RM0/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.camera_subscriber

        self.linear  = 0.0 # [m/s]
        self.angular = 0.0 # [rad/s]

        # NOTE: we're using relative names to specify the topics (i.e., without a
        # leading /). ROS resolves relative names by concatenating them with the
        # namespace in which this node has been started, thus allowing us to
        # specify which Thymio should be controlled.

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/10, self.update_callback)

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def camera_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        cv2.imshow('image',self.process_image()[0])
        cv2.imshow('image2',self.process_image()[1])
        #current_frame = self.br.imgmsg_to_cv2(msg)
        #cv2.imshow("camera", current_frame)
        cv2.waitKey(1)



        #self.get_logger().info(f"od={self.image})")



    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )

        return pose2

    def update_callback(self):
        # Let's just set some hard-coded velocities in this example

        cmd_vel = Twist()
        cmd_vel.linear.x  = self.linear
        cmd_vel.angular.z = self.angular

        # Publish the command
        self.vel_publisher.publish(cmd_vel)

    def find_edges(self,img_thresh):
        G_x = cv2.Sobel(img_thresh,cv2.CV_64F,0,1)
        G_y = cv2.Sobel(img_thresh,cv2.CV_64F,1,0)
        G = np.abs(G_x) + np.abs(G_y)
        G = np.sqrt(np.power(G_x,2)+np.power(G_y,2))
        G[G >255] = 255
        edges=G.astype( 'uint8')

        return edges

    def process_image(self):
        lines=None
        cv2.imwrite('saved_image.jpg', self.image[:,:,:])
        image_ret=self.image[:,:,:]
        blur = cv2.GaussianBlur(image_ret,(5,5),0)
        ret3,th2 = cv2.threshold(blur,90,255,cv2.THRESH_BINARY_INV)
        edges = self.find_edges(th2)
        edges= cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 50)
        cv2.imwrite('edges.jpg', edges)

        all_lines = []
        if len(lines) !=0:
            for line in lines:

                rho, theta = line[0][0], line[0][1]
                #print(rho,theta)
                a = np.cos(theta)
                b = np.sin(theta)

                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 4000*(-b))
                y1 = int(y0 + 4000*(a))
                x2 = int(x0 - 4000*(-b))
                y2 = int(y0 - 4000*(a))

                all_lines.append([x1, x2, y1, y2])
            
            for line in all_lines:
                #pass
                #if line != None:  
                image_ret = cv2.line(image_ret,(line[0],line[2]), (line[1],line[3]), (0, 0, 255), 1)

        return image_ret,th2,edges



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '_main_':
    main()
