import rclpy
from rclpy.node import Node
import tf_transformations
import cv2, math, time
import numpy as np
from cv_bridge import CvBridge
from math import atan2

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
        
        #homography of robots
        self.h=self.compute_homography()

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        #self.camera_subscriber = self.create_subscription(Image,'/RM0001/camera/image_raw',self.camera_callback,10)
        self.camera_subscriber = self.create_subscription(Image,'/RM0/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
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
        processe_f = self.compute_houghlines()

        #cv2.waitKey(1)

        #hsv = cv2.cvtColor(processe_f[3], cv2.COLOR_BGR2HSV)
        #lower_white = numpy.array([ 200, 200, 200])
        #upper_white = numpy.array([255, 255, 255])
        #mask = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = processe_f[0].shape
        #search_top = 3*h/4
        #search_bot = 3*h/4 + 20
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0

        M = cv2.moments(cv2.cvtColor(processe_f[2], cv2.COLOR_BGR2GRAY))
        if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(processe_f[2], (cx, cy), 10, (0,0,255), -1)
                #The proportional controller is implemented in the following four lines which
                #is reposible of linear scaling of an error to drive the control output.
                err = cx - w/2
                #self.twist.linear.x = 0.2
                self.angular = -float(err) / 100
        
        cv2.imshow('image2',processe_f[1])
        cv2.imshow('image3',processe_f[2])
        cv2.imshow('image4',processe_f[3])


        #self.get_logger().info(f"od={self.image})")



    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().info(
            "odometry: received pose (uuuuux: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
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
        self.linear=0.25
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

    
    def compute_homography(self):
        pts_src=np.array(([[338,	257],[233	,257], [352 ,	288],[180	,288] ]))
        pts_dst =np.array(([[338 ,	257] , [276	,257], [338	, 319], [276,	319]]))
        h, status = cv2.findHomography( pts_src,pts_dst)

        return h
    
    def compute_houghlines(self):

        image=self.image
        im_dst = cv2.warpPerspective(image, self.h, (640,360))
        image_re = im_dst[290:341,295:351]
        
        #Applying Gaussian Blur Before Thresholding
        #Since we know that the surface of the board is green we will only retain the green color channel to obtain better results
        blur = cv2.GaussianBlur(image_re[:,:,:],(5,5),0)

        #Applying Thresholding
        ret3,th2 = cv2.threshold(blur,90,255,cv2.THRESH_BINARY_INV)

        #Finding the edges of the boards
        edges=self.find_edges(th2)

        edges= cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
        #Finding the HoughLines from our threshold image
        lines = cv2.HoughLines(edges, 1, np.pi/180, 50)
  
        #print(np.shape(lines))
        
        #Hough lines returns the polar coordinates of line we will transform them to cartesian
        #The following part comes from the CV2 documentation
        #https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
        all_lines = []
        if lines is not None:
            #print(np.size(lines))
            x = y = 0
            for line in lines:



                        rho, theta = line[0][0], line[0][1]
                        #print(f"rho={rho},theta ={theta}")
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
                        x+=a
                        y+=b
            
            average_angle = atan2(y, x)
            print(lines)
            self.get_logger().info(f"average_angle:{average_angle}")
            
                    
            for line in all_lines:
                    image_re = cv2.line(image_re,(line[0],line[2]), (line[1],line[3]), (0, 0, 255), 1)

        return image_re ,blur,th2,edges,all_lines





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
