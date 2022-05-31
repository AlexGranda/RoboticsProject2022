import rclpy
from rclpy.node import Node
import tf_transformations
import cv2, math, time
import numpy as np
from cv_bridge import CvBridge
from math import atan2
import os

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage



import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.br = CvBridge()

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.path =  os.getcwd()
        self.probahough=True

        #For the array of positions
        self.future_pos=None
        self.past_pos=None
        
        #homography of robots
        self.h=self.compute_homography()
        self.c_fps=0

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        #self.camera_subscriber = self.create_subscription(Image,'/RM0001/camera/image_raw',self.camera_callback,10)
        #self.camera_subscriber = self.create_subscription(Image,'/RM0/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.camera_subscriber = self.create_subscription(Image,'/RM0/camera/image_raw',self.camera_callback,10)
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
        self.get_logger().info(f"od={self.c_fps})")
        self.image = self.br.imgmsg_to_cv2(msg)
        #processe_f = self.compute_houghlines()
        processe_f = self.compute_houghlinesP()
        h, w, d = processe_f[0].shape
        prev_ang=self.angular

        self.c_fps+=1 #number of frames processed
        self.past_pos = np.copy(self.future_pos)
        self.future_pos = self.masking_steps(processe_f[2],h,w,5)
        future_copy = self.future_pos.copy()
        future_copy = np.array(future_copy)
        averages = np.copy(self.future_pos)

        if self.past_pos.any() and future_copy.any():
            #self.get_logger().info(f'Past positions: {self.past_pos}')
            #self.get_logger().info(f'Future positions: {future_copy}')
            arr = np.ma.empty((12, 2, 2))
            arr.mask = True
            arr[:self.past_pos.shape[0], :self.past_pos.shape[1] , 0] = self.past_pos
            arr[:future_copy.shape[0], :future_copy.shape[1], 1] = future_copy
            # averages = np.mean( np.array([ self.past_pos, future_copy ]), axis=0 )
            averages = arr.mean(axis = 2)
            #self.get_logger().info(f'Fancy averages from positions: {arr.mean(axis = 2)}')
            #self.get_logger().info(f'Averages from positions: {averages}')

        
        mask=cv2.cvtColor(processe_f[2],cv2.COLOR_BGR2GRAY)
        search_top = int(round(3*h/4,0))
        #print(search_top)
        search_bot = int(round(3*h/4,0)) +15
        
        mask[0:search_top, 0:w] = 0
        #Computes the middle point the red
        #this part computes the center of the lower part of the image
        M = cv2.moments(mask)
        err=0
        err_=0
        #current_center = self.future_pos.pop(0)
        #self.future_pos = averages
        #cx=current_center[0]
        #cy=current_center[1]

        cx=None
        cy=None
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            cv2.circle(processe_f[0], (cx, cy), 10, (0,0,255), -1)
            
            #The proportional controller is implemented in the following four lines which
            #is reposible of linear scaling of an error to drive the control output.
            err = cx - w/2
            err_ = cx - w/2
            if err <0:
                err= -(err**2)
            else:
                err=err**2
            #self.twist.linear.x = 0.2
        anglecirc=anglecirc2=0
        angle=self.angle_turn(processe_f[5])
        angulo_prueba=self.angular

        if (self.odom_pose is not None) and processe_f[6]:
            if processe_f[5] >= (math.pi/2):
                angulo_prueba=round(processe_f[5]-self.pose3d_to_2d(self.odom_pose)[2]-math.pi,2)
            else:
                angulo_prueba=round(processe_f[5]-self.pose3d_to_2d(self.odom_pose)[2],2)

        
        


            





        erro2=0
        #Follow lines with houghcircle adjustement
        #if circles is not None
        if processe_f[8] is not None and cx !=None and cy!=None :
            #self.get_logger().info(f"CIRC: {processe_f[8][0][0]}")
            circ=processe_f[8][0][0]
            anglecirc=round(math.atan2(circ[0]-w/2, circ[1]-h),2)
            self.get_logger().info(f"CIRC: {circ[0]},{circ[1]}, point {cx},{cy} ")
            if anglecirc<0:
                anglecirc=anglecirc+math.pi
            anglecirc2=self.angle_turn(anglecirc)
            
            
            if   ((anglecirc < math.pi/2 and  processe_f[5] < math.pi/2) and abs(processe_f[5]-anglecirc)<.52) or len(self.future_pos)>6:
                #self.angular = float(angle/(math.pi))
                self.angular = float(angle)/2.5-(float(err) / 10000)*6

                cv2.line(processe_f[0],(int(circ[0]) ,int(circ[1])), (int(w/2),int(h)), (255, 255, 255), 2)
                #cv2.circle(processe_f[0],(int(circ[0]),int(circ[0])),int(circ[2]),(255,255,255),2)
                #cv2.circle(processe_f[0],(int(circ[0]),int(circ[1])),2,(255,255,255),3)

            else:
                erro2 = -cx + circ[0]*2
                if erro2 <0:
                    erro2= -(erro2**2)
                else:
                    erro2=erro2**2
                
                cv2.line(processe_f[0],(int(circ[0]) ,int(circ[1])), (int(w/2),int(h)), (0, 255, 255), 2)
                #cv2.circle(processe_f[0],(int(circ[0]),int(circ[0])),int(circ[2]),(0,255,255),2)
                #cv2.circle(processe_f[0],(int(circ[0]),int(circ[1])),2,(0,255,255),3)   

                self.angular=float(anglecirc2)*1.5
                #self.angular = float(angulo_prueba)*3
                #self.angular=-(float(erro2) / 10000)*3


            
        else : 

            #If circule not found line  with houghlines
            #self.angular = float(angle/(math.pi))
            self.angular = float(angle)/2.5-(float(err) / 10000)*6
            #self.angular = float(angulo_prueba)*3

            #If circule not found line   follow line with middle point
            #self.angular=-(float(err) / 10000)*3


        #comment both angular velocities if you want to take in to account the circle center
        #follow line with middle point
        #self.angular = float(angle/math.pi)

        #follow line with middle point
        #self.angular=-(float(err) / 10000)*3
        
 
        self.get_logger().info(f"float: {-round((float(err) / 10000)*3,2)} , average_angle_vel:{ -float(angle/math.pi)} ,tunr= {angle}")
        self.get_logger().info(f"circle_angle: {anglecirc2} , circ_vel:{ float(anglecirc2)/(math.pi**2)} ")

        self.linear = .25 
        #mini = np.concatenate((processe_f[0],processe_f[7]), axis=1)
        #Saves images of the robot camera
        #t=cv2.imwrite(os.path.join(self.path ,'src/RoboticsProject2022/video',"image_"+str(self.c_fps)+".png"), mini)
        #j=cv2.imwrite(os.path.join(self.path ,'src/RoboticsProject2022/video/testing',"image_"+str(self.c_fps)+".png"), processe_f[7])
        #self.get_logger().info(xf"{self.path} ,wrtie {t}")

        #f processe_f[6] ==  False:
        #    self.angular = prev_ang
        
        #if processe_f[5] < 1.8 and processe_f[5] >1.3 and cx is None:
        #    self.angular=0.0

        



        #images in real time

        image3=np.zeros_like(processe_f[1])
        image3[:, :,0]=processe_f[3]
        image3[:, :,1]=processe_f[3]
        image3[:, :,2]=processe_f[3]

 
        ###IMAGE_LOGGING

        cv2.putText(processe_f[7], text="avg_angle: "+str(round(processe_f[4],3)),org=(2,10), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        cv2.putText(processe_f[7], text="fst_angle: "+str(round(processe_f[5],3)),org=(2,20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(200,250,240), thickness=1)
        cv2.putText(processe_f[7], text="ang circ line: "+str(anglecirc),org=(2,30), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(200,250,240), thickness=1)
        cv2.putText(processe_f[7], text="houghline acc : "+str(round(float(angle)/2.5,3)),org=(2,40), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(200,250,240), thickness=1)
        cv2.putText(processe_f[7], text="midp acc: "+str(-round((float(err) / 10000)*3,3)),org=(2,50), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(200,250,240), thickness=1)
        cv2.putText(processe_f[7], text="circ_acc: "+str(round(float(anglecirc2)*2,3))+" r/s",org=(2,60), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        cv2.putText(processe_f[7], text="angular_vel: "+str(round(self.angular,3))+" r/s",org=(2,70), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        cv2.putText(processe_f[7], text="error in x axis: "+str(round(err_,3))+" r/s",org=(2,80), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        if self.odom_pose is not None:
            cv2.putText(processe_f[7], text="pose x: "+str(round(self.pose3d_to_2d(self.odom_pose)[0],3)),org=(2,90), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            cv2.putText(processe_f[7], text="pose y: "+str(round(self.pose3d_to_2d(self.odom_pose)[1],3)),org=(2,100), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            cv2.putText(processe_f[7], text="pose theta: "+str(round(self.pose3d_to_2d(self.odom_pose)[2],3)),org=(2,110), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            cv2.putText(processe_f[7], text="dif theta: "+str(angulo_prueba),org=(2,120), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        cv2.putText(processe_f[7], text="circ theta: "+str(anglecirc),org=(2,130), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
        cv2.putText(processe_f[7], text="shape: "+str(len(self.future_pos)),org=(2,140), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)






        horizontal = np.concatenate((processe_f[0],processe_f[2],image3), axis=1)
        #self.get_logger().info(f"h: {np.shape(horizontal)} , log:{ np.shape(processe_f[7])} ")
        horizontalf = np.concatenate((horizontal,processe_f[7]), axis=0)
        t=cv2.imwrite(os.path.join(self.path ,'src/RoboticsProject2022/video',"image_"+str(self.c_fps)+".png"), horizontalf)

        cv2.imshow('Processed images',horizontalf)
        
        cv2.waitKey(1)



    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        #self.get_logger().info(
        #    "odometry: received pose (uuuuux: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        #
    



    
    
    
    def masking_steps(self,A,h,w,size):

        #This creates an list of the centers of image sections

        # future_pos=[None]*int(h/size)
        future_pos = []

        for i in range(0, int(h/size)):
            
            mask_range_bot=(i)*size
            mask_range_top=(i+1)*size
            mask=cv2.cvtColor(A,cv2.COLOR_BGR2GRAY)
            mask[0:mask_range_bot, 0:w] = 0
            mask[mask_range_top:h, 0:w]=0

            M = cv2.moments(mask)
            err=0
            if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # future_pos[i]=[cx,cy]
                    future_pos.append([cx,cy])
                    #cv2.circle(mask, (cx, cy), 10, (255,255,255), -1)
        
        return future_pos

    


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
        #self.linear=0.25
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

    def apply_transformation(self):
        
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

        return image_re,blur,th2,edges
    
    def angle_turn(self, angle):

        #Adjustement for the angle since the angle returned by the HoughlinesP goes from pi/2 to pi for right turns
        #and  0 to pi/2 for left turns

        ang_ret=0
        sensitivy=.08

        #left turn
        if angle>=0 and angle<(math.pi/2) :

            if angle < sensitivy:
                ang_ret=0
            else:

                ang_ret= angle
        
        #right turn
        else :

            if math.pi-angle < sensitivy:
                ang_ret=0
            else:
                ang_ret= angle-math.pi


        return ang_ret
    
    def compute_houghlines(self):

        all_lines = []
        average_angle=0.0
        average_angle=self.angular
        r_angle=a=b=0.0
        l_found=False
        canvas = np.zeros(100,56*3,3)

        image_re,blur,th2,edges= self.apply_transformation()

        #Finding the HoughLines from our threshold image
        lines = cv2.HoughLines(edges, 1, np.pi/180, 50)
        if lines is None :
            lines = cv2.HoughLines(edges, 1, np.pi/180, 25)
        #print(np.shape(lines))
        
        #Hough lines returns the polar coordinates of line we will transform them to cartesian
        #The following part comes from the CV2 documentation
        #https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
        if lines is not None:
            l_found =True
            #print(np.size(lines))
            x = y = 0.0
            for line in lines:
                        rho, theta = line[0][0], line[0][1]
                        #print(f"rho={rho},theta ={theta}")
                        #print(rho,theta)
                        a = np.cos(theta)
                        b = np.sin(theta)

                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 90*(-b))
                        y1 = int(y0 + 90*(a))
                        x2 = int(x0 - 90*(-b))
                        y2 = int(y0 - 90*(a))

                        all_lines.append([x1, x2, y1, y2])
                        x+=a
                        y+=b

            s=np.shape(lines)
            average_angle = round(atan2(y/s[0],x/s[0]),2)
            r_angle = round(atan2(b, a),2)
            
            for line in all_lines:
                image_re = cv2.line(image_re,(line[0],line[2]), (line[1],line[3]), (0, 0, 255), 2)
                #canvas = cv2.line(canvas,(line[0],line[2]), (line[1],line[3]), (255, 255, 255), 2)
            
            #cv2.putText(image_re, text="a: "+str(average_angle),org=(2,10), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            #cv2.putText(image_re, text="avg angle: "+str(r_angle),org=(2,20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)

            cv2.putText(canvas, text="avg angle: "+str(average_angle),org=(2,10), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            cv2.putText(canvas, text="fst angle: "+str(r_angle),org=(2,20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)

        return image_re ,blur,th2,edges,average_angle,r_angle,l_found,canvas


        
    def compute_houghlinesP(self):

        all_lines = []
        average_angle=0.0
        #average_angle=self.angular
        r_angle=a=b=0.0
        l_found=False
        circles = None
        canvas = np.zeros((200,56*3,3))

        image_re,blur,th2,edges= self.apply_transformation()

        #Finding the HoughLines from our threshold image
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50,3,20)
        if lines is None :
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 25,3,20)
        #print(np.shape(lines))
        
        #Hough lines returns the polar coordinates of line we will transform them to cartesian
        #The following part comes from the CV2 documentation
        #https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
        all_lines = []
        if lines is not None:
            l_found=True
            for line in lines:

                for x1,y1,x2,y2 in line:
                    cv2.line(image_re,(x1,y1),(x2,y2),(0,255,0),2)
                    
                    #cv2.line(canvas,(x1,y1),(x2,y2),(255,255,255),2)
                    a+=x2-x1
                    b+=y2-y1

                    r_angle = atan2(x2-x1,y2-y1)
                        
                        
            s=np.shape(lines)
            average_angle = atan2(a/s[0],b/s[0])
            
            

            #cv2.putText(image_re, text="a: "+str(average_angle),org=(2,10), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)
            #cv2.putText(image_re, text="r: "+str(r_angle),org=(2,20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=.80, color=(50,250,240), thickness=1)

        #circles = cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=15,minRadius=5,maxRadius=30)
        circles = cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=15,minRadius=5,maxRadius=30)

        
        #if circles is not None:
        #    for i in circles[0,:]:
        #        # draw the outer circle
        #        cv2.circle(image_re,(int(i[0]),int(i[1])),int(i[2]),(255,255,255),2)
        #        #draw the center of the circle
        #        cv2.circle(image_re,(int(i[0]),int(i[1])),2,(255,255,255),3)



        return image_re ,blur,th2,edges,average_angle,r_angle,l_found,canvas,circles
        





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