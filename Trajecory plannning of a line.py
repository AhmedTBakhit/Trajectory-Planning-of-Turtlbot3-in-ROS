#! /usr/bin/env python

#Importing neccessary nodes
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import transform3d
import math
from tf.transformations import euler_from_quaternion

class TurtleBot3Controller:

    def __init__(self) -> None:

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #rospy.Subscriber('/scan', LaserScan, self.callback_function)
        rospy.Subscriber('/odom', Odometry,self.pose_update)
        rospy.init_node('turtlebot3', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        #self.rate = rospy.Rate(2)


    def callback_function(self,msg):
        self.scan_ranges = np.array(msg.ranges)
        self.Increment = msg.angle_increment

#Function to calculate line
    def line_calculation(self):
        p1 = np.array([1,0,0])
        p2 = np.array([0,0,0])
        p1_r = self.tf.inv @ p1
        p2_r = self.tf.inv @ p2
        A,B,C = self.calculate(p1_r,p2_r)  #(A*p1[0]+B*p1[1]+C)
        self.distance = (A*p1[0]+B*p1[1]+C)/math.sqrt(A**2+B**2)
        self.angle = math.atan2(p2_r[1]-p1_r[1],p2_r[0]-p1_r[0])



        #updates position and orientation of the robot to the world
    def pose_update(self,msg):
        #Ax + By + C = 0
    
        #print(self.x)
        #self.y = (self.b*self.x+self.c)/self.a

        #position
        P = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
        #orientation
        O = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        #transforming
        self.tf = transform3d.Transform(p=P,quat=O)
        
   
    def drive_to_point(self):
        #define our gain
        linear_gain = 2
        angular_gain = 0.5
        #distance to goal
        distance_with_gain  = linear_gain*self.distance
        if distance_with_gain > math.pi/2:
            distance_with_gain = math.pi/2
        elif distance_with_gain < -math.pi/2:
            distance_with_gain = -math.pi/2

        if self.angle > 0:
            if self.angle < np.pi:
                self.angle = -self.angle
            else:
                self.angle = 2*np.pi - self.angle
        else:
            if self.angle > -np.pi:
                self.angle = -self.angle
            else:
                self.angle = -(2*np.pi + self.angle)
    
        theta_desired = (self.angle) + distance_with_gain
        
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z =angular_gain*theta_desired
        self.pub.publish(cmd_vel)

    def sleep(self):
        self.rate.sleep()    
    
    def calculate(self,p1,p2):
        A = p2[1]-p1[1]
        B = p1[0]-p2[0]
        C = p2[0]*p1[1]-p1[0]-p1[1]
        
        return A,B,C

if __name__ == '__main__':

    try:
        node = TurtleBot3Controller()
        
        
        while not rospy.is_shutdown():
            #node.x_y_location()
            node.line_calculation()
            node.drive_to_point()
            node.sleep()
            
            #node.sleep()

    except rospy.ROSInterruptException:
        pass
