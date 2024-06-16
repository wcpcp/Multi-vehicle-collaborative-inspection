#!/usr/bin/env python
# from importlib.resources import path
import math
import copy
import numpy as np
import tf
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from nav_msgs.msg import Odometry
# nav_msgs/Odometry



people = Point()
people.x = 0
people.y = 0
dis_qingzhou_1_stop =0



class PathTracking(): 

    # def dis_cb(self,msg):
    #   self.msg =msg
    #   if abs(self.msg.pose.pose.position.x-19.6)<0.3 and abs(self.msgpose.pose.position.y-40)<0.3:
    #     while(True):
    #         print("stop!")
    #     twist = Twist()
    #     twist.linear.x=0
    #     twist.linear.y=0
    #     twist.angular.z=0
    #     self.cmd_vel_pub.publish(twist)
    #     rospy.signal_shutdown("Quit!")

        





    def loc_cb(self, point):
        print("recive people location")
        global people
        people = point
        


    def path_cb(self, path):
        self.updated_path = path

    def pure_pursuit(self):
        global people
        if people.x!=0 and people.y!=0:
            print("find people")
            # if self.robot_name=="qingzhou_1":
            print(self.robot_name+"qingzhou_1    stop")
            global dis_qingzhou_1_stop
            dis_qingzhou_1_stop+=1
            # twist = Twist()
            # twist.linear.x=0
            # twist.linear.y=0
            # twist.angular.z=0
            # self.cmd_vel_pub.publish(twist)
            # rospy.signal_shutdown("shutdown!")
            # return 0
    
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.path = copy.deepcopy(self.updated_path)
            self.d = []

            try:
                (trans, rot) = self.listener.lookupTransform("world", self.robot_name+'/base_link', rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rospy.loginfo_once('TF is ready')

            current_x = trans[0]
            current_y = trans[1]
            # print(current_x)
            # print(current_y)
            current_theta = tf.transformations.euler_from_quaternion(rot)[2]

            if self.robot_name !="qingzhou_1" or dis_qingzhou_1_stop>5:
                if abs(current_x-19.6)<2.0 and abs(current_y-30)<2.0:
                    twist = Twist()
                    twist.linear.x=0
                    twist.linear.y=0
                    twist.angular.z=0
                    self.cmd_vel_pub.publish(twist)
                    # while(True):
                    #     print("quit!")
                    rospy.signal_shutdown("Quit!")

            # find nearest points
            for i in range(len(self.path.poses)):
                dx = current_x - self.path.poses[i].pose.position.x
                dy = current_y - self.path.poses[i].pose.position.y
                self.d.append(np.hypot(dx, dy))
            
            if len(self.d):

                ind = np.argmin(self.d)
                lf_distance = 0
                while self.Lf > lf_distance and ind < len(self.path.poses) - 2:                       
                    delta_x = self.path.poses[ind+1].pose.position.x - self.path.poses[ind].pose.position.x
                    delta_y = self.path.poses[ind+1].pose.position.y - self.path.poses[ind].pose.position.y 
                    lf_distance = lf_distance + np.hypot(delta_x, delta_y)
                    if ind > len(self.path.poses):
                        break
                    ind = ind + 1
            
                target_x = self.path.poses[ind].pose.position.x
                target_y = self.path.poses[ind].pose.position.y
                
                # Calculate twist
                alpha = math.atan2(target_y - current_y, target_x - current_x) - current_theta
                alpha = np.mod(alpha + math.pi, 2*math.pi) - math.pi
                alpha = math.atan2(2 * 0.20 * np.sin(alpha), self.Lf)               



                # alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
                # if state.v < 0: 
                #     alpha = math.pi - alpha
                # Lf = k * state.v + Lfc
                # delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)


                twist = Twist()
                angular_z = 0.63 * alpha                                     


                
                # publish twist
                # if lf_distance > self.Lf / 2:
                #     twist.linear.x = 0.34
                #     # if angular_z > 1.0:
                #     #     angular_z = 1.0
                #     # elif angular_z < -1.0:
                #     #     angular_z = -1.0
                #     twist.angular.z = angular_z
                # elif lf_distance < self.Lf / 4:
                #     twist.linear.x = -0.1
                #     twist.angular.z = 0

                twist.linear.x = 1
                twist.angular.z = angular_z

                # if lf_distance < self.Lf / 4:
                #     twist.linear.x = -0.12
                #     twist.angular.z = 0

                # print("the lf_distance is :        "+str(twist.linear.x))
                # print("the linear is :        "+str(twist.linear.x))
                # print("the angular is :        "+str(angular_z))
                # print("the stamp is :        "+str(self.path.header.stamp.secs))
                if  target_x==0:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    
                self.cmd_vel_pub.publish(twist)
                
        
        rate.sleep()
        rospy.spin()

    def __init__(self):  

        rospy.init_node('path_tracking', anonymous=False)  
        # member var
        self.path = Path()
        self.updated_path = Path()
        self.d = []
        self.Lf = 0.25
        # param
        self.robot_name = rospy.get_param('~robot_name', 'qingzhou')
        self.plan_topic_name = rospy.get_param('~plan_topic', 'global_plan')
        # self.plan_topic_name = rospy.get_param('~plan_topic', '/move_base/TrajectoryPlannerROS/global_plan')

        # subs and pubs
        # self.path_sub = rospy.Subscriber(self.robot_name+self.plan_topic_name, Path, self.path_cb)
        self.path_sub = rospy.Subscriber(self.plan_topic_name, Path, self.path_cb)
        self.loc_sub = rospy.Subscriber("/qingzhou_1/loc_people", Point, self.loc_cb)

        # self.loc_sub = rospy.Subscriber('/'+self.robot_name+"/odom", Odometry, self.dis_cb)   

        self.cmd_vel_pub = rospy.Publisher( '/'+self.robot_name+'/ackermann_steering_controller/cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()

        

if __name__ == '__main__':  

    try:  
        tracker = PathTracking() 
        tracker.pure_pursuit()

    except rospy.ROSInterruptException:  

        rospy.loginfo("Path Tracking finished.")