#!/usr/bin/env python
# coding:utf-8 
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import interp1d      # Interpolation module
from nav_msgs.msg import Path
import numpy as np
from qingzhou_nav.msg import points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

# people = Point()
# people.x=0
# people.y=0


# def B_nx(n, i, x):
#     if i > n:
#         return 0
#     elif i == 0:
#         return (1-x)**n
#     elif i == 1:
#         return n*x*((1-x)**(n-1))
#     return B_nx(n-1, i, x)*(1-x)+B_nx(n-1, i-1, x)*x

# def get_value(p, canshu):
#     sumx = 0.
#     sumy = 0.
#     length = len(p)-1
#     for i in range(0, len(p)):
#         sumx += (B_nx(length, i, canshu) * p[i][0])
#         sumy += (B_nx(length, i, canshu) * p[i][1])
#     return sumx, sumy

# def get_newxy(p,x):
#     xx = [0] * len(x)
#     yy = [0] * len(x)
#     for i in range(0, len(x)):
#         print('x[i]=', x[i])
#         a, b = get_value(p, x[i])
#         xx[i] = a
#         yy[i] = b
#         print('xx[i]=', xx[i])
#     return xx, yy

# p = np.array([                           
#     [2, -4],
#     [3, 8],
#     [5, 1],
#     [7, 6],
#     [9, 4],pip install scipy
#     [7, 1],
# ])

# x = np.linspace(0, 1, 101)
# xx, yy = get_newxy(p, x)
# plt.plot(xx, yy, 'r', linewidth=1)       
# plt.scatter(xx[:], yy[:], 1, "blue")    
# plt.show()

# def float_range(start, stop, steps):
#     return [start + float(i) * (stop - start) / (float(steps) - 1) for i in range(steps)]

class PathPlanning():

    # def do_stop(self,msg):
    #     self.xy[0,0] = msg.x_c
    #     self.xy[0,1] = msg.y_c
    #     print("stop "+self.xy)
        
    #     if abs(msg.x_c-19.6)<0.25 and abs(msg.y_c-40)<0.25:
    #         rospy.signal_shutdown("shutdown !")
    # def wpts_cb(self, msg):
    #     self.wpt = np.zeros((1,2))
    #     self.wpt[0,0] = msg.point.x
    #     self.wpt[0,1] = msg.point.y
    #     self.wpts = np.append(self.wpts, self.wpt, axis = 0)
        # self.pt = Marker()
        # self.pt.header.frame_id = 'world'
        # self.pt.header.stamp = rospy.Time.now()
        # self.pt.ns = "tag"
        # self.pt.id = self.i
        # self.pt.action = Marker.ADD
        # self.pt.type = Marker.CYLINDER

        # self.pt.pose.position.x = msg.point.x
        # self.pt.pose.position.y = msg.point.y
        # self.pt.pose.position.z = msg.point.z
        
        # self.pt.pose.orientation.w = 1.0
        # self.pt.scale.x = 0.05
        # self.pt.scale.y = 0.05
        # self.pt.scale.z = 0.3
		
        # self.pt.color.r = 1.0
        # self.pt.color.g = 0.0
        # self.pt.color.b = 0.0
        # self.pt.color.a = 1.0
		
        # self.i = self.i + 1

        # self.pts_marker.markers.append(self.pt)
        
        # rospy.loginfo("Add way point successfully!")

    def do_people(self,point):    
        self.people = point


    def do_points(self,msg):
        if self.people.x==0 and self.people.y==0:
            self.wpts = np.empty((0,2))
            self.wpt = np.zeros((1,2))
            self.wpt[0,0] = msg.x_p
            self.wpt[0,1] = msg.y_p
            self.wpts = np.append(self.wpts, self.wpt, axis = 0)
            self.wpt[0,0] = msg.x_c
            self.wpt[0,1] = msg.y_c
            self.wpts = np.append(self.wpts, self.wpt, axis = 0)
            self.wpt[0,0] = msg.x_n
            self.wpt[0,1] = msg.y_n
            self.wpts = np.append(self.wpts, self.wpt, axis = 0)
            self.wpt[0,0] = msg.x_n_1
            self.wpt[0,1] = msg.y_n_1
            self.wpts = np.append(self.wpts, self.wpt, axis = 0)

            self.i = self.i+1

            if len(self.wpts) > 3:

                # t = np.linspace(0, 1, 101)
                # newx, newy = get_newxy(self.wpts, t)

                x = self.wpts[:,0]
                y = self.wpts[:,1]
                t = np.linspace(0, 1, num=len(x))
                f1 = interp1d(t,x,kind='cubic')       #三样条曲线算法
                f2 = interp1d(t,y,kind='cubic')
                newt = np.linspace(0,1,30*len(self.wpts))
                newx = f1(newt)
                newy = f2(newt)	
                self.wps_global = np.column_stack((newx, newy))

                self.global_path = Path()
                self.global_path.header.stamp = rospy.Time.now()
                self.global_path.header.frame_id = 'world'

                for i in range(len(self.wps_global)):
                    pose = PoseStamped()
                    pose.pose.position.x = self.wps_global[i,0]
                    pose.pose.position.y = self.wps_global[i,1]
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = 'world'	

                    self.global_path.poses.append(copy.deepcopy(pose))
                self.global_path_pub.publish(self.global_path)
                
        else:
            if self.robot_name !="qingzhou_1":
                print("find people  send  direct path")
                xx = []
                yy = []
                start = Point()
                start.x = msg.x_c
                start.y = msg.y_c

                end = Point()
                end.x = self.people.x
                end.y = self.people.y

                if start.x>=end.x and self.cnt ==0:
                    self.step = -self.step

                self.cnt+=1
                
                for x in np.arange(start.x,end.x,self.step):
                    y =(x*(start.y-end.y)+end.y*start.x-end.x*start.y)/(start.x-end.x)
                    xx.append(x)
                    yy.append(y)

                self.global_path = Path()
                self.global_path.header.stamp = rospy.Time.now()
                self.global_path.header.frame_id = 'world'
                for i in range(len(xx)):
                    pose = PoseStamped()
                    pose.pose.position.x = xx[i]
                    pose.pose.position.y = yy[i]
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = 'world'	

                    self.global_path.poses.append(copy.deepcopy(pose))
                self.global_path_pub.publish(self.global_path)
            else:
                self.global_path.poses=[]
                self.global_path_pub.publish(self.global_path)
            


    # def cubic_spline(self):
    #     self.rate = rospy.Rate(2)
    #     while not rospy.is_shutdown():
    #         # if self.pts_marker:
    #         #     self.marker_pub.publish(self.pts_marker)       
    #         if len(self.wpts) > 3:
    #             x = self.wpts[:,0]
    #             y = self.wpts[:,1]
    #             t = np.linspace(0, 1, num=len(x))
    #             f1 = interp1d(t,x,kind='cubic')       #三样条曲线算法
    #             f2 = interp1d(t,y,kind='cubic')
    #             newt = np.linspace(0,1,20*len(self.wpts))
    #             newx = f1(newt)
    #             newy = f2(newt)	
    #             self.wps_global = np.column_stack((newx, newy))
    #             self.global_path = Path()
    #             self.global_path.header.stamp = rospy.Time.now()
    #             self.global_path.header.frame_id = 'world'

    #             for i in range(len(self.wps_global)):
    #                 pose = PoseStamped()
    #                 pose.pose.position.x = self.wps_global[i,0]
    #                 pose.pose.position.y = self.wps_global[i,1]
    #                 pose.header.stamp = rospy.Time.now()
    #                 pose.header.frame_id = 'world'	

    #                 self.global_path.poses.append(copy.deepcopy(pose))
    #             self.global_path_pub.publish(self.global_path)
    #         self.rate.sleep()

    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)
        # param
        self.robot_name = rospy.get_param('~robot_name', 'qingzhou')
        self.plan_topic_name = rospy.get_param('~plan_topic', ' ')
        # subs and pubs
        # self.marker_pub = rospy.Publisher("way_points", MarkerArray, queue_size=1)
        self.global_path_pub = rospy.Publisher(self.plan_topic_name, Path, queue_size=10)
        # self.way_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.wpts_cb, queue_size=1)
        self.points_sub = rospy.Subscriber("three_points",points,self.do_points,queue_size=10)
        self.people_sub = rospy.Subscriber("/qingzhou_1/loc_people",Point,self.do_people,queue_size=10)

        # self.points_sub = rospy.Subscriber("stop_car",points,self.do_stop,queue_size=10)


        
        # member var
        # self.wpts = np.empty((0,2))
        # self.pts_marker = MarkerArray()
        self.i = 0
        self.step = 0.5
        self.people = Point()
        self.people.x=0
        self.people.y=0
        self.cnt=0

if __name__ == "__main__":
    try:
        planner = PathPlanning()
        # planner.cubic_spline()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Path planning is finished')

