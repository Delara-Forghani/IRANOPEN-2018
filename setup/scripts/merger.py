#!/usr/bin/env python

import rospy;
import math;
import threading;
import numpy as np;
from nav_msgs.srv import GetPlan ,GetPlanRequest,GetMap;
from nav_msgs.msg import Path , OccupancyGrid, Odometry;
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool;


name_space="robot1";
robot_number=0;
number_of_robots=0;
######################
merged_map_lock=threading.Lock()
merged_map=None;
#####################
goal_clients=[];
#############
odom_subscriber=None;
robot_x=0;
robot_y=0;
#################
beta=1;
alpha=25;
utility=100;
laser_range=30;
#################
my_server=None;
goals_list=[];
goals_list_lock=threading.Lock();
OurWrapper=[];
#############################
checking_goals_flag=False;
checking_goals_subscriber=None;
checking_goals_publisher=None;
############################
###############################
move_client_=None
move_client_goal_=None;
goal_pose=PoseStamped();
current_goal_status = 0 ; # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
move_base_status_subscriber=None;
#########################
visited_goals_list=[];

class MyWrapper:
    def __init__(self):
        self.map_pub=rospy.Publisher("/global_map", OccupancyGrid,queue_size=15);
        self.map_clients=[];
        for i in range(6):
            rospy.wait_for_service("/robot"+str(i)+"/dynamic_map")
            try:
                self.map_clients.append(rospy.ServiceProxy("/robot"+str(i)+"/dynamic_map", GetMap));
            except Exception as e:
                print ("problem")

    def set_Map(self):
        global merged_map_lock;
        global merged_map;
        try:
           map_data=self.map_clients[0]();
           merged_map=map_data.map;
        except Exception as e:
                print ("problem")
        for i in range(1,6):
            try:
            	map_data=self.map_clients[i]();
            	temp_map=np.array([map_data.map.data,merged_map.data]);
            	merged_map.data=list(np.max(temp_map,axis=0));
            except Exception as e:
                print ("problem")
        rate = rospy.Rate(0.5); 
        while not (rospy.is_shutdown()):
                self.map_pub.publish(merged_map);
		rate.sleep();
      
 
def main():
    rospy.init_node("merger_node");
  
    OurWrapper=MyWrapper();
    OurWrapper.set_Map();
    rospy.spin();

if __name__ == '__main__':
    main();
