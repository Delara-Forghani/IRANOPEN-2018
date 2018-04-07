#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdlib.h> // atof
#include <string>
#include <geometry_msgs/PoseWithCovariance.h>

double roll, pitch, yaw;

sensor_msgs::LaserScan scan;
ros::Publisher laser_publisher;

void remapping_function(const sensor_msgs::LaserScan ls){
  scan=ls;

}

void odom_callback(nav_msgs::Odometry q){

  tf::Matrix3x3 m(tf::Quaternion(q.pose.pose.orientation.x,q.pose.pose.orientation.y,q.pose.pose.orientation.z,q.pose.pose.orientation.w));
m.getRPY(roll, pitch, yaw);

}




int main(int argc, char** argv){
  std::string ns=argv[1];
  std::string topic=argv[2];
  ros::init(argc, argv, ns+"laser_remapper");
  ros::NodeHandle n;
  std::string laser_topic1="/"+ns+"/"+topic;
  std::string laser_topic2="/"+ns+"/remmaped_scan";
  std::string odom_topic="/"+ns+"/odom";
   ros::Publisher  laser_publisher = n.advertise<sensor_msgs::LaserScan>(laser_topic2,10);
   ros::Subscriber laser_subscirber= n.subscribe(laser_topic1,10,remapping_function);
   ros::Subscriber odom_subscirber= n.subscribe(odom_topic,10,odom_callback);
ros::Rate r(1);
while(n.ok()){
  if (pitch<0.1 && roll < 0.1 && pitch > -0.1 && roll > -0.1) {
   laser_publisher.publish(scan);
  }
  else {
  
  }
ros::spinOnce();
r.sleep();
}

  return 0;}
