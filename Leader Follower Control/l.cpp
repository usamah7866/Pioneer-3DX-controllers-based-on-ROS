#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;

double current_time = 0.0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "circle_leader");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/leader/cmd_vel", 10);
  

  // Wait for Gazebo to be ready and clock to start
  ROS_INFO("Waiting for /clock to start...");
  while (ros::Time::now().toSec() == 0) {
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("Clock started, beginning to publish...");
  ros::Rate rate(10);
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.2;
  cmd.angular.z = 0.2;

  double start_time = ros::Time::now().toSec();

  while (ros::ok()) {
    
    current_time = ros::Time::now().toSec() - start_time;
    pub.publish(cmd);
    rate.sleep();
    if (current_time > 34) {
      ROS_INFO("Reached target within tolerance.");
      break;
    }
  }
  cmd.linear.x = 0;
  cmd.angular.z = 0;
  pub.publish(cmd);

  return 0;
}
