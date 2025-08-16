#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <cmath>
#include <vector>
#include <cstdlib> // For system()
#include <cstdio>  // For popen()
#include <iostream>
#include <fstream>

using namespace std;

// Global variables for current position
nav_msgs::Odometry current_position;
ros::Publisher pub;
bool odom_received = false;

// Robot parameters
double v_r = 0.25; // Desired linear velocity (m/s)
double w_r = 0.5; // Desired angular velocity (rad/s)
double theta_r = 0.0; // Desired orientation (rad)
double e_x = 0.0, e_y=0.0, e_theta=0.0;
double x_ref, y_ref;
double x_m, y_m, theta_m;
double current_time, v_d, w_d, v_m, w_m, ev, ew, v_dd, w_dd;
double desired_angle;

// Vectors to log trajectory data
vector<double> actual_x, actual_y, desired_x, desired_y;
vector<double> lref, lreal, aref, areal;
vector<double> theta_rr, theta_mm;

// Callback to update current position
void positionCallback(const nav_msgs::Odometry &msg) {
    current_position = msg;
    odom_received = true;
}


// Main control function
void followCircularTrajectory() {
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(10); // Control loop frequency (Hz)
    double start_time = ros::Time::now().toSec();

    while (ros::ok()) {
        if (!odom_received) {
            ros::spinOnce();
            continue;
        }

        // Calculate elapsed time
        current_time = ros::Time::now().toSec() - start_time;
        theta_r = theta_r + (w_r * 0.1);
        x_ref = (v_r/w_r) * cos(theta_r);
        y_ref = (v_r/w_r)  * sin(theta_r);
        
        // Log desired trajectory
        desired_x.push_back(x_ref);
        desired_y.push_back(y_ref);
        

        // Get the current position and orientation
        x_m = current_position.pose.pose.position.x;
        y_m = current_position.pose.pose.position.y;
        theta_m = tf::getYaw(current_position.pose.pose.orientation);
        
        // Log actual trajectory
        actual_x.push_back(x_m);
        actual_y.push_back(y_m);
        theta_mm.push_back(theta_m);

        e_x = ((x_ref - x_m) * cos(theta_m)) + ((y_ref - y_m) * sin(theta_m));

        e_y = - ((x_ref - x_m) * sin(theta_m)) + ((y_ref - y_m) * cos(theta_m));

        desired_angle = atan2(y_ref - y_m , x_ref - x_m);
        e_theta = angles::shortest_angular_distance(theta_m, desired_angle);
        theta_rr.push_back(desired_angle);

        v_d = (0.1 * e_x) + (v_r * cos(e_theta));
        w_d = (0.1 * v_r * e_y) + w_r + (0.1 * sin(e_theta));

        // Set velocity commands
        cmd_vel.linear.x = v_d;
        cmd_vel.angular.z = w_d;

        // Publish velocity commands
        pub.publish(cmd_vel);

        // Log velocities
        lref.push_back(v_d);
        lreal.push_back(current_position.twist.twist.linear.x);
        aref.push_back(w_d);
        areal.push_back(current_position.twist.twist.angular.z);
        // cout<<"e_x: "<<e_x<<" e_y: "<<e_y<<" l: "<<v_d<<" a: "<<w_d<<" t: "<<e_theta<<endl;
        // Stop if within tolerance
        if (/*ev < tolerance ||*/ current_time > 34) {
            ROS_INFO("Reached target within tolerance.");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Stop the robot
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub.publish(cmd_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circular_trajectory_controller");
    // nh is for gazebo nad nh1 and 2 are for experiment
    ros::NodeHandle nh1, nh2, nh;

    // Subscriber and Publisher
    // ros::Subscriber sub = nh.subscribe("/odom", 10, positionCallback);
    // pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub=nh2.subscribe("/RosAria/pose",10,&positionCallback);
    pub =nh1.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
    // Wait for odometry data
    ROS_INFO("Waiting for odometry data...");
    ros::Rate wait_rate(10);
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Odometry data received. Starting circular trajectory.");

    // Start following the trajectory
    followCircularTrajectory();

    // Plot the trajectory using Gnuplot
    FILE *gnuplotPipe = popen("gnuplot -persistent", "w");
    if (gnuplotPipe) {
        fprintf(gnuplotPipe, "set title 'Desired vs Actual Trajectory'\n");
        fprintf(gnuplotPipe, "set xlabel 'X Position (m)'\n");
        fprintf(gnuplotPipe, "set ylabel 'Y Position (m)'\n");
        fprintf(gnuplotPipe, "plot '-' with lines title 'Desired Trajectory' lc rgb 'blue', '-' with lines title 'Actual Trajectory' lc rgb 'red'\n");

        // Send desired trajectory data
        for (size_t i = 0; i < desired_x.size(); i++) {
            fprintf(gnuplotPipe, "%f %f\n", desired_x[i], desired_y[i]);
        }
        fprintf(gnuplotPipe, "e\n");

        // Send actual trajectory data
        for (size_t i = 0; i < actual_x.size(); i++) {
            fprintf(gnuplotPipe, "%f %f\n", actual_x[i], actual_y[i]);
        }
        fprintf(gnuplotPipe, "e\n");
        fflush(gnuplotPipe);
        pclose(gnuplotPipe);
    } else {
        ROS_ERROR("Could not open Gnuplot.");
    }

    FILE *gnuplotPipe2 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe2) {
        fprintf(gnuplotPipe2, "set title 'linear velocity'\n");
        fprintf(gnuplotPipe2, "set xlabel 'Time Step'\n");
        fprintf(gnuplotPipe2, "set ylabel 'v'\n");
        fprintf(gnuplotPipe2, "plot '-' with lines title 'Desired V_r' lc rgb 'green', '-' with lines title 'Actual V_r' lc rgb 'purple'\n");

         // Linear velocity data
        for (size_t i = 0; i < lref.size(); i++) {
            fprintf(gnuplotPipe2, "%lu %f\n", i, lref[i]);
        }
         fprintf(gnuplotPipe2, "e\n");

        // Send actual theta_m data
         for (size_t i = 0; i < lreal.size(); i++) {
             fprintf(gnuplotPipe2, "%lu %f\n", i, lreal[i]);
         }
         fprintf(gnuplotPipe2, "e\n");
        fflush(gnuplotPipe2);
        pclose(gnuplotPipe2);
    } else {
        ROS_ERROR("Could not open Gnuplot.");
    }
    
    FILE *gnuplotPipe3 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe3) {
        fprintf(gnuplotPipe3, "set title 'angular velocity'\n");
        fprintf(gnuplotPipe3, "set xlabel 'Time Step'\n");
        fprintf(gnuplotPipe3, "set ylabel 'w'\n");
        fprintf(gnuplotPipe3, "plot '-' with lines title 'Desired W_r' lc rgb 'green', '-' with lines title 'Actual W_r' lc rgb 'purple'\n");
        
        // angular velocity data
        for (size_t i = 0; i < aref.size(); i++) {
            fprintf(gnuplotPipe3, "%lu %f\n", i, aref[i]);
        }
         fprintf(gnuplotPipe3, "e\n");

        // Send actual theta_m data
         for (size_t i = 0; i < areal.size(); i++) {
             fprintf(gnuplotPipe3, "%lu %f\n", i, areal[i]);
         }
         fprintf(gnuplotPipe3, "e\n");
        fflush(gnuplotPipe3);
        pclose(gnuplotPipe3);
    } else {
        ROS_ERROR("Could not open Gnuplot.");
    }

    FILE *gnuplotPipe4 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe4) {
         fprintf(gnuplotPipe4, "set title 'Desired vs Actual Orientation (Theta)'\n");
         fprintf(gnuplotPipe4, "set xlabel 'Time Step'\n");
         fprintf(gnuplotPipe4, "set ylabel 'Theta (rad)'\n");
         fprintf(gnuplotPipe4, "plot '-' with lines title 'Desired Theta (Theta_rr)' lc rgb 'green', '-' with lines title 'Actual Theta (Theta_mm)' lc rgb 'purple'\n");

         // Send desired theta_r data
         for (size_t i = 0; i < theta_rr.size(); i++) {
             fprintf(gnuplotPipe4, "%lu %f\n", i, theta_rr[i]);
         }
         fprintf(gnuplotPipe4, "e\n");

         // Send actual theta_m data
         for (size_t i = 0; i < theta_mm.size(); i++) {
             fprintf(gnuplotPipe4, "%lu %f\n", i, theta_mm[i]);
         }
         fprintf(gnuplotPipe4, "e\n");

        fflush(gnuplotPipe4);
        pclose(gnuplotPipe4);
    } else {
        ROS_ERROR("Could not open Gnuplot.");
    }

    return 0;
}

