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
double v_r = 0.25;    // Desired linear velocity (m/s)
double w_r = 0.5;     // Desired angular velocity (rad/s)
double theta_r = 0.0; // Desired orientation (rad)
double e_x = 0.0, e_y = 0.0, e_theta = 0.0;
double x_ref, y_ref;
double x_m, y_m, theta_m;
double current_time, v_c, w_c, v_m, w_m, ev, ew, v_d, w_d;
double desired_angle;

// Variables for ADRC control
double last_ev = 0.0, integral_ev = 0.0;
double last_ew = 0.0, integral_ew = 0.0;
double x1v = 0.0, x2v = 0.0, x1w = 0.0, x2w = 0.0;
double uv_o = 0.0, uw_o = 0.0;

// Vectors to log trajectory data
vector<double> actual_x, actual_y, desired_x, desired_y;
vector<double> lref, lreal, aref, areal;
vector<double> theta_rr, theta_mm;

// Callback to update current position
void positionCallback(const nav_msgs::Odometry &msg)
{
    current_position = msg;
    odom_received = true;
}
// paramters of MF for input E
// MF1 NL
double a11 = -1, b11 = -0.66, c11 = -0.33;
// MF2 NS
double a12 = -0.66, b12 = -0.33, c12 = 0;
// MF3 Z
double a13 = -0.33, b13 = 0, c13 = 0.33;
// MF4 PS
double a14 = 0, b14 = 0.33, c14 = 0.66;
// MF5 PL
double a15 = 0.33, b15 = 0.66, c15 = 1;
//==========================
// paramters of MF for input CE
// MF1 NL
double a21 = -1, b21 = -0.66, c21 = -0.33;
// MF2 NS
double a22 = -0.66, b22 = -0.33, c22 = 0;
// MF3 Z
double a23 = -0.33, b23 = 0, c23 = 0.33;
// MF4 PS
double a24 = 0, b24 = 0.33, c24 = 0.66;
// MF5 PL
double a25 = 0.33, b25 = 0.66, c25 = 1;

double NLE = 0, NSE = 0, ZE = 0, PSE = 0, PLE = 0;
double NLCE = 0, NSCE = 0, ZCE = 0, PSCE = 0, PLCE = 0;

void fuzzification_e(double e)
{
    // fuzzification of E
    // MF1: NLE
    if (e >= a11 && e <= b11)
    {
        NLE = (a11 - e) / (a11 - b11);
    }
    else if (e >= b11 && e <= c11)
    {
        NLE = (c11 - e) / (c11 - b11);
    }
    else
    {
        NLE = 0;
    }

    //================
    // MF2: NSE
    if (e >= a12 && e <= b12)
    {
        NSE = (a12 - e) / (a12 - b12);
    }
    else if (e >= b12 && e <= c12)
    {
        NSE = (c12 - e) / (c12 - b12);
    }
    else
    {
        NSE = 0;
    }
    //==============
    // MF3: ZE
    if (e >= a13 && e <= b13)
    {
        ZE = (a13 - e) / (a13 - b13);
    }
    else if (e >= b13 && e <= c13)
    {
        ZE = (c13 - e) / (c13 - b13);
    }
    else
    {
        ZE = 0;
    }
    //===============
    // MF4: PSE
    if (e >= a14 && e <= b14)
    {
        PSE = (a14 - e) / (a14 - b14);
    }
    else if (e >= b14 && e <= c14)
    {
        PSE = (c14 - e) / (c14 - b14);
    }
    else
    {
        PSE = 0;
    }
    //===============
    // MF5: PLE
    if (e >= a15 && e <= b15)
    {
        PLE = (a15 - e) / (a15 - b15);
    }
    else if (e >= b15 && e <= c15)
    {
        PLE = (c15 - e) / (c15 - b15);
    }
    else
    {
        PLE = 0;
    }
}

void fuzzification_ce(double ce)
{
    // fuzzification of CE
    // MF1: NLCE
    if (ce >= a21 && ce <= b21)
    {
        NLCE = (a21 - ce) / (a21 - b21);
    }
    else if (ce >= b21 && ce <= c21)
    {
        NLCE = (c21 - ce) / (c21 - b21);
    }
    else
    {
        NLCE = 0;
    }
    //================
    // MF2: NSCE
    if (ce >= a22 && ce <= b22)
    {
        NSCE = (a22 - ce) / (a22 - b22);
    }
    else if (ce >= b22 && ce <= c22)
    {
        NSCE = (c22 - ce) / (c22 - b22);
    }
    else
    {
        NSCE = 0;
    }
    //===============
    // MF3: ZCE
    if (ce >= a23 && ce <= b23)
    {
        ZCE = (a23 - ce) / (a23 - b23);
    }
    else if (ce >= b23 && ce <= c23)
    {
        ZCE = (c23 - ce) / (c23 - b23);
    }
    else
    {
        ZCE = 0;
    }
    //===============
    // MF4: PSCE
    if (ce >= a24 && ce <= b24)
    {
        PSCE = (a24 - ce) / (a24 - b24);
    }
    else if (ce >= b24 && ce <= c24)
    {
        PSCE = (c24 - ce) / (c24 - b24);
    }
    else
    {
        PSCE = 0;
    }
    //===============
    // MF5: PLCE
    if (ce >= a25 && ce <= b25)
    {
        PLCE = (a25 - ce) / (a25 - b25);
    }
    else if (ce >= b25 && ce <= c25)
    {
        PLCE = (c25 - ce) / (c25 - b25);
    }
    else
    {
        PLCE = 0;
    }
}

double G_e = 0.7, G_ce = 0.7, G_u = 0.1, wo = 0.5;

double FUZZY1V(double error, double last_error, double u, double y)
{
    double derivative_e = error - last_error;
    double e = error * G_e;
    double ce = derivative_e * G_ce;

    fuzzification_e(e);
    fuzzification_ce(ce);
    //========================================
    // Rules
    double r1 = min(PLE, NLCE);
    double r2 = min(PLE, NSCE);
    double r3 = min(PLE, ZCE);
    double r4 = min(PLE, PSCE);
    double r5 = min(PLE, PLCE);
    double r6 = min(PSE, NLCE);
    double r7 = min(PSE, NSCE);
    double r8 = min(PSE, ZCE);
    double r9 = min(PSE, PSCE);
    double r10 = min(PSE, PLCE);
    double r11 = min(ZE, NLCE);
    double r12 = min(ZE, NSCE);
    double r13 = min(ZE, ZCE);
    double r14 = min(ZE, PSCE);
    double r15 = min(ZE, PLCE);
    double r16 = min(NSE, NLCE);
    double r17 = min(NSE, NSCE);
    double r18 = min(NSE, ZCE);
    double r19 = min(NSE, PSCE);
    double r20 = min(NSE, PLCE);
    double r21 = min(NLE, NLCE);
    double r22 = min(NLE, NSCE);
    double r23 = min(NLE, ZCE);
    double r24 = min(NLE, PSCE);
    double r25 = min(NLE, PLCE);

    double h1 = max(max(r3, r4), r5), h2 = max(max(r9, r10), r15);
    double PL = max(h1, h2);
    //=========
    double h3 = max(max(r2, r8), max(r14, r20));
    double PS = h3;
    //========
    double h4 = max(max(r1, r7), r13);
    double h5 = max(r19, r25);
    double Z = max(h4, h5);
    //=========
    double h6 = max(max(r6, r12), max(r18, r24));
    double NS = h6;
    //=========
    double h7 = max(max(r11, r16), r17);
    double h8 = max(max(r21, r22), r23);
    double NL = max(h7, h8);
    //=======================================
    // defuzzification
    double num = (NL * (-0.601)) + (NS * (-0.2411)) + (Z * 0) + (PS * 0.33) + (PL * 0.66);
    double den = NL + NS + Z + PS + PL;
    double yV = 0.0;
    if (den == 0)
    {
        yV = 0;
    }
    else
    {
        yV = num / den;
    }
    // double f_g = yV * 0.01;
    // uv_o = uv_o + f_g;

    uv_o = yV * G_u;

    double bo = 12;
    double L1 = 2 * wo;
    double L2 = wo * wo;

    double e1 = x1v - y;
    double dx1 = x2v + (bo * u) - (L1 * e1);
    x1v = x1v + dx1;

    double dx2 = -(L2 * e1);
    x2v = x2v + dx2;
    return ((uv_o - x2v) * 0.0833);
}
double FUZZY1W(double error, double last_error, double u, double y)
{
    double derivative_e = error - last_error;
    double e = error * G_e;
    double ce = derivative_e * G_ce;

    fuzzification_e(e);
    fuzzification_ce(ce);
    //========================================
    // Rules
    double r1 = min(PLE, NLCE);
    double r2 = min(PLE, NSCE);
    double r3 = min(PLE, ZCE);
    double r4 = min(PLE, PSCE);
    double r5 = min(PLE, PLCE);
    double r6 = min(PSE, NLCE);
    double r7 = min(PSE, NSCE);
    double r8 = min(PSE, ZCE);
    double r9 = min(PSE, PSCE);
    double r10 = min(PSE, PLCE);
    double r11 = min(ZE, NLCE);
    double r12 = min(ZE, NSCE);
    double r13 = min(ZE, ZCE);
    double r14 = min(ZE, PSCE);
    double r15 = min(ZE, PLCE);
    double r16 = min(NSE, NLCE);
    double r17 = min(NSE, NSCE);
    double r18 = min(NSE, ZCE);
    double r19 = min(NSE, PSCE);
    double r20 = min(NSE, PLCE);
    double r21 = min(NLE, NLCE);
    double r22 = min(NLE, NSCE);
    double r23 = min(NLE, ZCE);
    double r24 = min(NLE, PSCE);
    double r25 = min(NLE, PLCE);

    double h1 = max(max(r3, r4), r5), h2 = max(max(r9, r10), r15);
    double PL = max(h1, h2);
    //=========
    double h3 = max(max(r2, r8), max(r14, r20));
    double PS = h3;
    //========
    double h4 = max(max(r1, r7), r13);
    double h5 = max(r19, r25);
    double Z = max(h4, h5);
    //=========
    double h6 = max(max(r6, r12), max(r18, r24));
    double NS = h6;
    //=========
    double h7 = max(max(r11, r16), r17);
    double h8 = max(max(r21, r22), r23);
    double NL = max(h7, h8);
    //=======================================
    // defuzzification
    double num = (NL * (-0.601)) + (NS * (-0.2411)) + (Z * 0) + (PS * 0.33) + (PL * 0.66);
    double den = NL + NS + Z + PS + PL;
    double yW = 0.0;
    if (den == 0)
    {
        yW = 0;
    }
    else
    {
        yW = num / den;
    }
    // double f_g = yW * 0.008;
    // uw_o = uw_o + f_g;
    uw_o = yW * G_u;

    double bo = 12;
    double L1 = 2 * wo;
    double L2 = wo * wo;

    double e1 = x1w - y;
    double dx1 = x2w + (bo * u) - (L1 * e1);
    x1w = x1w + dx1;

    double dx2 = -(L2 * e1);
    x2w = x2w + dx2;
    return ((uw_o - x2w) * 0.0833);
}

// Main control function
void followCircularTrajectory()
{
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(1000); // Control loop frequency (Hz)
    double start_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        if (!odom_received)
        {
            ros::spinOnce();
            continue;
        }

        // Calculate elapsed time
        current_time = ros::Time::now().toSec() - start_time;
        theta_r = theta_r + (w_r * 0.001);
        x_ref = (v_r / w_r) * cos(theta_r);
        y_ref = (v_r / w_r) * sin(theta_r);

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

        e_y = -((x_ref - x_m) * sin(theta_m)) + ((y_ref - y_m) * cos(theta_m));

        desired_angle = atan2(y_ref - y_m, x_ref - x_m);
        theta_rr.push_back(desired_angle);
        e_theta = angles::shortest_angular_distance(theta_m, desired_angle);

        v_d = (0.2 * e_x) + (v_r * cos(e_theta));
        w_d = (0.5 * v_r * e_y) + w_r + (0.8 * sin(e_theta));

        v_m = current_position.twist.twist.linear.x;
        /*         if(current_time > 16){

                    // v_m=v_m+0.00945*sin(current_time);
                    v_m=v_m + 0.05;

                } */

        ev = v_d - v_m;
        v_c = FUZZY1V(ev, last_ev, v_c, v_m);
        last_ev = ev;

        w_m = current_position.twist.twist.angular.z;
        /*         if(current_time > 16){

                   //  w_m=w_m+0.0094*sin(current_time);
                   w_m=w_m + 0.05;

                } */
        ew = w_d - w_m;
        w_c = FUZZY1W(ew, last_ew, w_c, w_m);
        last_ew = ew;

        // distubrance in control signal for comparison

        // Set velocity commands
        cmd_vel.linear.x = v_c;
        cmd_vel.angular.z = w_c;

        // Publish velocity commands
        pub.publish(cmd_vel);

        // Log velocities
        lref.push_back(v_c);
        lreal.push_back(current_position.twist.twist.linear.x);
        aref.push_back(w_c);
        areal.push_back(current_position.twist.twist.angular.z);
        // cout<<"e_x: "<<e_x<<" e_y: "<<e_y<<" l: "<<v_d<<" a: "<<w_d<<" t: "<<e_theta<<endl;
        // Stop if within tolerance
        if (/*ev < tolerance ||*/ current_time > 12)
        {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_trajectory_controller");
    ros::NodeHandle nh1, nh2, nh;

    // Subscriber and Publisher
    ros::Subscriber sub = nh.subscribe("/pioneer/odom", 10, positionCallback);
    pub = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 10);
    //ros::Subscriber sub=nh2.subscribe("/RosAria/pose",10,&positionCallback);
    //pub =nh1.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1000);
    //  Wait for odometry data
    ROS_INFO("Waiting for odometry data...");
    ros::Rate wait_rate(10);
    while (ros::ok() && !odom_received)
    {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Odometry data received. Starting circular trajectory.");

    // Start following the trajectory
    followCircularTrajectory();

    // Plot the trajectory using Gnuplot
    FILE *gnuplotPipe = popen("gnuplot -persistent", "w");
    if (gnuplotPipe)
    {
        fprintf(gnuplotPipe, "set title 'Desired vs Actual Trajectory'\n");
        fprintf(gnuplotPipe, "set xlabel 'X Position (m)'\n");
        fprintf(gnuplotPipe, "set ylabel 'Y Position (m)'\n");
        fprintf(gnuplotPipe, "plot '-' with lines title 'Desired Trajectory' lc rgb 'blue', '-' with lines title 'Actual Trajectory' lc rgb 'red'\n");

        // Send desired trajectory data
        for (size_t i = 0; i < desired_x.size(); i++)
        {
            fprintf(gnuplotPipe, "%f %f\n", desired_x[i], desired_y[i]);
        }
        fprintf(gnuplotPipe, "e\n");

        // Send actual trajectory data
        for (size_t i = 0; i < actual_x.size(); i++)
        {
            fprintf(gnuplotPipe, "%f %f\n", actual_x[i], actual_y[i]);
        }
        fprintf(gnuplotPipe, "e\n");
        fflush(gnuplotPipe);
        pclose(gnuplotPipe);
    }
    else
    {
        ROS_ERROR("Could not open Gnuplot.");
    }

    FILE *gnuplotPipe2 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe2)
    {
        fprintf(gnuplotPipe2, "set title 'linear velocity'\n");
        fprintf(gnuplotPipe2, "set xlabel 'Time Step'\n");
        fprintf(gnuplotPipe2, "set ylabel 'v'\n");
        fprintf(gnuplotPipe2, "plot '-' with lines title 'Desired V_r' lc rgb 'green', '-' with lines title 'Actual V_r' lc rgb 'purple'\n");

        // Linear velocity data
        for (size_t i = 0; i < lref.size(); i++)
        {
            fprintf(gnuplotPipe2, "%lu %f\n", i, lref[i]);
        }
        fprintf(gnuplotPipe2, "e\n");

        // Send actual theta_m data
        for (size_t i = 0; i < lreal.size(); i++)
        {
            fprintf(gnuplotPipe2, "%lu %f\n", i, lreal[i]);
        }
        fprintf(gnuplotPipe2, "e\n");
        fflush(gnuplotPipe2);
        pclose(gnuplotPipe2);
    }
    else
    {
        ROS_ERROR("Could not open Gnuplot.");
    }

    FILE *gnuplotPipe3 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe3)
    {
        fprintf(gnuplotPipe3, "set title 'angular velocity'\n");
        fprintf(gnuplotPipe3, "set xlabel 'Time Step'\n");
        fprintf(gnuplotPipe3, "set ylabel 'w'\n");
        fprintf(gnuplotPipe3, "plot '-' with lines title 'Desired W_r' lc rgb 'green', '-' with lines title 'Actual W_r' lc rgb 'purple'\n");

        // angular velocity data
        for (size_t i = 0; i < aref.size(); i++)
        {
            fprintf(gnuplotPipe3, "%lu %f\n", i, aref[i]);
        }
        fprintf(gnuplotPipe3, "e\n");

        // Send actual theta_m data
        for (size_t i = 0; i < areal.size(); i++)
        {
            fprintf(gnuplotPipe3, "%lu %f\n", i, areal[i]);
        }
        fprintf(gnuplotPipe3, "e\n");
        fflush(gnuplotPipe3);
        pclose(gnuplotPipe3);
    }
    else
    {
        ROS_ERROR("Could not open Gnuplot.");
    }

    FILE *gnuplotPipe4 = popen("gnuplot -persistent", "w");
    if (gnuplotPipe4)
    {
        fprintf(gnuplotPipe4, "set title 'Desired vs Actual Orientation (Theta)'\n");
        fprintf(gnuplotPipe4, "set xlabel 'Time Step'\n");
        fprintf(gnuplotPipe4, "set ylabel 'Theta (rad)'\n");
        fprintf(gnuplotPipe4, "plot '-' with lines title 'Desired Theta (Theta_rr)' lc rgb 'green', '-' with lines title 'Actual Theta (Theta_mm)' lc rgb 'purple'\n");

        // Send desired theta_r data
        for (size_t i = 0; i < theta_rr.size(); i++)
        {
            fprintf(gnuplotPipe4, "%lu %f\n", i, theta_rr[i]);
        }
        fprintf(gnuplotPipe4, "e\n");

        // Send actual theta_m data
        for (size_t i = 0; i < theta_mm.size(); i++)
        {
            fprintf(gnuplotPipe4, "%lu %f\n", i, theta_mm[i]);
        }
        fprintf(gnuplotPipe4, "e\n");

        fflush(gnuplotPipe4);
        pclose(gnuplotPipe4);
    }
    else
    {
        ROS_ERROR("Could not open Gnuplot.");
    }

    return 0;
}
