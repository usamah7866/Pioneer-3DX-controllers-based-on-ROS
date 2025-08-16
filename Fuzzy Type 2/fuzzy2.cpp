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
double current_time, v_d, w_d, v_m, w_m, ev, ew, v_dd, w_dd;
double desired_angle;

// Variables for ADRC control
double last_ev = 0.0, integral_ev = 0.0;
double last_ew = 0.0, integral_ew = 0.0;
double x1v = 0.0, x2v = 0.0, x1w = 0.0, x2w = 0.0;
double uv_o =0.0, uw_o=0.0;

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

// MF1 NL
double m11u = 0.1, m11l = 0.1;
double a11u = -1 - m11u, b11u = -0.66, c11u = -0.33 + m11u;
double a11l = -1 + m11l, b11l = -0.66, c11l = -0.33 - m11l;
// MF2 NS
double m12u = 0.1, m12l = 0.1;
double a12u = -0.66 - m12u, b12u = -0.33, c12u = 0.0005;
double a12l = -0.66 + m12l, b12l = -0.33, c12l = 0.0001;
// MF3 Z
double m13u = 0.1, m13l = 0.1;
double a13u = -0.33 - m13u, b13u = 0, c13u = 0.33 + m13u;
double a13l = -0.33 + m13l, b13l = 0, c13l = 0.33 - m13l;
// MF4 PS
double m14u = 0.1, m14l = 0.1;
double a14u = 0, b14u = 0.33, c14u = 0.66 + m14u;
double a14l = 0, b14l = 0.33, c14l = 0.66 - m14l;
// MF5 PL
double m15u = 0.1, m15l = 0.1;
double a15u = 0.33 - m15u, b15u = 0.66, c15u = 1 + m15u;
double a15l = 0.33 + m15l, b15l = 0.66, c15l = 1 - m15l;
// ==========================

// paramters of MF for input CE
// MF1 NL
double m21u = 0.1, m21l = 0.1;
double a21u = -1 - m21u, b21u = -0.66, c21u = -0.33 + m21u;
double a21l = -1 + m21l, b21l = -0.66, c21l = -0.33 - m21l;
// MF2 NS
double m22u = 0.1, m22l = 0.1;
double a22u = -0.66 - m22u, b22u = -0.33, c22u = 0.0005;
double a22l = -0.66 + m22l, b22l = -0.33, c22l = 0.0001;
// MF3 Z
double m23u = 0.1, m23l = 0.1;
double a23u = -0.33 - m23u, b23u = 0, c23u = 0.33 + m23u;
double a23l = -0.33 + m23l, b23l = 0, c23l = 0.33 - m23l;
// MF4 PS
double m24u = 0.1, m24l = 0.1;
double a24u = 0, b24u = 0.33, c24u = 0.66 + m24u;
double a24l = 0, b24l = 0.33, c24l = 0.66 - m24l;
// MF5 PL
double m25u = 0.1, m25l = 0.1;
double a25u = 0.33 - m25u, b25u = 0.66, c25u = 1 + m25u;
double a25l = 0.33 + m25l, b25l = 0.66, c25l = 1 - m25l;

double NLEU = 0, NSEU = 0, ZEU = 0, PSEU = 0, PLEU = 0;
double NLEL = 0, NSEL = 0, ZEL = 0, PSEL = 0, PLEL = 0;

double NLCEU = 0, NSCEU = 0, ZCEU = 0, PSCEU = 0, PLCEU = 0;
double NLCEL = 0, NSCEL = 0, ZCEL = 0, PSCEL = 0, PLCEL = 0;

double FUZZY2V(double error, double last_error, double u, double y)
{
    double derivative_e = error - last_error;
    double e = error * 1.0;
    double ce = derivative_e * 1;

        // fuzzification of E
    //  MF1: NLEU
    if (e >= a11u && e <= b11u)
    {
        NLEU = (a11u - e) / (a11u - b11u);
    }
    else if (e >= b11u && e <= c11u)
    {
        NLEU = (c11u - e) / (c11u - b11u);
    }
    else
    {
        NLEU = 0;
    }

    // MF1: NLEL
    if (e >= a11l && e <= b11l)
    {
        NLEL = (a11l - e) / (a11l - b11l);
    }
    else if (e >= b11l && e <= c11l)
    {
        NLEL = (c11l - e) / (c11l - b11l);
    }
    else
    {
        NLEL = 0;
    }

    // ================
    // MF2: NSEU
    if (e >= a12u && e <= b12u)
    {
        NSEU = (a12u - e) / (a12u - b12u);
    }
    else if (e >= b12u && e <= c12u)
    {
        NSEU = (c12u - e) / (c12u - b12u);
    }
    else
    {
        NSEU = 0;
    }

    // MF2: NSEL
    if (e >= a12l && e <= b12l)
    {
        NSEL = (a12l - e) / (a12l - b12l);
    }
    else if (e >= b12l && e <= c12l)
    {
        NSEL = (c12l - e) / (c12l - b12l);
    }
    else
    {
        NSEL = 0;
    }

    //===============
    // MF3: ZEU
    if (e >= a13u && e <= b13u)
    {
        ZEU = (a13u - e) / (a13u - b13u);
    }
    else if (e >= b13u && e <= c13u)
    {
        ZEU = (c13u - e) / (c13u - b13u);
    }
    else
    {
        ZEU = 0;
    }

    // MF3: ZEL
    if (e >= a13l && e <= b13l)
    {
        ZEL = (a13l - e) / (a13l - b13l);
    }
    else if (e >= b13l && e <= c13l)
    {
        ZEL = (c13l - e) / (c13l - b13l);
    }
    else
    {
        ZEL = 0;
    }

    // ===============
    // MF4: PSEU
    if (e >= a14u && e <= b14u)
    {
        PSEU = (a14u - e) / (a14u - b14u);
    }
    else if (e >= b14u && e <= c14u)
    {
        PSEU = (c14u - e) / (c14u - b14u);
    }
    else
    {
        PSEU = 0;
    }

    // MF4: PSEL
    if (e >= a14u && e <= b14u)
    {
        PSEL = (a14u - e) / (a14u - b14u);
    }
    else if (e >= b14u && e <= c14u)
    {
        PSEL = (c14u - e) / (c14u - b14u);
    }
    else
    {
        PSEL = 0;
    }

    // ===============
    // MF5: PLEU
    if (e >= a15u && e <= b15u)
    {
        PLEU = (a15u - e) / (a15u - b15u);
    }
    else if (e >= b15u && e <= c15u)
    {
        PLEU = (c15u - e) / (c15u - b15u);
    }
    else
    {
        PLEU = 0;
    }

    // MF5: PLE
    if (ce >= a15u && e <= b15u)
    {
        PLEL = (a15u - e) / (a15u - b15u);
    }
    else if (e >= b15u && e <= c15u)
    {
        PLEL = (c15u - e) / (c15u - b15u);
    }
    else
    {
        PLEL = 0;
    }

    //========================================
    // fuzzification of CE
    // MF1: NLCEU
    if (e >= a21u && ce <= b21u)
    {
        NLCEU = (a21u - ce) / (a21u - b21u);
    }
    else if (ce >= b21u && ce <= c21u)
    {
        NLCEU = (c21u - ce) / (c21u - b21u);
    }
    else
    {
        NLCEU = 0;
    }

    //     MF1: NLCEL
    if (ce >= a21l && ce <= b21l)
    {
        NLCEL = (a21l - ce) / (a21l - b21l);
    }
    else if (ce >= b21l && ce <= c21l)
    {
        NLCEL = (c21l - ce) / (c21l - b21l);
    }
    else
    {
        NLCEL = 0;
    }

    // ================
    // MF2: NSCEU
    if (ce >= a22u && ce <= b22u)
    {
        NSCEU = (a22u - ce) / (a22u - b22u);
    }
    else if (ce >= b22u && ce <= c22u)
    {
        NSCEU = (c22u - ce) / (c22u - b22u);
    }
    else
    {
        NSCEU = 0;
    }

    // MF2: NSCEL
    if (ce >= a22l && ce <= b22l)
    {
        NSCEL = (a22l - ce) / (a22l - b22l);
    }
    else if (ce >= b22l && ce <= c22l)
    {
        NSCEL = (c22l - ce) / (c22l - b22l);
    }
    else
    {
        NSCEL = 0;
    }

    //===============
    // MF3: ZCEU
    if (ce >= a23u && ce <= b23u)
    {
        ZCEU = (a23u - ce) / (a23u - b23u);
    }
    else if (ce >= b23u && ce <= c23u)
    {
        ZCEU = (c23u - ce) / (c23u - b23u);
    }
    else
    {
        ZCEU = 0;
    }

    // MF3: ZCEL
    if (ce >= a23l && ce <= b23l)
    {
        ZCEL = (a23l - ce) / (a23l - b23l);
    }
    else if (ce >= b23l && ce <= c23l)
    {
        ZCEL = (c23l - ce) / (c23l - b23l);
    }
    else
    {
        ZCEL = 0;
    }

    // ===============
    // MF4: PSCEU
    if (ce >= a24u && ce <= b24u)
    {
        PSCEU = (a24u - ce) / (a24u - b24u);
    }
    else if (ce >= b24u && ce <= c24u)
    {
        PSCEU = (c24u - ce) / (c24u - b24u);
    }
    else
    {
        PSCEU = 0;
    }

    // MF4: PSCEL
    if (ce >= a24l && ce <= b24l)
    {
        PSCEL = (a24l - ce) / (a24l - b24l);
    }
    else if (ce >= b24l && ce <= c24l)
    {
        PSCEL = (c24l - ce) / (c24l - b24l);
    }
    else
    {
        PSCEL = 0;
    }

    // ===============
    // MF5: PLCEU
    if (ce >= a25u && ce <= b25u)
    {
        PLCEU = (a25u - ce) / (a25u - b25u);
    }
    else if (ce >= b25u && ce <= c25u)
    {
        PLCEU = (c25u - ce) / (c25u - b25u);
    }
    else
    {
        PLCEU = 0;
    }

    // MF5: PLCEL
    if (ce >= a25l && ce <= b25l)
    {
        PLCEL = (a25l - ce) / (a25l - b25l);
    }
    else if (ce >= b25l && ce <= c25l)
    {
        PLCEL = (c25l - ce) / (c25l - b25l);
    }
    else
    {
        PLCEL = 0;
    }

    double r1u = min(PLEU, NLCEU), r1l = min(PLEL, NLCEL);
    double r2u = min(PLEU, NSCEU), r2l = min(PLEL, NSCEL);
    double r3u = min(PLEU, ZCEU), r3l = min(PLEL, ZCEL);
    double r4u = min(PLEU, PSCEU), r4l = min(PLEL, PSCEL);
    double r5u = min(PLEU, PLCEU), r5l = min(PLEL, PLCEL);
    double r6u = min(PSEU, NLCEU), r6l = min(PSEL, NLCEL);
    double r7u = min(PSEU, NSCEU), r7l = min(PSEL, NSCEL);
    double r8u = min(PSEU, ZCEU), r8l = min(PSEL, ZCEL);
    double r9u = min(PSEU, PSCEU), r9l = min(PSEL, PSCEL);
    double r10u = min(PSEU, PLCEU), r10l = min(PSEL, PLCEL);
    double r11u = min(ZEU, NLCEU), r11l = min(ZEL, NLCEL);
    double r12u = min(ZEU, NSCEU), r12l = min(ZEL, NSCEL);
    double r13u = min(ZEU, ZCEU), r13l = min(ZEL, ZCEL);
    double r14u = min(ZEU, PSCEU), r14l = min(ZEL, PSCEL);
    double r15u = min(ZEU, PLCEU), r15l = min(ZEL, PLCEL);
    double r16u = min(NSEU, NLCEU), r16l = min(NSEL, NLCEL);
    double r17u = min(NSEU, NSCEU), r17l = min(NSEL, NSCEL);
    double r18u = min(NSEU, ZCEU), r18l = min(NSEL, ZCEL);
    double r19u = min(NSEU, PSCEU), r19l = min(NSEL, PSCEL);
    double r20u = min(NSEU, PLCEU), r20l = min(NSEL, PLCEL);
    double r21u = min(NLEU, NLCEU), r21l = min(NLEL, NLCEL);
    double r22u = min(NLEU, NSCEU), r22l = min(NLEL, NSCEL);
    double r23u = min(NLEU, ZCEU), r23l = min(NLEL, ZCEL);
    double r24u = min(NLEU, PSCEU), r24l = min(NLEL, PSCEL);
    double r25u = min(NLEU, PLCEU), r25l = min(NLEL, PLCEL);

    double h1u = max(max(r3u, r4u), r5u), h2u = max(max(r9u, r10u), r15u);
    double PLU = max(h1u, h2u);
    double h1l = max(max(r3l, r4l), r5l), h2l = max(max(r9l, r10l), r15l);
    double PLL = max(h1l, h2l);
    // =========
    double h3u = max(max(r2u, r8u), max(r14u, r20u));
    double PSU = h3u;
    double h3l = max(max(r2l, r8l), max(r14l, r20l));
    double PSL = h3l;
    // ========
    double h4u = max(max(r1u, r7u), r13u), h5u = max(r19u, r25u);
    double ZU = max(h4u, h5u);
    double h4l = max(max(r1l, r7l), r13l), h5l = max(r19l, r25l);
    double ZL = max(h4l, h5l);
    // =========
    double h6u = max(max(r6u, r12u), max(r18u, r24u));
    double NSU = h6u;
    double h6l = max(max(r6l, r12l), max(r18l, r24l));
    double NSL = h6l;
    // =========
    double h7u = max(max(r11u, r16u), r17u), h8u = max(max(r21u, r22u), r23u);
    double NLU = max(h7u, h8u);
    double h7l = max(max(r11l, r16l), r17l), h8l = max(max(r21l, r22l), r23l);
    double NLL = max(h7l, h8l);

    double num = (NLL * (-0.7683)) + (NSL * (-0.4189)) + (ZL * 0.0) + (PSL * 0.2515) + (PLL * 0.5740);
    double den = NLL + NSL + ZL + PSL + PLL;
    double yLV= 0.0, yRV=0.0;
    if (den == 0)
    {
        yLV = 0;
    }
    else
    {
        yLV = num / den;
    }

    num = (NLU * (-0.7683)) + (NSU * (-0.4189)) + (ZU * 0.0) + (PSU * 0.2515) + (PLU * 0.5740);
    den = NLU + NSU + ZU + PSU + PLU;
    if (den == 0)
    {
        yRV = 0;
    }
    else
    {
        yRV = num / den;
    }

    double f = (yLV + yRV) / 2;
    double f_g = f * 0.01;
    uv_o = uv_o + f_g;
    return uv_o;

/*     double wo = 0.01;
    double bo = 12;
    double L1 = 2 * wo;
    double L2 = wo * wo;

    double e1 = x1v - y;
    double dx1 = x2v + (bo * u) - (L1 * e1);
    x1v = x1v + dx1;

    double dx2 = -(L2 * e1);
    x2v = x2v + dx2;
    return ((uv_o - x2v) * 0.0833); */
}
double FUZZY2W(double error, double last_error, double u, double y)
{
    double derivative_e = error - last_error;
    double e = error * 1.0;
    double ce = derivative_e * 1;

        // fuzzification of E
    //  MF1: NLEU
    if (e >= a11u && e <= b11u)
    {
        NLEU = (a11u - e) / (a11u - b11u);
    }
    else if (e >= b11u && e <= c11u)
    {
        NLEU = (c11u - e) / (c11u - b11u);
    }
    else
    {
        NLEU = 0;
    }

    // MF1: NLEL
    if (e >= a11l && e <= b11l)
    {
        NLEL = (a11l - e) / (a11l - b11l);
    }
    else if (e >= b11l && e <= c11l)
    {
        NLEL = (c11l - e) / (c11l - b11l);
    }
    else
    {
        NLEL = 0;
    }

    // ================
    // MF2: NSEU
    if (e >= a12u && e <= b12u)
    {
        NSEU = (a12u - e) / (a12u - b12u);
    }
    else if (e >= b12u && e <= c12u)
    {
        NSEU = (c12u - e) / (c12u - b12u);
    }
    else
    {
        NSEU = 0;
    }

    // MF2: NSEL
    if (e >= a12l && e <= b12l)
    {
        NSEL = (a12l - e) / (a12l - b12l);
    }
    else if (e >= b12l && e <= c12l)
    {
        NSEL = (c12l - e) / (c12l - b12l);
    }
    else
    {
        NSEL = 0;
    }

    //===============
    // MF3: ZEU
    if (e >= a13u && e <= b13u)
    {
        ZEU = (a13u - e) / (a13u - b13u);
    }
    else if (e >= b13u && e <= c13u)
    {
        ZEU = (c13u - e) / (c13u - b13u);
    }
    else
    {
        ZEU = 0;
    }

    // MF3: ZEL
    if (e >= a13l && e <= b13l)
    {
        ZEL = (a13l - e) / (a13l - b13l);
    }
    else if (e >= b13l && e <= c13l)
    {
        ZEL = (c13l - e) / (c13l - b13l);
    }
    else
    {
        ZEL = 0;
    }

    // ===============
    // MF4: PSEU
    if (e >= a14u && e <= b14u)
    {
        PSEU = (a14u - e) / (a14u - b14u);
    }
    else if (e >= b14u && e <= c14u)
    {
        PSEU = (c14u - e) / (c14u - b14u);
    }
    else
    {
        PSEU = 0;
    }

    // MF4: PSEL
    if (e >= a14u && e <= b14u)
    {
        PSEL = (a14u - e) / (a14u - b14u);
    }
    else if (e >= b14u && e <= c14u)
    {
        PSEL = (c14u - e) / (c14u - b14u);
    }
    else
    {
        PSEL = 0;
    }

    // ===============
    // MF5: PLEU
    if (e >= a15u && e <= b15u)
    {
        PLEU = (a15u - e) / (a15u - b15u);
    }
    else if (e >= b15u && e <= c15u)
    {
        PLEU = (c15u - e) / (c15u - b15u);
    }
    else
    {
        PLEU = 0;
    }

    // MF5: PLE
    if (ce >= a15u && e <= b15u)
    {
        PLEL = (a15u - e) / (a15u - b15u);
    }
    else if (e >= b15u && e <= c15u)
    {
        PLEL = (c15u - e) / (c15u - b15u);
    }
    else
    {
        PLEL = 0;
    }

    //========================================
    // fuzzification of CE
    // MF1: NLCEU
    if (e >= a21u && ce <= b21u)
    {
        NLCEU = (a21u - ce) / (a21u - b21u);
    }
    else if (ce >= b21u && ce <= c21u)
    {
        NLCEU = (c21u - ce) / (c21u - b21u);
    }
    else
    {
        NLCEU = 0;
    }

    //     MF1: NLCEL
    if (ce >= a21l && ce <= b21l)
    {
        NLCEL = (a21l - ce) / (a21l - b21l);
    }
    else if (ce >= b21l && ce <= c21l)
    {
        NLCEL = (c21l - ce) / (c21l - b21l);
    }
    else
    {
        NLCEL = 0;
    }

    // ================
    // MF2: NSCEU
    if (ce >= a22u && ce <= b22u)
    {
        NSCEU = (a22u - ce) / (a22u - b22u);
    }
    else if (ce >= b22u && ce <= c22u)
    {
        NSCEU = (c22u - ce) / (c22u - b22u);
    }
    else
    {
        NSCEU = 0;
    }

    // MF2: NSCEL
    if (ce >= a22l && ce <= b22l)
    {
        NSCEL = (a22l - ce) / (a22l - b22l);
    }
    else if (ce >= b22l && ce <= c22l)
    {
        NSCEL = (c22l - ce) / (c22l - b22l);
    }
    else
    {
        NSCEL = 0;
    }

    //===============
    // MF3: ZCEU
    if (ce >= a23u && ce <= b23u)
    {
        ZCEU = (a23u - ce) / (a23u - b23u);
    }
    else if (ce >= b23u && ce <= c23u)
    {
        ZCEU = (c23u - ce) / (c23u - b23u);
    }
    else
    {
        ZCEU = 0;
    }

    // MF3: ZCEL
    if (ce >= a23l && ce <= b23l)
    {
        ZCEL = (a23l - ce) / (a23l - b23l);
    }
    else if (ce >= b23l && ce <= c23l)
    {
        ZCEL = (c23l - ce) / (c23l - b23l);
    }
    else
    {
        ZCEL = 0;
    }

    // ===============
    // MF4: PSCEU
    if (ce >= a24u && ce <= b24u)
    {
        PSCEU = (a24u - ce) / (a24u - b24u);
    }
    else if (ce >= b24u && ce <= c24u)
    {
        PSCEU = (c24u - ce) / (c24u - b24u);
    }
    else
    {
        PSCEU = 0;
    }

    // MF4: PSCEL
    if (ce >= a24l && ce <= b24l)
    {
        PSCEL = (a24l - ce) / (a24l - b24l);
    }
    else if (ce >= b24l && ce <= c24l)
    {
        PSCEL = (c24l - ce) / (c24l - b24l);
    }
    else
    {
        PSCEL = 0;
    }

    // ===============
    // MF5: PLCEU
    if (ce >= a25u && ce <= b25u)
    {
        PLCEU = (a25u - ce) / (a25u - b25u);
    }
    else if (ce >= b25u && ce <= c25u)
    {
        PLCEU = (c25u - ce) / (c25u - b25u);
    }
    else
    {
        PLCEU = 0;
    }

    // MF5: PLCEL
    if (ce >= a25l && ce <= b25l)
    {
        PLCEL = (a25l - ce) / (a25l - b25l);
    }
    else if (ce >= b25l && ce <= c25l)
    {
        PLCEL = (c25l - ce) / (c25l - b25l);
    }
    else
    {
        PLCEL = 0;
    }

    double r1u = min(PLEU, NLCEU), r1l = min(PLEL, NLCEL);
    double r2u = min(PLEU, NSCEU), r2l = min(PLEL, NSCEL);
    double r3u = min(PLEU, ZCEU), r3l = min(PLEL, ZCEL);
    double r4u = min(PLEU, PSCEU), r4l = min(PLEL, PSCEL);
    double r5u = min(PLEU, PLCEU), r5l = min(PLEL, PLCEL);
    double r6u = min(PSEU, NLCEU), r6l = min(PSEL, NLCEL);
    double r7u = min(PSEU, NSCEU), r7l = min(PSEL, NSCEL);
    double r8u = min(PSEU, ZCEU), r8l = min(PSEL, ZCEL);
    double r9u = min(PSEU, PSCEU), r9l = min(PSEL, PSCEL);
    double r10u = min(PSEU, PLCEU), r10l = min(PSEL, PLCEL);
    double r11u = min(ZEU, NLCEU), r11l = min(ZEL, NLCEL);
    double r12u = min(ZEU, NSCEU), r12l = min(ZEL, NSCEL);
    double r13u = min(ZEU, ZCEU), r13l = min(ZEL, ZCEL);
    double r14u = min(ZEU, PSCEU), r14l = min(ZEL, PSCEL);
    double r15u = min(ZEU, PLCEU), r15l = min(ZEL, PLCEL);
    double r16u = min(NSEU, NLCEU), r16l = min(NSEL, NLCEL);
    double r17u = min(NSEU, NSCEU), r17l = min(NSEL, NSCEL);
    double r18u = min(NSEU, ZCEU), r18l = min(NSEL, ZCEL);
    double r19u = min(NSEU, PSCEU), r19l = min(NSEL, PSCEL);
    double r20u = min(NSEU, PLCEU), r20l = min(NSEL, PLCEL);
    double r21u = min(NLEU, NLCEU), r21l = min(NLEL, NLCEL);
    double r22u = min(NLEU, NSCEU), r22l = min(NLEL, NSCEL);
    double r23u = min(NLEU, ZCEU), r23l = min(NLEL, ZCEL);
    double r24u = min(NLEU, PSCEU), r24l = min(NLEL, PSCEL);
    double r25u = min(NLEU, PLCEU), r25l = min(NLEL, PLCEL);

    double h1u = max(max(r3u, r4u), r5u), h2u = max(max(r9u, r10u), r15u);
    double PLU = max(h1u, h2u);
    double h1l = max(max(r3l, r4l), r5l), h2l = max(max(r9l, r10l), r15l);
    double PLL = max(h1l, h2l);
    // =========
    double h3u = max(max(r2u, r8u), max(r14u, r20u));
    double PSU = h3u;
    double h3l = max(max(r2l, r8l), max(r14l, r20l));
    double PSL = h3l;
    // ========
    double h4u = max(max(r1u, r7u), r13u), h5u = max(r19u, r25u);
    double ZU = max(h4u, h5u);
    double h4l = max(max(r1l, r7l), r13l), h5l = max(r19l, r25l);
    double ZL = max(h4l, h5l);
    // =========
    double h6u = max(max(r6u, r12u), max(r18u, r24u));
    double NSU = h6u;
    double h6l = max(max(r6l, r12l), max(r18l, r24l));
    double NSL = h6l;
    // =========
    double h7u = max(max(r11u, r16u), r17u), h8u = max(max(r21u, r22u), r23u);
    double NLU = max(h7u, h8u);
    double h7l = max(max(r11l, r16l), r17l), h8l = max(max(r21l, r22l), r23l);
    double NLL = max(h7l, h8l);

    double num = (NLL * (-0.6011)) + (NSL * (-0.2410)) + (ZL * 0.0) + (PSL * 0.3948) + (PLL * 0.7666);
    double den = NLL + NSL + ZL + PSL + PLL;
    double yLW= 0.0, yRW=0.0;
    if (den == 0)
    {
        yLW = 0;
    }
    else
    {
        yLW = num / den;
    }

    num = (NLU * (-0.6011)) + (NSU * (-0.2410)) + (ZU * 0.0) + (PSU * 0.3948) + (PLU * 0.7666);
    den = NLU + NSU + ZU + PSU + PLU;
    if (den == 0)
    {
        yRW = 0;
    }
    else
    {
        yRW = num / den;
    }

    double f = (yLW + yRW) / 2;
    double f_g = f * 0.008;
    uw_o = uw_o + f_g;
    return uw_o;

/*     double wo= 0.01;
    double bo= 12;
    double L1 = 2 * wo;
    double L2 = wo * wo;

    double e1 = x1w - y ;
    double dx1 = x2w + (bo * u) - (L1 * e1);
    x1w = x1w + dx1;

    double dx2 = - (L2 * e1);
    x2w = x2w + dx2;
    return ((uw_o - x2w ) * 0.0833);  */
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
        e_theta = angles::shortest_angular_distance(theta_m, desired_angle);
        theta_rr.push_back(desired_angle);

        v_dd = (0.2 * e_x) + (v_r * cos(e_theta));
        w_dd = (0.5 * v_r * e_y) + w_r + (0.8 * sin(e_theta));

        v_m = current_position.twist.twist.linear.x;
        /*         if(current_time > 16){

                    // v_m=v_m+0.00945*sin(current_time);
                    v_m=v_m + 0.05;

                } */

        ev = v_dd - v_m;
        v_d = FUZZY2V(ev, last_ev, v_d, v_m);
        last_ev = ev;

        w_m = current_position.twist.twist.angular.z;
        /*         if(current_time > 16){

                   //  w_m=w_m+0.0094*sin(current_time);
                   w_m=w_m + 0.05;

                } */
        ew = w_dd - w_m;
        w_d = FUZZY2W(ew, last_ew, w_d, w_m);
        last_ew = ew;

        //distubrance in control signal for comparison



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
        if (/*ev < tolerance ||*/ current_time > 22)
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
    // ros::Subscriber sub=nh2.subscribe("/RosAria/pose",10,&positionCallback);
    // pub =nh1.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1000);
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
