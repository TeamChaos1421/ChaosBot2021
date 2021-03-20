//This file contains the formulas for kinematics using integration the Trapezoid Rule


#include "kinematics.h"
#include "math.h"
#include <cmath>
#include "frc/WPILib.h"

/*
The kinematic array is
kin[0]=current acceleration
kin[1]=previous acceleration
kin[2]=current velocity
kin[3]=previous velocity
kin[4]=angle (in degrees)
kin[5]=current x position
kin[6]=previous x position
kin[7]=current y position
kin[8]=previous y position
kin[9]=current time
kin[10]=previous time
*/
double kinematics(double a[])
{
double acc_cur = a[0];
double acc_pre = a[1];
double vel_cur = a[2];
double vel_pre = a[3];
double angle = a[4];
//double xpos_cur = a[5];
double xpos_pre = a[6];
//double ypos_cur = a[7];
double ypos_pre = a[8];
double samp_time_cur = a[9];
double samp_time_pre = a[10];
double temp_velocity;
double temp_x_position;
double temp_y_position;
double conv=3.1415927/180;


//Calculate current robot velocity
temp_velocity=.5 * (samp_time_cur-samp_time_pre) * (acc_cur+acc_pre)+vel_pre;

//Calculate current robot x position

temp_x_position = xpos_pre + ((samp_time_cur-samp_time_pre)*vel_cur -.25*(samp_time_cur-samp_time_pre)*(samp_time_cur-samp_time_pre) * (acc_cur+acc_pre))*cos(angle*conv);

 //Calculate current robot y position

temp_y_position = ypos_pre + ((samp_time_cur-samp_time_pre)*vel_cur -.25*(samp_time_cur-samp_time_pre)*(samp_time_cur-samp_time_pre) * (acc_cur+acc_pre))*sin(angle*conv);
//std::cout << "[" << " " << a[0] << " "<< a[1]<< " " << a[2]<< " "<< a[3]<<"]"<<"\n";
a[1]=a[0]; //current acceleration becomes previous acceleration for next iteration
a[3]=a[2]; //current velocity becomes previous velocity for next iteration
a[2]=temp_velocity; //current velocity
a[6]=a[5];
a[5]=temp_x_position; //current x position
a[8]=a[7];
a[7]=temp_y_position; //current y position
a[10]=a[9]; //current sample time becomes previous sample time

    //a[4]=3.14 * a[4];

//wpi::outs() << "accel, " << a[0] << "\n";
//wpi::outs() << "Y-Pos, " << a[7] << "\n";

return 0;
}

