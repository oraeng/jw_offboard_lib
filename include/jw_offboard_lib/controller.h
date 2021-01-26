#include <ros/ros.h>
#include <std_msgs/String.h> 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/AttitudeTarget.h"  //
#include "mavros_msgs/VFR_HUD.h"         // Airspeed
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
using boost::math::sign;

geometry_msgs::PoseStamped current_attitude;  // receive the attitude data from pixhawk(px4-based gazebo)
mavros_msgs::VFR_HUD current_speed;  // receive the current airspeed from pixhawk(px4-based gazebo)
mavros_msgs::AttitudeTarget attitudecommand; // to publish the attitude command to pixhawk(px4-based gazebo)
mavros_msgs::State current_status;

// 콜백함수 정의
void sayHello_TEST();
void status_cb(const mavros_msgs::State::ConstPtr& msg);
void state_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); 
void state_speed_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);
void quaternion_to_euler_angle(const geometry_msgs::PoseStamped msg, double Current_attitude[3]);
void euler_angle_to_quaternion(double roll, double pitch, double yaw, double quater_angle[3]);

class Heading
{
public:
	Heading(double P, double I, double D); //Constructor! you have to understand this!  it is for inlitial valu.

    void Heading_Parameter(double P, double I, double D);
	double Heading_Controller(double desired_heading, double current_heading);
   
private:
        double P_gain;
        double I_gain;
        double D_gain;
		double I_term;
        double error_prev;

};

class Altitude
{
public:
	Altitude(double P, double I, double D); //Constructor! you have to understand this!  it is for inlitial valu.
    void  Altitude_Parameter(double P, double I, double D);
    double Altitude_Controller(double desired_altitude, double current_altitude);
  
private:
	double P_gain;
    double I_gain;
    double D_gain;
    double I_term;
    double error_prev;
    // double pitch_wind_up;
};


class Speed
{
public:
	Speed(double P, double I, double D); //Constructor! you have to understand this!  it is for inlitial valu.

    void   Speed_Parameter(double P, double I, double D);
    double Speed_Controller(double desired_speed, double current_speed);
   
private:
	double P_gain;
    double I_gain;
    double D_gain;
    double I_term;
    double error_prev;
    // double speed_wind_up;
};




double Sidebearing_Controller(double UAV_pos[2], double psi, double Target_pos[2], double R_ref, double UAV_vel);