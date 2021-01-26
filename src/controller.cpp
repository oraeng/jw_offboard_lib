#include "jw_offboard_lib/controller.h"

//  geometry_msgs::PoseStamped current_state;  // receive the attitude data from pixhawk(px4-based gazebo)
//  mavros_msgs::VFR_HUD current_airspeed;  // receive the current airspeed from pixhawk(px4-based gazebo)
//  mavros_msgs::AttitudeTarget attitudecommand; // to publish the attitude command to pixhawk(px4-based gazebo)



void sayHello_TEST()
{
    ROS_INFO("Hello! Jinwoo!!");
}

void status_cb(const mavros_msgs::State::ConstPtr& msg){
    current_status = *msg;
}

void state_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
   current_attitude =  *msg;
  }

void state_speed_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){ 
   current_speed =  *msg; 
  }





void quaternion_to_euler_angle(const geometry_msgs::PoseStamped msg, double Current_attitude[3]){
    double  x = msg.pose.orientation.x;
	double  y = msg.pose.orientation.y;
	double  z = msg.pose.orientation.z;
	double  w = msg.pose.orientation.w;

    // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        Current_attitude[0] = std::atan2(sinr_cosp, cosr_cosp);     
    // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            Current_attitude[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            Current_attitude[1] = std::asin(sinp);

    // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        Current_attitude[2] = std::atan2(siny_cosp, cosy_cosp);
        // Current_attitude[2] = 9999;
    // return ;
}


void euler_angle_to_quaternion(double roll, double pitch, double yaw, double quater_angle[3]){

            double cy = cos(yaw*0.5);
            double sy = sin(yaw*0.5);
            double cp = cos(pitch*0.5);
            double sp = sin(pitch*0.5); 
            double cr = cos(roll*0.5);
            double sr = sin(roll*0.5);

            // quaternion q;
            quater_angle[0] = cy*cp*cr + sy*sp*sr;
            quater_angle[1] = cy*cp*sr - sy*sp*cr;
            quater_angle[2] = sy*cp*sr + cy*sp*cr;
            quater_angle[3] = sy*cp*cr - cy*sp*sr;   
    // return ;
}


Heading::Heading(double P, double I, double D) //Constructor! you have to understand this!  it is for inlitial valu.
	{
        P_gain = P;
        I_gain = I;
        D_gain = D;
		I_term = 0;
		error_prev = 0;
	};


void Heading::Heading_Parameter(double P, double I, double D)
	{
      P_gain = P;
      I_gain = I;
      D_gain = D;
	}

double Heading::Heading_Controller(double desired_heading, double current_heading)
    {
        // chi_dot = g/Va *tan (phi)
        // phi = atan(Va/g * chi_dot)
        double error_curr;
        double P_term;
        double D_term;
        double pitch;

        error_curr = desired_heading-current_heading;     
        error_curr = remainder(error_curr,2* M_PI);

       // Wrapped heading
       if (abs(error_curr) > M_PI )
      {
      	error_curr = error_curr - 2*M_PI*sign(error_curr);
      }
       if(error_curr>1.0)
       {
          error_curr= 1.0;
       }
       else if(error_curr<-1)
       { 
          error_curr = -1;
       }
        ROS_INFO("%f", I_term);
        P_term = P_gain*error_curr;
        I_term = I_term + I_gain*error_curr;
        D_term = D_gain*(error_curr-error_prev); // dt is in the Gain.


    //    iout = iout + heading_wind_up; // integrater antiwind up 
       double roll = P_term + I_term + D_term;
    //    double roll_pre_statuation = roll;

       if(roll > 1)
          {
           roll= 1;
          }
       else if(roll < -1) 
         {
          roll = -1;
         }
    //    double roll_after_statuation = roll;
       
    //    heading_wind_up = roll_after_statuation - roll_pre_statuation;

       return roll;
    }


Altitude::Altitude(double P, double I, double D) //Constructor! you have to understand this!  it is for inlitial valu.
	{
        P_gain = P;
        I_gain = I;
        D_gain = D;
		I_term = 0;
        error_prev = 0;
	};

void Altitude::Altitude_Parameter(double P, double I, double D)
	{
      P_gain = P;
      I_gain = I;
      D_gain = D;

	}

double Altitude::Altitude_Controller(double desired_altitude, double current_altitude){
    double error_curr;
    double P_term;
    double D_term;
    double pitch;

    error_curr = desired_altitude-current_altitude;

    P_term = P_gain*error_curr;
    I_term = I_term + I_gain*error_curr;
    D_term = D_gain*(error_curr-error_prev); // dt is in the Gain.

    pitch = P_term + I_term + D_term;


        if(pitch>0.15)
       {
          pitch= 0.15;
      }
       else if(pitch<-0.15) 
       {
          pitch = -0.15;
       }

    error_prev = error_curr;
    return pitch; //입력속도 출력함.
}


Speed::Speed(double P, double I, double D) //Constructor! you have to understand this!  it is for inlitial valu.
	{
        P_gain = P;
        I_gain = I;
        D_gain = D;
		I_term = 0;
        error_prev = 0;
		// speed_wind_up = 0;
	};

void Speed::Speed_Parameter(double P, double I, double D)
	{
      P_gain = P;
      I_gain = I;
      D_gain = D;
	}

double Speed::Speed_Controller(double desired_speed, double current_speed){
    double error_curr;
    double P_term;
    double D_term;
    double thrust;

    error_curr = desired_speed-current_speed;


    P_term = P_gain*error_curr;
    I_term = I_term + I_gain*error_curr;
    D_term = D_gain*(error_curr-error_prev); // dt is in the Gain.
    // ROS_INFO("P Term : %f // I Term : ")
    thrust = P_term + I_term + D_term;

    if(thrust > 0.8)
          {
           thrust= 0.8;
          }
    else if(thrust <= 0)
    {thrust = 0;}
    // else if(thrust < -0.8) 
    //      {
    //       thrust = -0.8;
    //      }

    error_prev = error_curr;
    return thrust; //입력속도 출력함.
}


double Sidebearing_Controller(double UAV_pos[2], double psi, double Target_pos[2], double R_ref, double UAV_vel)
{   
    double x, y, v_x, v_y, V_uav; 
    double xt, yt, vt_x, vt_y; 
    double rt;
    double V_AT_Vec[2], e_n[2], e_TA[2];
    double sin_eta, cos_csi, w;
    double phi_cmd;

    
    x  = UAV_pos[0];
    y  = UAV_pos[1];
    V_uav = UAV_vel;
    xt = Target_pos[0]+0.0001;
    yt = Target_pos[1]+0.0001;
    rt = sqrt((xt-x)*(xt-x)+(yt-y)*(yt-y));

    vt_x = 0;
    vt_y = 0;

    v_x = V_uav*cos(psi);
    v_y = V_uav*sin(psi);

    V_AT_Vec[0] = v_x-vt_x;
    V_AT_Vec[1] = v_y-vt_y;
    
    e_n[0] = -sin(psi); // sin
    e_n[1] = cos(psi);  // cos 
    e_TA[0] = (xt-x)/rt;
    e_TA[1] = (yt-y)/rt;

    // e_n[0] = -cos(psi); // sin
    // e_n[1] = sin(psi);  // cos 
    // e_TA[1] = (xt-x)/rt;
    // e_TA[0] = (yt-y)/rt;


    sin_eta = (e_n[0]*e_TA[1] - e_TA[0]*e_n[1]);
    cos_csi = (v_x*V_AT_Vec[0] + v_y*V_AT_Vec[1])/sqrt(V_AT_Vec[0]*V_AT_Vec[0]+V_AT_Vec[1]*V_AT_Vec[1])/V_uav;




    // w = (V_uav*V_uav/R_ref) * (1+2*0.707*sin_eta)/cos_csi/V_uav;
    w = V_uav/R_ref + (2/V_uav)*sin_eta;
    
    phi_cmd = w*V_uav/9.81;
    // ROS_INFO("psi : %f", psi);
    // ROS_INFO("sin_eta : %f", sin_eta);
    // ROS_INFO("Ref_rate_w : %f", UAV_vel/R_ref);
    // ROS_INFO("phi_cmd : %f", phi_cmd*180/3.14);
    return phi_cmd;
}