#include<iostream>
#include <rt/rt_rc_interface.h>

struct Rc_control_input{
    double     p_des[2]; // (x, y) -1 ~ 1
    double     height_variation; // -1 ~ 1
    double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3]; // -1 ~ 1
    double     omega_des[3];
};

struct VisionData {
  float tracker_x;
  float tracker_y;
  float tracker_rpy[3];
  float tracker_depth;
  
};
get_planner_rc_settings(&rc_control);
class Planner{

int tracker_to_rc_control(Rc_control_input* rc_control_input,VisionData &visionData);
float previous_rpy[3]= {0}; 
float vel_x;
float vel_y;
float error_vel_x;
float error_vel_y;
float previous_pos_x;
float previous_pos_y;
float kp_vel = 0;
float kd_vel = 0.5;

};
// get_planner_rc_settings(rc_control);

