#include "Planner/planner.h"
#include <string.h>

// rc_control_settings rc_control;
// void get_rc_control_settings(void *settings) {
//   pthread_mutex_lock(&lcm_get_set_mutex);
//   v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
//   pthread_mutex_unlock(&lcm_get_set_mutex);
// }
void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}
int Planner::tracker_to_rc_control(rc_control_settings *rc_control_input,VisionData &visionData)
{
    _rc_control.p_des[0] = visionData.tracker_x; // (x, y) -1 ~ 1
    _rc_control.p_des[1] = visionData.tracker_y;
    //double     height_variation; // -1 ~ 1
   
    _rc_control.rpy_des[0] = visionData.tracker_rpy[0]; 
    _rc_control.rpy_des[1] = visionData.tracker_rpy[1];
    _rc_control.rpy_des[2] = visionData.tracker_rpy[2];

    //TODO
    _rc_control.omega_des[0] = visionData.tracker_rpy[0] - previous_rpy[0];
    _rc_control.omega_des[1] = visionData.tracker_rpy[1] - previous_rpy[1];
    _rc_control.omega_des[2] = visionData.tracker_rpy[2] -  previous_rpy[2];

    vel_x = visionData.tracker_x - previous_pos_x;
    vel_y = visionData.tracker_y - previous_pos_y;

    if(vel_x>2){vel_x = 2;}
    else if(vel_x<0){vel_y = 0;}
    if(vel_y>2){vel_x = 2;}
    else if(vel_y<0){vel_y = 0;}


    _rc_control.v_des[0] = kp_vel * vel_x + kd_vel * error_vel_x; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    _rc_control.v_des[1] = kp_vel * vel_y + kd_vel * error_vel_y;
    _rc_control.v_des[2] = 0;
    
    v_memcpy(rc_control_input, &_rc_control, sizeof(rc_control_settings));

    return 0;

}
