#include "planner.h"


rc_control_settings rc_control;
void get_rc_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

int Planner::tracker_to_rc_control(Rc_control_input *rc_control_input,VisionData &visionData)
{
    rc_control->p_des[0] = visionData.tracker_x; // (x, y) -1 ~ 1
    rc_control->p_des[1] = visionData.tracker_y;
    //double     height_variation; // -1 ~ 1
   
    rpy_des[0] = visionData.tracker_rpy[0]; 
    rpy_des[1] = visionData.tracker_rpy[1];
    rpy_des[2] = visionData.tracker_rpy[2];

    //TODO
    rc_control->omega_des[0] = visionData.rpy[0] - previous_rpy[0];
    rc_control->omega_des[1] = visionData.rpy[1] - previous_rpy[1];
    rc_control->omega_des[2] = visionData.rpy[2] -  previous_rpy[2];

    vel_x = visionData.tracker_x - previous_pos_x;
    vel_y = visionData.tracker_y - previous_pos_y;

    if(vel_x>2){vel_x = 2;}
    else if(vel_x<0){vel_y = 0;}
    if(vel_y>2){vel_x = 2;}
    else if(vel_y<0){vel_y = 0;}


    rc_control->v_des[0] = kp_vel * vel_x + kd_vel * error_vel_x; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    rc_control->v_des[1] = kp_vel * vel_y + kd_vel * error_vel_y;
    rc_control->v_des[2] = 0;
    


    return 0;

}