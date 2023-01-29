#include <librealsense2/rs.hpp> 
#include <iostream>
#include "cppTypes.h"

//contains velocity as well
struct Pose{
    // Wait for the next set of frames from the camera
    
    // float x,y,z; // linear translation
    // float rot_w, rot_x, rot_y,rot_z; // quaternions
    // float vel_x, vel_y, vel_z; // linear velocity
    // float acc_x, acc_y, acc_z; // linear acceleration
    // float ang_vel_x, ang_vel_y, ang_vel_z; // angular acceleration

    Vec3<float> position;
    Vec3<float> velocity;
    Vec3<float> acceleration;
    Quat<float> quat;
    Vec3<float> omega;
    Vec3<float> ang_acc;
};

class TCam{
public:
    bool init();
    void get_pose();
    Pose pose;


private:
    rs2::pipeline p;
};