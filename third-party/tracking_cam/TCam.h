#ifndef PROJECT_TCAM_H
#define PROJECT_TCAM_H


#include <librealsense2/rs.hpp> 
#include <iostream>
#include "cppTypes.h"
#include "../../lcm-types/cpp/tCam_lcmt.hpp"
//this is a misnomer but did not want to name it state
struct Pose{
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
    void updateLCM(tCam_lcmt *message);
    Pose pose;


private:
    rs2::pipeline p;
};

#endif //PROJECT_TCAM_H