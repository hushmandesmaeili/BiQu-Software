#ifndef PROJECT_personTracking_H
#define PROJECT_personTracking_H


#include <librealsense2/rs.hpp>
#include <iostream>
#include "cppTypes.h"


struct personPose{
    float pose_x;
    float pose_y;
    float depth_to_person;
    float angle;
  ;
};

class personTracker{
public:
    int init();
    void get_pose();
    personPose pose;


private:
    rs2::pipeline p;
};

#endif
