#ifndef PROJECT_personTracking_H
#define PROJECT_personTracking_H


#include <librealsense2/rs.hpp>
#include <iostream>
#include "cppTypes.h"


struct Pose{
    float pose_x;
    float pose_y;
    float depth_to_person;
    float angle;
  ;
};

class personTracker{
public:
    int init();
    personPose pose;


};

#endif
