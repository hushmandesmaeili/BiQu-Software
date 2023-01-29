#include "../TCam.h"
#include <cstdio>

int main(int argc, char** argv) {

    TCam tCam;
    tCam.init();
    while(true){
        tCam.get_pose();
        printf(" position: %.3f %.3f %.3f\n", tCam.pose.x, tCam.pose.y, tCam.pose.z);
    }
    
    
}
