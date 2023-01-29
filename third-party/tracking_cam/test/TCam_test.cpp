#include "../TCam.h"
#include <cstdio>


int main(int argc, char** argv) {

    TCam tCam;
    
    if(tCam.init()){
        while(true){
            tCam.get_pose();
            printf(" position: %.3f %.3f %.3f\n", tCam.pose.position[0], tCam.pose.position[1], tCam.pose.position[2]);
    }
    }
    
    else {
        printf("no cam connected");
    }   
}
