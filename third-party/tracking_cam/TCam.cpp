#include "TCam.h"
#include <mutex>
#include <thread>
#include "../../common/include/Math/orientation_tools.h"


static std::mutex dataMutex;
bool TCam::init(){
try{
p.start();
return true;
}
catch (const rs2::error & e){
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return false;
}

}

void TCam::get_pose(){
    try{
         dataMutex.lock();
// Wait for the next set of frames from the camera
        auto frames = p.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        pose.position[0] = pose_data.translation.x;
        pose.position[1] = pose_data.translation.y;
        pose.position[2] = pose_data.translation.z;
        
        pose.quat[0] = pose_data.rotation.w;
        pose.quat[1] = pose_data.rotation.x;
        pose.quat[2] = pose_data.rotation.y;
        pose.quat[3] = pose_data.rotation.z;

        pose.omega[0] = pose_data.angular_velocity.x;
        pose.omega[1] = pose_data.angular_velocity.y;
        pose.omega[2] = pose_data.angular_velocity.z;

        pose.acceleration[0] = pose_data.acceleration.x;
        pose.acceleration[1] = pose_data.acceleration.y;
        pose.acceleration[2] = pose_data.acceleration.z;

        pose.ang_acc[0] = pose_data.angular_acceleration.x;
        pose.ang_acc[1] = pose_data.angular_acceleration.y;
        pose.ang_acc[2] = pose_data.angular_acceleration.z;

        dataMutex.unlock();
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    }
}

void TCam::updateLCM(tCam_lcmt *message) {
    dataMutex.lock();    

    message->quat[0] = pose.quat[0];
    message->quat[1] = pose.quat[1];
    message->quat[2] = pose.quat[2];
    message->quat[3] = pose.quat[3];
    
    message->position[0]= pose.position[0];
    message->position[1]= pose.position[1];
    message->position[2]= pose.position[2];

    message->omega[0] = pose.omega[0];
    message->omega[1] = pose.omega[1];
    message->omega[2] = pose.omega[2];

    message->acceleration[0] = pose.acceleration[0];
    message->acceleration[1] = pose.acceleration[1];
    message->acceleration[2] = pose.acceleration[2];

    message->ang_acc[0] = pose.ang_acc[0];
    message->ang_acc[1] = pose.ang_acc[1];
    message->ang_acc[2] = pose.ang_acc[2];


    Vec3<float> rpy = ori::quatToRPY(pose.quat);
    for(u32 i = 0; i < 3; i++) {
        message->rpy[i] = rpy[i];
    }

  
  dataMutex.unlock();
}

