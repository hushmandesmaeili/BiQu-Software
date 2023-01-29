#include "TCam.h"
#include <mutex>
#include <thread>

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
        // pose.x = pose_data.translation.x;
        // pose.y = pose_data.translation.y;
        // pose.z = pose_data.translation.z;
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


        // pose.rot_w = pose_data.rotation.w;
        // pose.rot_x = pose_data.rotation.x;
        // pose.rot_y= pose_data.rotation.y;
        // pose.rot_z= pose_data.rotation.z;

        // pose.vel_x = pose_data.velocity.x;
        // pose.vel_y = pose_data.velocity.y;
        // pose.vel_z = pose_data.velocity.z;

        // pose.acc_x = pose_data.acceleration.x;
        // pose.acc_y = pose_data.acceleration.y;
        // pose.acc_z= pose_data.acceleration.z;

        // pose.ang_vel_x = pose_data.angular_velocity.x;
        // pose.ang_vel_y = pose_data.angular_velocity.y;
        // pose.ang_vel_z = pose_data.angular_velocity.z;
    


        dataMutex.unlock();
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    }
   

}


int main(){
    
}