/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cppTypes.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat;
  // todo is there status for the vectornav?
};

struct CamVectorNavData {
  float x,y,z; // linear translation
  float rot_w, rot_x, rot_y,rot_z; // orientation
  float vel_x, vel_y, vel_z; // linear velocity
  float acc_x, acc_y, acc_z; // linear acceleration
  float ang_vel_x, ang_vel_y, ang_vel_z; // angular acceleration
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState {
  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
};

#endif  // PROJECT_IMUTYPES_H
