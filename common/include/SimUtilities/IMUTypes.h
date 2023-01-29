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
  Vec3<float> position;
  Vec3<float> velocity;
  Vec3<float> acceleration;
  Quat<float> quat;
  Vec3<float> omega;
  Vec3<float> ang_acc;
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
