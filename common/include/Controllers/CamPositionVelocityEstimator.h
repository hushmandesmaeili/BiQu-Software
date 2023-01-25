/*! @file CamPositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class CamLinearKFPositionVelocityEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CamLinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();  
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterPositionVelocityEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
