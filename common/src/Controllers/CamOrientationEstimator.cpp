#include "Controllers/CamOrientationEstimator.h"




/*! @file CamOrientationEstimator.cpp
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */


/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
template <typename T>
void CamVectorNavOrientationEstimator<T>::run() {
    //TODO Please test assignment
  this->_stateEstimatorData.result->orientation[0] =
  this->_stateEstimatorData.camVectorNavData->quat[0]; 
  this->_stateEstimatorData.result->orientation[1] =
  this->_stateEstimatorData.camVectorNavData->quat[1]; 
  this->_stateEstimatorData.result->orientation[2] =
  this->_stateEstimatorData.camVectorNavData->quat[2]; 
  this->_stateEstimatorData.result->orientation[3] =
  this->_stateEstimatorData.camVectorNavData->quat[3]; 
  

  if(_b_first_visit){
    Vec3<T> rpy_ini = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }
  this->_stateEstimatorData.result->orientation = 
    ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);


  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaBody[0] =
      this->_stateEstimatorData.camVectorNavData->omega[0];

  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->aBody[0] =
      this->_stateEstimatorData.camVectorNavData->ang_acc[0];
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}

template class CamVectorNavOrientationEstimator<float>;
template class CamVectorNavOrientationEstimator<double>;
