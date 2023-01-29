
#include "Controllers/CamPositionVelocityEstimator.h"
/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CamLinearKFPositionVelocityEstimator<T>::setup() {} // no need
template <typename T>
CamLinearKFPositionVelocityEstimator<T>::CamLinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void CamLinearKFPositionVelocityEstimator<T>::run() {

  this->_stateEstimatorData.result->position[0] = this->_stateEstimatorData.camVectorNavData->position[0];
  this->_stateEstimatorData.result->vWorld[0] =this->_stateEstimatorData.camVectorNavData->velocity[0];
  
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}
// template <typename T>
// void CheaterPositionVelocityEstimator<T>::run() {
//   this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
//   this->_stateEstimatorData.result->vWorld =
//       this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
//   this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();

// }

// template class CheaterPositionVelocityEstimator<float>;
// template class CheaterPositionVelocityEstimator<double>;

template class CamLinearKFPositionVelocityEstimator<float>;
template class CamLinearKFPositionVelocityEstimator<double>;