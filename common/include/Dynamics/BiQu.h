/*! @file BiQu.h
 *  @brief Utility function to build a BiQu Quadruped object
 *
 * The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_BIQU_H
#define PROJECT_BIQU_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of BIQU
 */
template <typename T>
Quadruped<T> buildBiQu() {
  Quadruped<T> biqu;
  biqu._robotType = RobotType::BIQU;

  biqu._bodyMass = 1.2512;
  biqu._bodyLength = 0.194 * 2;
  biqu._bodyWidth = 0.0975 * 2;
  biqu._bodyHeight = 0.025 * 2;
  biqu._abadGearRatio = 9;
  biqu._hipGearRatio = 9;
  biqu._kneeGearRatio = 9;
  biqu._abadLinkLength = 0.014;
  biqu._hipLinkLength = 0.160;
  biqu._kneeLinkY_offset = 0.0; // not sure what this Y offset is
  biqu._kneeLinkLength = 0.1675;
  biqu._maxLegLength = 0.3275;


  biqu._motorTauMax = 0.27;
  biqu._batteryV = 24;
  /* KT = flux linkage * number of poles,
  where the number of poles P = 12 for the Antigravity MN4004 KV300 motor.
  Flux linkage is estimated with $\lambda_{m} = \frac{60}{2\pi P\sqrt{3}K_{v}}$. Thus,
  $K_{t} = 0.018*/
  biqu._motorKT = .018;  
  biqu._motorR = 0.452;
  biqu._jointDamping = .01;
  biqu._jointDryFriction = .2;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 6.05e-6, 0, 0, 0, 6.05e-6, 0, 0, 0, 9.19e-6;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // All inertias are taken at the CoM and aligned with the body coordinate system
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 1.429e-5, 4.46e-6, -1.3e-7, 4.46e-6, 2.8123e-4, -2.0e-8, -1.3e-7, -2.0e-8, 2.7434e-4;
  Vec3<T> abadCOM(-5.407164e-2, 5.1252e-4, 2.186e-5);
  SpatialInertia<T> abadInertia(0.089, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia <<  3.1489e-4, 0, 1.6e-7, 0, 3.1895e-4, -2.241e-5,  1.6e-7, -2.241e-5, 1.754e-5;
  Vec3<T> hipCOM(2.689e-5, -4.09003e-3, -5.080385e-2);
  SpatialInertia<T> hipInertia(0.093, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 1.2115e-4, 0, 0, 0, 1.2208e-4,  -2.76e-6, 0, -2.76e-6, 1.97e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, -1.10445e-3, -6.682471e-2);
  SpatialInertia<T> kneeInertia(0.037, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 8.17e-3);
  SpatialInertia<T> rotorInertiaX(0.054, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.054, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 2.6622e-3, 0, 0, 0, 1.38851e-3, 0, 0, 0, 1.60537e-3;
  Vec3<T> bodyCOM(-8.2966e-4, 1.05e-6, -6.021e-4);
  SpatialInertia<T> bodyInertia(biqu._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  biqu._abadInertia = abadInertia;
  biqu._hipInertia = hipInertia;
  biqu._kneeInertia = kneeInertia;
  biqu._abadRotorInertia = rotorInertiaX;
  biqu._hipRotorInertia = rotorInertiaY;
  biqu._kneeRotorInertia = rotorInertiaY;
  biqu._bodyInertia = bodyInertia;

  // locations
  biqu._abadRotorLocation = Vec3<T>(-0.026, 0.0175, 0);
  biqu._abadLocation =
      Vec3<T>(biqu._bodyLength, biqu._bodyWidth, 0) * 0.5;
  biqu._hipLocation = Vec3<T>(0, biqu._abadLinkLength, 0);
  biqu._hipRotorLocation = Vec3<T>(-0.113, 0, 0);
  biqu._kneeLocation = Vec3<T>(0, 0, -biqu._hipLinkLength);
  biqu._kneeRotorLocation = Vec3<T>(0, 0, 0.113);

  return biqu;
}

#endif  // PROJECT_BIQU_H
