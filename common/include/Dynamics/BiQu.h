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
Quadruped<T> buildBiQu()
{
    Quadruped<T> biqu;
    biqu._robotType = RobotType::BIQU;

    biqu._bodyMass = 0.70760362;
    biqu._bodyLength = 0.194 * 2;
    biqu._bodyWidth = 0.0975 * 2;
    biqu._bodyHeight = 0.025 * 2;
    biqu._abadGearRatio = 9;
    biqu._hipGearRatio = 9;
    biqu._kneeGearRatio = 9;
    biqu._abadLinkLength = 0.01295;
    biqu._hipLinkLength = 0.160;
    biqu._kneeLinkY_offset = 0.04745;
    biqu._kneeLinkLength = 0.1675;
    biqu._maxLegLength = 0.3275;

    biqu._motorTauMax = 2.7;
    biqu._batteryV = 24;
    /* KT = flux linkage * number of poles,
    where the number of poles P = 12 for the Antigravity MN4004 KV300 motor.
    Flux linkage is estimated with $\lambda_{m} = \frac{3}{2}\frac{60}{2\pi P\sqrt{3}K_{v}}$. Thus,
    $K_{t} = 0.02756*/
    biqu._motorKT = .02756;
    biqu._motorR = 0.452;
    biqu._jointDamping = .01;
    biqu._jointDryFriction = .2;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 6.68e-6, 0.0, 0.0, 0.0, 6.68e-6, 0.0, 0.0, 0.0, 9.49e-6;

    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();

    // All inertias are taken at the CoM and expressed in the body coordinate system (as done in Jared's paper)
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 1.429e-5, 4.46e-6, -1.3e-7, 4.46e-6, 2.8123e-4, -2.0e-8, -1.3e-7, -2.0e-8, 2.7434e-4;
    Vec3<T> abadCOM(-7.257111e-2, 1.48758e-3, 2.191e-5);
    SpatialInertia<T> abadInertia(8.96779e-2, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 3.0639e-4, -1e-8, 1.7e-7, -1e-8, 3.1039e-4, -2.214e-5, 1.7e-7, -2.214e-5, 1.731e-5;
    Vec3<T> hipCOM(2.781e-5, 2.41857e-2, -9.874663e-2);
    SpatialInertia<T> hipInertia(9.023798e-2, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 1.2115e-4, 0.0, 0.0, 0.0, 1.2208e-4, -2.76e-6, 0.0, -2.76e-6, 1.97e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0.0, 7.89555e-3, -6.682471e-2);
    SpatialInertia<T> kneeInertia(3.74002e-2, kneeCOM, kneeRotationalInertia);

    Vec3<T> rotorCOM(0.0, 0.0, -7.52857e-3);
    SpatialInertia<T> rotorInertiaX(5.718802e-2, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(5.718802e-2, rotorCOM, rotorRotationalInertiaY);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 2.12852e-3, 1.1363e-4, -1.662e-5, 1.1363e-4, 8.79547e-3, -4.8e-7, -1.662e-5, -4.8e-7, 1.059412e-2;
    Vec3<T> bodyCOM(-1.550066e-3, -3.108e-5, 8.872e-5);
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
    biqu._abadRotorLocation = Vec3<T>(4.82e-2, -2.63e-2, 0);
    biqu._abadLocation = Vec3<T>(2.1285e-1, 8.75e-2, 0.0);
    biqu._hipLocation = Vec3<T>(-1.85e-2, biqu._abadLinkLength, 0.0);
    biqu._hipRotorLocation = Vec3<T>(3.46e-2, -5.7e-3, 0);
    biqu._kneeLocation = Vec3<T>(0.0, 0.0, -biqu._hipLinkLength);
    biqu._kneeRotorLocation = Vec3<T>(0, 1.88e-2, -4.62e-2);
    return biqu;
}

#endif // PROJECT_BIQU_H
