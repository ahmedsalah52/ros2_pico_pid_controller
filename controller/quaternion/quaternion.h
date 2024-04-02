// Quaternion.h
#ifndef QUATERNION_H
#define QUATERNION_H

// Include the necessary header for mathematical operations
#include <cmath>

// Structure to represent a quaternion
struct Quaternion {
    double x, y, z, w;
};

struct Euler {
    double roll, pitch, yaw;
};

// Declaration of the EulerToQuaternion function
Quaternion EulerToQuaternion(double roll, double pitch, double yaw);
Euler QuaternionToEuler(double x, double y, double z, double w);

#endif // QUATERNION_H