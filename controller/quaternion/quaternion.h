// Quaternion.h
#ifndef QUATERNION_H
#define QUATERNION_H

// Include the necessary header for mathematical operations
#include <cmath>

// Structure to represent a quaternion
struct Quaternion {
    double x, y, z, w;
};

// Declaration of the EulerToQuaternion function
Quaternion EulerToQuaternion(double roll, double pitch, double yaw);

#endif // QUATERNION_H