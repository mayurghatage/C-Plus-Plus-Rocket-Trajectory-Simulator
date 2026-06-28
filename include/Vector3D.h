#ifndef RTS_SIM_VECTOR3D_H
#define RTS_SIM_VECTOR3D_H
#include <cmath>

namespace RTS {

// 3D vector struct for kinematic calculations (position, velocity, force vectors)
// No external dependencies — compiles anywhere
struct Vector3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // Standard constructive declarations
    Vector3D() = default;
    Vector3D(double valX, double valY, double valZ) : x(valX), y(valY), z(valZ) {}

    // Operator overloads for vector math
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    Vector3D operator-(const Vector3D& other) const {
    return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    Vector3D operator/(double scalar) const {
    return Vector3D(x / scalar, y / scalar, z / scalar);
    }

    double dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
    }

    double magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
    }

    Vector3D normalize() const {
        double mag = magnitude();
        return Vector3D(x / mag, y / mag, z / mag);
    }
};

} // namespace RTS

#endif // RTS_SIM_VECTOR3D_H