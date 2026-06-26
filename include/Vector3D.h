#ifndef AETHER_SIM_VECTOR3D_H
#define AETHER_SIM_VECTOR3D_H

namespace Aether {

/**
 * @brief Minimalist, independent 3D Vector structure to handle kinematic translations.
 * Requires absolutely zero external dependencies, making it safe to compile anywhere.
 */
struct Vector3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // Standard constructive declarations
    Vector3D() = default;
    Vector3D(double valX, double valY, double valZ) : x(valX), y(valY), z(valZ) {}

    // Overloaded utility operator to allow rapid force vectors accumulation
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
};

} // namespace Aether

#endif // AETHER_SIM_VECTOR3D_H