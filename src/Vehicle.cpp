#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace RTS {

    Vehicle::Vehicle(double dryMass, double propellantMass, double thrust, double isp)
        : dryMass(dryMass), propellantMass(propellantMass), thrust(thrust), isp(isp),
          currentMass(dryMass + propellantMass), velocity(0.0), altitude(0.0) {}

    void Vehicle::update(double dt, double airDensity) {
        bool burning = propellantMass > 0.0;

        double currentThrust = burning ? thrust : 0.0;

        // Drag force (simple flat-plate model for now, refine later)
        const double dragCoeff = 0.5;
        const double refArea = 0.1; // m^2, placeholder until CAD-derived value
        double drag = 0.5 * airDensity * velocity * std::abs(velocity) * dragCoeff * refArea;

        double weight = currentMass * g0;

        double netForce = currentThrust - drag - weight;
        double acceleration = netForce / currentMass;

        velocity += acceleration * dt;
        altitude += velocity * dt;

        if (burning) {
            double massFlowRate = thrust / (isp * g0);
            double massBurned = massFlowRate * dt;

            propellantMass -= massBurned;
            if (propellantMass < 0.0) {
                massBurned += propellantMass; // correct overshoot
                propellantMass = 0.0;
            }
            currentMass -= massBurned;
        }
    }

    double Vehicle::getMass() const { return currentMass; }
    double Vehicle::getVelocity() const { return velocity; }
    double Vehicle::getAltitude() const { return altitude; }
    bool Vehicle::isBurnout() const { return propellantMass <= 0.0; }

}