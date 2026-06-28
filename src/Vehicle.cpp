#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace RTS {

    Vehicle::Vehicle(double dryMass, double propellantMass, double thrust, double isp)
        : dryMass(dryMass), propellantMass(propellantMass), thrust(thrust), isp(isp),
          currentMass(dryMass + propellantMass), velocity(0.0), altitude(0.0) {}

    Derivative Vehicle::computeDerivative(const State& s, double airDensity) const {
        bool burning = s.mass > dryMass;

        const double dragCoeff = 0.5;
        const double refArea   = 0.1;
        double drag = 0.5 * airDensity * s.velocity * std::abs(s.velocity) * dragCoeff * refArea;
        double weight       = s.mass * g0;
        double netForce     = (burning ? thrust : 0.0) - drag - weight;
        double acceleration = netForce / s.mass;

        double massFlowRate = burning ? -(thrust / (isp * g0)) : 0.0;

        return Derivative{s.velocity, acceleration, massFlowRate};
    }

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

        // RK4 integration
        State current{altitude, velocity, currentMass};
        Derivative k1 = computeDerivative(current, airDensity);
        
        State s2{current.altitude + 0.5 * dt * k1.dAltitude,
            current.velocity + 0.5 * dt * k1.dVelocity,
            current.mass    + 0.5 * dt * k1.dMass};
        Derivative k2 = computeDerivative(s2, airDensity);
            
        State s3{current.altitude + 0.5 * dt * k2.dAltitude,
            current.velocity + 0.5 * dt * k2.dVelocity,
            current.mass    + 0.5 * dt * k2.dMass};
        Derivative k3 = computeDerivative(s3, airDensity);
                
        State s4{current.altitude + dt * k3.dAltitude,
            current.velocity + dt * k3.dVelocity,
            current.mass    + dt * k3.dMass};
        Derivative k4 = computeDerivative(s4, airDensity);
        
        altitude    += (dt / 6.0) * (k1.dAltitude + 2*k2.dAltitude + 2*k3.dAltitude + k4.dAltitude);
        velocity    += (dt / 6.0) * (k1.dVelocity + 2*k2.dVelocity + 2*k3.dVelocity + k4.dVelocity);
        currentMass += (dt / 6.0) * (k1.dMass    + 2*k2.dMass     + 2*k3.dMass     + k4.dMass);

    }

    double Vehicle::getMass() const { return currentMass; }
    double Vehicle::getVelocity() const { return velocity; }
    double Vehicle::getAltitude() const { return altitude; }
    bool Vehicle::isBurnout() const { return propellantMass <= 0.0; }

}