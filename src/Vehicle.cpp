#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace RTS {

    Vehicle::Vehicle(double dryMass, double propellantMass, double thrust, double isp)
    : dryMass(dryMass), propellantMass(propellantMass), thrust(thrust), isp(isp),
      currentMass(dryMass + propellantMass), velocity(0.0), altitude(0.0),
      currentPhase(FlightPhase::PRE_LAUNCH) {}

    Derivative Vehicle::computeDerivative(const State& s, double airDensity, double speedOfSound) const {
        bool burning = s.mass > dryMass;

        const double refArea = 0.1;
        double cd   = dragModel.getCd(s.velocity, speedOfSound);
        double drag = 0.5 * airDensity * s.velocity * std::abs(s.velocity) * cd * refArea;

        double weight       = s.mass * g0;
        double netForce     = (burning ? thrust : 0.0) - drag - weight;
        double acceleration = netForce / s.mass;

        double massFlowRate = burning ? -(thrust / (isp * g0)) : 0.0;

        return Derivative{s.velocity, acceleration, massFlowRate};
    }

    void Vehicle::update(double dt, double airDensity, double speedOfSound) {

        // RK4 integration
        State current{altitude, velocity, currentMass};
        Derivative k1 = computeDerivative(current, airDensity, speedOfSound);
        
        State s2{current.altitude + 0.5 * dt * k1.dAltitude,
                 current.velocity + 0.5 * dt * k1.dVelocity,
                 current.mass     + 0.5 * dt * k1.dMass};
        Derivative k2 = computeDerivative(s2, airDensity, speedOfSound);
            
        State s3{current.altitude + 0.5 * dt * k2.dAltitude,
                 current.velocity + 0.5 * dt * k2.dVelocity,
                 current.mass     + 0.5 * dt * k2.dMass};
        Derivative k3 = computeDerivative(s3, airDensity, speedOfSound);
                
        State s4{current.altitude + dt * k3.dAltitude,
                 current.velocity + dt * k3.dVelocity,
                 current.mass     + dt * k3.dMass};
        Derivative k4 = computeDerivative(s4, airDensity, speedOfSound);
        
        altitude    += (dt / 6.0) * (k1.dAltitude + 2*k2.dAltitude + 2*k3.dAltitude + k4.dAltitude);
        velocity    += (dt / 6.0) * (k1.dVelocity + 2*k2.dVelocity + 2*k3.dVelocity + k4.dVelocity);
        currentMass += (dt / 6.0) * (k1.dMass     + 2*k2.dMass     + 2*k3.dMass     + k4.dMass);

        // Sync propellant mass from RK4 mass update
        double massLost = (dryMass + propellantMass) - currentMass;
        propellantMass = std::max(0.0, propellantMass - massLost);
        if (currentMass < dryMass) currentMass = dryMass;

        // Flight phase detection
        double dynamicPressure = 0.5 * airDensity * velocity * velocity;

        if (currentPhase == FlightPhase::PRE_LAUNCH && velocity > 0.0) {
            currentPhase = FlightPhase::BOOST;
        }
        else if (currentPhase == FlightPhase::BOOST && dynamicPressure > 3000.0) {
            currentPhase = FlightPhase::MAX_Q;
        }
        else if (currentPhase == FlightPhase::MAX_Q && isBurnout()) {
            currentPhase = FlightPhase::BURNOUT;
        }
        else if (currentPhase == FlightPhase::BURNOUT && velocity > 0.0) {
            currentPhase = FlightPhase::COAST;
        }
        else if (currentPhase == FlightPhase::COAST && velocity <= 0.0) {
            currentPhase = FlightPhase::APOGEE;
        }
        else if (currentPhase == FlightPhase::APOGEE) {
            currentPhase = FlightPhase::DESCENT;
        }
        else if (currentPhase == FlightPhase::DESCENT && altitude <= 0.0) {
            currentPhase = FlightPhase::LANDED;
            altitude = 0.0;
            velocity = 0.0;
        }

        if (currentPhase == FlightPhase::LANDED) return;
    }

    double Vehicle::getMass() const { return currentMass; }
    double Vehicle::getVelocity() const { return velocity; }
    double Vehicle::getAltitude() const { return altitude; }
    bool Vehicle::isBurnout() const { return propellantMass <= 0.0; }
    FlightPhase Vehicle::getPhase() const { return currentPhase; }

}