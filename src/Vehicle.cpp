#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace RTS {

    Vehicle::Vehicle(double dryMass, double propellantMass, double thrust, double isp)
    : dryMass(dryMass), propellantMass(propellantMass), thrust(thrust), isp(isp),
    currentMass(dryMass + propellantMass),
    positionX(0.0), positionY(0.0), positionZ(0.0),
    velocityX(0.0), velocityY(0.0), velocityZ(0.0),
    currentPhase(FlightPhase::PRE_LAUNCH) {}

    Derivative Vehicle::computeDerivative(const State& s, double airDensity, double speedOfSound) const {
        bool burning = s.mass > dryMass;

        const double refArea = 0.1;

        double pitchAngle = 0.0;
        if (s.z > 50.0 && burning) {
            pitchAngle = 5.0 * M_PI / 180.0;
        }
        double yawAngle = 0.0; // no lateral drift yet — wind model comes later

        double thrustX = burning ? thrust * std::sin(pitchAngle) * std::cos(yawAngle) : 0.0;
        double thrustY = burning ? thrust * std::sin(pitchAngle) * std::sin(yawAngle) : 0.0;
        double thrustZ = burning ? thrust * std::cos(pitchAngle) : 0.0;

        double speed = std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
        double cd = dragModel.getCd(speed, speedOfSound);
        double dragForce = 0.5 * airDensity * speed * speed * cd * refArea;
        double dragX = (speed > 0) ? dragForce * (s.vx / speed) : 0.0;
        double dragY = (speed > 0) ? dragForce * (s.vy / speed) : 0.0;
        double dragZ = (speed > 0) ? dragForce * (s.vz / speed) : 0.0;

        double accX = (thrustX - dragX) / s.mass;
        double accY = (thrustY - dragY) / s.mass;
        double accZ = (thrustZ - dragZ - s.mass * g0) / s.mass;

        double massFlowRate = burning ? -(thrust / (isp * g0)) : 0.0;

        return Derivative{s.vx, s.vy, s.vz, accX, accY, accZ, massFlowRate};
    }

    void Vehicle::update(double dt, double airDensity, double speedOfSound) {

        State current{positionX, positionY, positionZ, velocityX, velocityY, velocityZ, currentMass};
        Derivative k1 = computeDerivative(current, airDensity, speedOfSound);

        State s2{current.x + 0.5*dt*k1.dx, current.y + 0.5*dt*k1.dy, current.z + 0.5*dt*k1.dz,
                 current.vx + 0.5*dt*k1.dvx, current.vy + 0.5*dt*k1.dvy, current.vz + 0.5*dt*k1.dvz,
                 current.mass + 0.5*dt*k1.dMass};
        Derivative k2 = computeDerivative(s2, airDensity, speedOfSound);

        State s3{current.x + 0.5*dt*k2.dx, current.y + 0.5*dt*k2.dy, current.z + 0.5*dt*k2.dz,
                 current.vx + 0.5*dt*k2.dvx, current.vy + 0.5*dt*k2.dvy, current.vz + 0.5*dt*k2.dvz,
                 current.mass + 0.5*dt*k2.dMass};
        Derivative k3 = computeDerivative(s3, airDensity, speedOfSound);

        State s4{current.x + dt*k3.dx, current.y + dt*k3.dy, current.z + dt*k3.dz,
                 current.vx + dt*k3.dvx, current.vy + dt*k3.dvy, current.vz + dt*k3.dvz,
                 current.mass + dt*k3.dMass};
        Derivative k4 = computeDerivative(s4, airDensity, speedOfSound);

        positionX += (dt/6.0)*(k1.dx + 2*k2.dx + 2*k3.dx + k4.dx);
        positionY += (dt/6.0)*(k1.dy + 2*k2.dy + 2*k3.dy + k4.dy);
        positionZ += (dt/6.0)*(k1.dz + 2*k2.dz + 2*k3.dz + k4.dz);
        velocityX += (dt/6.0)*(k1.dvx + 2*k2.dvx + 2*k3.dvx + k4.dvx);
        velocityY += (dt/6.0)*(k1.dvy + 2*k2.dvy + 2*k3.dvy + k4.dvy);
        velocityZ += (dt/6.0)*(k1.dvz + 2*k2.dvz + 2*k3.dvz + k4.dvz);
        currentMass += (dt/6.0)*(k1.dMass + 2*k2.dMass + 2*k3.dMass + k4.dMass);

        double massLost = (dryMass + propellantMass) - currentMass;
        propellantMass = std::max(0.0, propellantMass - massLost);
        if (currentMass < dryMass) currentMass = dryMass;

        double speed = std::sqrt(velocityX*velocityX + velocityY*velocityY + velocityZ*velocityZ);
        double dynamicPressure = 0.5 * airDensity * speed * speed;

        if (currentPhase == FlightPhase::PRE_LAUNCH && velocityZ > 0.0) {
            currentPhase = FlightPhase::BOOST;
        }
        else if (currentPhase == FlightPhase::BOOST && dynamicPressure > 3000.0) {
            currentPhase = FlightPhase::MAX_Q;
        }
        else if (currentPhase == FlightPhase::MAX_Q && isBurnout()) {
            currentPhase = FlightPhase::BURNOUT;
        }
        else if (currentPhase == FlightPhase::BURNOUT && velocityZ > 0.0) {
            currentPhase = FlightPhase::COAST;
        }
        else if (currentPhase == FlightPhase::COAST && velocityZ <= 0.0) {
            currentPhase = FlightPhase::APOGEE;
        }
        else if (currentPhase == FlightPhase::APOGEE) {
            currentPhase = FlightPhase::DESCENT;
        }
        else if (currentPhase == FlightPhase::DESCENT && positionZ <= 0.0) {
            currentPhase = FlightPhase::LANDED;
            positionZ = 0.0;
            velocityZ = 0.0;
        }

        if (currentPhase == FlightPhase::LANDED) return;
    }

    double Vehicle::getMass() const { return currentMass; }
    double Vehicle::getVelocity() const { return std::sqrt(velocityX*velocityX + velocityY*velocityY + velocityZ*velocityZ); }
    double Vehicle::getAltitude() const { return positionZ; }
    double Vehicle::getPositionX() const { return positionX; }
    double Vehicle::getPositionY() const { return positionY; }
    bool Vehicle::isBurnout() const { return propellantMass <= 0.0; }
    FlightPhase Vehicle::getPhase() const { return currentPhase; }

}