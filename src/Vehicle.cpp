#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace RTS {

    constexpr double GM_EARTH = 3.986004418e14;
    constexpr double R_EARTH = 6371000.0;

    Vehicle::Vehicle(std::vector<Stage> stages, double payloadMass,
                  double bodyDiameter, double fairingDiameter, double noseLength,
                  int finCount, double finSpan, double finRootChord,
                  double finTipChord, double finSweepDistance, double finPosition)
    : stages(std::move(stages)), currentStageIndex(0),
    payloadMass(payloadMass),
    bodyDiameter(bodyDiameter), fairingDiameter(fairingDiameter), noseLength(noseLength),
    finCount(finCount), finSpan(finSpan), finRootChord(finRootChord),
    finTipChord(finTipChord), finSweepDistance(finSweepDistance), finPosition(finPosition),
    finCnAlpha(computeFinCnAlpha(finCount, finSpan, finRootChord, finTipChord, finSweepDistance, bodyDiameter)),
    noseAero(computeNoseAero(bodyDiameter)),
    currentMass(this->stages[0].dryMass + this->stages[0].propellantMass + payloadMass),
    positionX(0.0), positionY(0.0), positionZ(0.0),
    velocityX(0.0), velocityY(0.0), velocityZ(0.0),
    currentPhase(FlightPhase::PRE_LAUNCH), impactVelocity(0.0) {}

    Derivative Vehicle::computeDerivative(const State& s, double airDensity, double speedOfSound) const {
        const Stage& stage = stages[currentStageIndex];
        bool burning = s.mass > stage.dryMass;

        const double refArea = 10.8;

        double pitchAngle = 0.0;
        if (s.z > 50.0 && burning) {
            double turnProgress = std::min((s.z - 50.0) / 120000.0, 1.0);
            pitchAngle = turnProgress * 85.0 * M_PI / 180.0;
        }
        double yawAngle = 0.0;

        double thrustX = burning ? stage.thrust * std::sin(pitchAngle) * std::cos(yawAngle) : 0.0;
        double thrustY = burning ? stage.thrust * std::sin(pitchAngle) * std::sin(yawAngle) : 0.0;
        double thrustZ = burning ? stage.thrust * std::cos(pitchAngle) : 0.0;

        double speed = std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
        double cd = dragModel.getCd(speed, speedOfSound);
        double dragForce = 0.5 * airDensity * speed * speed * cd * refArea;
        double dragX = (speed > 0) ? dragForce * (s.vx / speed) : 0.0;
        double dragY = (speed > 0) ? dragForce * (s.vy / speed) : 0.0;
        double dragZ = (speed > 0) ? dragForce * (s.vz / speed) : 0.0;

        double accX = (thrustX - dragX) / s.mass;
        double accY = (thrustY - dragY) / s.mass;
        double r = R_EARTH + s.z;
        double localG = GM_EARTH / (r * r);
        double accZ = (thrustZ - dragZ - s.mass * localG) / s.mass;

        double massFlowRate = burning ? -(stage.thrust / (stage.isp * g0)) : 0.0;

        return Derivative{s.vx, s.vy, s.vz, accX, accY, accZ, massFlowRate};
    }

    bool Vehicle::hasEscapedGravity(double vx, double vy, double vz, double altitude) const {
        double r = R_EARTH + altitude;
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        double vEscape = std::sqrt(2 * GM_EARTH / r);
        return speed >= vEscape;
    }

    double Vehicle::getOrbitalVelocity(double altitude) const {
        double r = R_EARTH + altitude;
        return std::sqrt(GM_EARTH / r);
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

        double massLost = (stages[currentStageIndex].dryMass + stages[currentStageIndex].propellantMass + payloadMass) - currentMass;
        stages[currentStageIndex].propellantMass = std::max(0.0, stages[currentStageIndex].propellantMass - massLost);
        if (currentMass < stages[currentStageIndex].dryMass + payloadMass) currentMass = stages[currentStageIndex].dryMass + payloadMass;

        double speed = std::sqrt(velocityX*velocityX + velocityY*velocityY + velocityZ*velocityZ);
        double dynamicPressure = 0.5 * airDensity * speed * speed;

        if (currentPhase == FlightPhase::PRE_LAUNCH && velocityZ > 0.0) {
            currentPhase = FlightPhase::BOOST;
        }
        else if (currentPhase == FlightPhase::BOOST && dynamicPressure > 3000.0) {
            currentPhase = FlightPhase::MAX_Q;
        }
        else if ((currentPhase == FlightPhase::BOOST || currentPhase == FlightPhase::MAX_Q) && isBurnout()) {
            if (currentStageIndex + 1 < static_cast<int>(stages.size())) {
                currentPhase = FlightPhase::STAGE_SEPARATION;
                currentMass -= stages[currentStageIndex].dryMass;
                currentStageIndex++;
                currentMass += stages[currentStageIndex].dryMass + stages[currentStageIndex].propellantMass;
                currentPhase = FlightPhase::BOOST;
            } else {
                currentPhase = FlightPhase::BURNOUT;
            }
        }
        else if (currentPhase == FlightPhase::BURNOUT && velocityZ > 0.0) {
            currentPhase = FlightPhase::COAST;
        }
        else if (currentPhase == FlightPhase::BURNOUT && velocityZ <= 0.0) {
            currentPhase = FlightPhase::DESCENT;
        }
        else if (currentPhase == FlightPhase::COAST && velocityZ <= 0.0) {
            currentPhase = FlightPhase::APOGEE;
        }
        else if (currentPhase == FlightPhase::APOGEE) {
            currentPhase = FlightPhase::DESCENT;
        }
        else if (currentPhase == FlightPhase::DESCENT && positionZ <= 0.0) {
            impactVelocity = std::sqrt(velocityX*velocityX + velocityY*velocityY + velocityZ*velocityZ);
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
    bool Vehicle::isBurnout() const {
    return stages[currentStageIndex].propellantMass <= 0.0;
    }
    double Vehicle::getImpactVelocity() const { return impactVelocity; }
    FlightPhase Vehicle::getPhase() const { return currentPhase; }
    double Vehicle::getStabilityMargin() const {
        double xNose = computeNoseCP(noseLength);
        double xFin = computeFinCP(finPosition, finRootChord, finTipChord, finSweepDistance);
        double xCp = computeCP(noseAero.cnAlphaNose, xNose, finCnAlpha, xFin);
        double totalLength = noseLength + finPosition + finRootChord; // rough estimate, adjust if you track exact length
        double xCg = 0.55 * totalLength;
        return computeStabilityMargin(xCp, xCg, bodyDiameter);
    }

}