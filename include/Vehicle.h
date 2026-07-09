#pragma once
#include <vector>
#include "Integrator.h"
#include "DragModel.h"

namespace RTS {

    struct Stage {
        double dryMass;
        double propellantMass;
        double thrust;
        double isp;
        double burnTime;
    };

    class Vehicle {
    public:
        Vehicle(std::vector<Stage> stages, double payloadMass);

        void update(double dt, double airDensity, double speedOfSound);

        double getMass() const;
        double getVelocity() const;
        double getAltitude() const;
        double getPositionX() const;
        double getPositionY() const;
        bool isBurnout() const;

        FlightPhase getPhase() const;

    private:
        std::vector<Stage> stages;
        int currentStageIndex;

        double payloadMass;
        double parachuteDeployAltitude;
        double parachuteCd;
        double parachuteArea;
        double currentMass;
        double positionX, positionY, positionZ;
        double velocityX, velocityY, velocityZ;

        DragModel dragModel;

        Derivative computeDerivative(const State& s, double airDensity, double speedOfSound) const;

        static constexpr double g0 = 9.80665;

        FlightPhase currentPhase;
    };

}