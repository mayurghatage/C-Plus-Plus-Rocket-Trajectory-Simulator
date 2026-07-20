#pragma once
#include <vector>
#include "Integrator.h"
#include "DragModel.h"
#include "Aerodynamics.h"

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
        Vehicle(std::vector<Stage> stages, double payloadMass,
            double bodyDiameter, double fairingDiameter, double noseLength,
            int finCount, double finSpan, double finRootChord,
            double finTipChord, double finSweepDistance, double finPosition, double totalLength);

        void update(double dt, double airDensity, double speedOfSound);

        double getMass() const;
        double getVelocity() const;
        double getAltitude() const;
        double getPositionX() const;
        double getPositionY() const;
        bool isBurnout() const;
        double getImpactVelocity() const;
        bool hasEscapedGravity(double vx, double vy, double vz, double altitude) const;
        double getOrbitalVelocity(double altitude) const;
        double getStabilityMargin() const;

        void applyThrustMultiplier(int stageIndex, double multiplier);
        void applyIspMultiplier(int stageIndex, double multiplier);
        void applyStageSeparationDelay(double extraSeconds);

        FlightPhase getPhase() const;
        int getCurrentStageIndex() const { return currentStageIndex; }
        int getTotalStages() const { return static_cast<int>(stages.size()); }
        int getFinCount() const { return finCount; }

    private:
        std::vector<Stage> stages;
        int currentStageIndex;

        double payloadMass;
        double parachuteDeployAltitude;
        double parachuteCd;
        double parachuteArea;
        double bodyDiameter;
        double fairingDiameter;
        double noseLength;
        int finCount;
        double finSpan;
        double finRootChord;
        double finTipChord;
        double finSweepDistance;
        double finPosition;
        double totalLength;
        std::vector<double> stagePositions;
        double payloadPosition;
        double finCnAlpha;
        AeroCoefficients noseAero;
        double currentMass;
        double positionX, positionY, positionZ;
        double velocityX, velocityY, velocityZ;
        double impactVelocity;
        double separationDelay = 0.0;

        DragModel dragModel;

        Derivative computeDerivative(const State& s, double airDensity, double speedOfSound) const;

        static constexpr double g0 = 9.80665;

        FlightPhase currentPhase;
    };

};