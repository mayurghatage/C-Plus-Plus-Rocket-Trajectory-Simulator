#pragma once
#include "Integrator.h"

namespace RTS {

    class Vehicle {
    public:
        Vehicle(double dryMass, double propellantMass, double thrust, double isp);

        void update(double dt, double airDensity);

        double getMass() const;
        double getVelocity() const;
        double getAltitude() const;
        bool isBurnout() const;

        FlightPhase getPhase() const;

    private:
        double dryMass;
        double propellantMass;
        double thrust;
        double isp;

        double currentMass;
        double velocity;
        double altitude;

        Derivative computeDerivative(const State& s, double airDensity) const;

        static constexpr double g0 = 9.80665;

        FlightPhase currentPhase;
    };

}