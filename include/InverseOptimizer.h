#pragma once
#include <vector>
#include <cmath>
#include <iostream>
#include "Vehicle.h"
#include "ConfigLoader.h"
#include "FaultDetector.h"

namespace RTS {

constexpr double OPT_GM_EARTH = 3.986004418e14;
constexpr double OPT_R_EARTH = 6371000.0;

// Returns the altitude at which the vehicle first reaches orbital velocity
// (insertion altitude), or -1.0 if it never reaches orbit within the run.
inline double getOrbitalInsertionAltitude(const std::vector<TelemetryPoint>& trajectory) {
    for (const auto& pt : trajectory) {
        double r = OPT_R_EARTH + pt.altitude;
        double orbitalV = std::sqrt(OPT_GM_EARTH / r);
        if (pt.velocity >= orbitalV) {
            return pt.altitude;
        }
    }
    return -1.0;
}

// Binary search on propellant mass of the final stage to hit a target
// orbital insertion altitude. Uses the full multi-stage vehicle as-is,
// since the pitch/gravity-turn program (see Vehicle::computeDerivative)
// is designed for orbital insertion, not raw ballistic apogee. More
// propellant on the final stage delays burnout, giving more time to climb
// before reaching orbital velocity, so insertion altitude increases
// monotonically with propellant mass — this is what makes bisection valid.
inline double optimizePropellantMass(VehicleConfig cfg, int stageIndex, double targetAltitude,
                                      double toleranceMeters = 1000.0, int maxIterations = 20) {
    double lowMass = cfg.stages[stageIndex].propellantMass * 0.5;
    double highMass = cfg.stages[stageIndex].propellantMass * 2.0;

    for (int iter = 0; iter < maxIterations; ++iter) {
        double midMass = (lowMass + highMass) / 2.0;
        cfg.stages[stageIndex].propellantMass = midMass;

        auto trajectory = runSimulation(cfg);
        double insertionAlt = getOrbitalInsertionAltitude(trajectory);

        if (insertionAlt < 0.0) {
            // Never reached orbit within this run — treat as "too low",
            // so bisection pushes toward more propellant next iteration.
            std::cout << "[ITER " << iter << "] propellantMass=" << midMass
                      << " kg -> did not reach orbital velocity" << std::endl;
            lowMass = midMass;
            continue;
        }

        double error = insertionAlt - targetAltitude;
        std::cout << "[ITER " << iter << "] propellantMass=" << midMass
                  << " kg -> insertion altitude=" << insertionAlt << " m (target=" << targetAltitude
                  << " m, error=" << error << " m)" << std::endl;

        if (std::abs(error) <= toleranceMeters) {
            std::cout << "\n[CONVERGED] propellantMass=" << midMass << " kg gives insertion altitude="
                      << insertionAlt << " m (within " << toleranceMeters << " m of target)" << std::endl;
            return midMass;
        }

        if (insertionAlt < targetAltitude) {
            lowMass = midMass;
        } else {
            highMass = midMass;
        }
    }

    std::cout << "\n[NOT CONVERGED] Max iterations reached. Best guess: "
              << ((lowMass + highMass) / 2.0) << " kg" << std::endl;
    return (lowMass + highMass) / 2.0;
}

}