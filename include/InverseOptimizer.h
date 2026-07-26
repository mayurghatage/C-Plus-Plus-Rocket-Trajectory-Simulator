#pragma once
#include <vector>
#include <cmath>
#include <iostream>
#include "Vehicle.h"
#include "ConfigLoader.h"
#include "FaultDetector.h"

namespace RTS {

// Runs a sim and returns the peak altitude reached (apogee), regardless of
// how the run ends (landed, orbit, timeout). Reuses runSimulation so the
// optimizer always tests against the same physics as everything else.
inline double getApogee(const std::vector<TelemetryPoint>& trajectory) {
    double maxAlt = 0.0;
    for (const auto& pt : trajectory) {
        if (pt.altitude > maxAlt) maxAlt = pt.altitude;
    }
    return maxAlt;
}

// Binary search on propellant mass of one stage to hit a target apogee.
// Apogee increases monotonically with propellant mass over the practical
// range, so bisection converges reliably without needing gradients.
inline double optimizePropellantMass(VehicleConfig cfg, int stageIndex, double targetApogee,
                                      double toleranceMeters = 1000.0, int maxIterations = 20) {
    double lowMass = cfg.stages[stageIndex].propellantMass * 0.5;
    double highMass = cfg.stages[stageIndex].propellantMass * 2.0;

    for (int iter = 0; iter < maxIterations; ++iter) {
        double midMass = (lowMass + highMass) / 2.0;
        cfg.stages[stageIndex].propellantMass = midMass;

        auto trajectory = runSimulation(cfg);
        double apogee = getApogee(trajectory);
        double error = apogee - targetApogee;

        std::cout << "[ITER " << iter << "] propellantMass=" << midMass
                  << " kg -> apogee=" << apogee << " m (target=" << targetApogee
                  << " m, error=" << error << " m)" << std::endl;

        if (std::abs(error) <= toleranceMeters) {
            std::cout << "\n[CONVERGED] propellantMass=" << midMass << " kg gives apogee="
                      << apogee << " m (within " << toleranceMeters << " m of target)" << std::endl;
            return midMass;
        }

        if (apogee < targetApogee) {
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