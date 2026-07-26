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

    // Validate the bracket before searching: if the upper bound itself never
    // reaches orbit (e.g. too much mass slows climb past the sim timeout),
    // bisection has no valid solution inside this range and will converge
    // on garbage. Warn rather than silently returning a bad answer.
    {
        VehicleConfig testCfg = cfg;
        testCfg.stages[stageIndex].propellantMass = highMass;
        auto testTrajectory = runSimulation(testCfg);
        if (getOrbitalInsertionAltitude(testTrajectory) < 0.0) {
            std::cout << "[WARNING] Upper bound propellantMass=" << highMass
                      << " kg does not reach orbit within sim time. "
                      << "Target may be unreachable with this search range." << std::endl;
        }
    }

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

            // Sensitivity check: real aerospace practice — after finding a nominal
            // design point, check how much a small manufacturing/loading tolerance
            // (±5%) shifts the outcome. High sensitivity flags a fragile design.
            VehicleConfig lowSensCfg = cfg;
            lowSensCfg.stages[stageIndex].propellantMass = midMass * 0.95;
            double lowSensAlt = getOrbitalInsertionAltitude(runSimulation(lowSensCfg));

            VehicleConfig highSensCfg = cfg;
            highSensCfg.stages[stageIndex].propellantMass = midMass * 1.05;
            double highSensAlt = getOrbitalInsertionAltitude(runSimulation(highSensCfg));

            std::cout << "[SENSITIVITY] -5% propellant (" << (midMass * 0.95) << " kg) -> "
                      << (lowSensAlt < 0.0 ? "did not reach orbit" : std::to_string(lowSensAlt) + " m") << std::endl;
            std::cout << "[SENSITIVITY] +5% propellant (" << (midMass * 1.05) << " kg) -> "
                      << (highSensAlt < 0.0 ? "did not reach orbit" : std::to_string(highSensAlt) + " m") << std::endl;

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

// Binary search on thrust of the final stage to hit a target orbital
// insertion altitude, holding propellant mass fixed. More thrust means
// faster acceleration and reaching orbital velocity sooner — but it also
// increases mass flow rate (massFlowRate = thrust / (Isp * g0)), burning
// fuel faster. Net effect on insertion altitude is monotonic increasing
// for typical thrust ranges (faster climb dominates faster fuel burn),
// same bisection logic as propellant mass optimization above.
inline double optimizeThrust(VehicleConfig cfg, int stageIndex, double targetAltitude,
                              double toleranceMeters = 1000.0, int maxIterations = 20) {
    double lowThrust = cfg.stages[stageIndex].thrust * 0.5;
    double highThrust = cfg.stages[stageIndex].thrust * 2.0;

    {
        VehicleConfig testCfg = cfg;
        testCfg.stages[stageIndex].thrust = highThrust;
        auto testTrajectory = runSimulation(testCfg);
        if (getOrbitalInsertionAltitude(testTrajectory) < 0.0) {
            std::cout << "[WARNING] Upper bound thrust=" << highThrust
                      << " N does not reach orbit within sim time. "
                      << "Target may be unreachable with this search range." << std::endl;
        }
    }

    for (int iter = 0; iter < maxIterations; ++iter) {
        double midThrust = (lowThrust + highThrust) / 2.0;
        cfg.stages[stageIndex].thrust = midThrust;

        auto trajectory = runSimulation(cfg);
        double insertionAlt = getOrbitalInsertionAltitude(trajectory);

        if (insertionAlt < 0.0) {
            std::cout << "[ITER " << iter << "] thrust=" << midThrust
                      << " N -> did not reach orbital velocity" << std::endl;
            lowThrust = midThrust;
            continue;
        }

        double error = insertionAlt - targetAltitude;
        std::cout << "[ITER " << iter << "] thrust=" << midThrust
                  << " N -> insertion altitude=" << insertionAlt << " m (target=" << targetAltitude
                  << " m, error=" << error << " m)" << std::endl;

        if (std::abs(error) <= toleranceMeters) {
            std::cout << "\n[CONVERGED] thrust=" << midThrust << " N gives insertion altitude="
                      << insertionAlt << " m (within " << toleranceMeters << " m of target)" << std::endl;
            return midThrust;
        }

        if (insertionAlt < targetAltitude) {
            lowThrust = midThrust;
        } else {
            highThrust = midThrust;
        }
    }

    std::cout << "\n[NOT CONVERGED] Max iterations reached. Best guess: "
              << ((lowThrust + highThrust) / 2.0) << " N" << std::endl;
    return (lowThrust + highThrust) / 2.0;
}

}