#pragma once
#include <string>
#include <vector>
#include "Vehicle.h"
#include "Atmosphere.h"
#include "ConfigLoader.h"

namespace RTS {

struct TelemetryPoint {
    double time;
    double altitude;
    double velocity;
    double mass;
    FlightPhase phase;
};

inline VehicleConfig injectThrustFault(const VehicleConfig& cfg, int stageIndex, double percentReduction) {
    VehicleConfig faulty = cfg;
    faulty.stages[stageIndex].thrust *= (1.0 - percentReduction);
    return faulty;
}

inline std::vector<TelemetryPoint> runSimulation(const VehicleConfig& cfg, double dt = 0.1, double maxTime = 1000.0) {
    std::vector<TelemetryPoint> trajectory;

    AtmosphereModel atmo;
    Vehicle rocket(cfg.stages, cfg.payloadMass, cfg.bodyDiameter, cfg.fairingDiameter, cfg.noseLength,
                   cfg.finCount, cfg.finSpan, cfg.finRootChord, cfg.finTipChord, cfg.finSweepDistance, cfg.finPosition,
                   cfg.totalLength);

    double simTime = 0.0;
    while (simTime < maxTime) {
        AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density, props.speedOfSound);
        simTime += dt;

        trajectory.push_back({simTime, rocket.getAltitude(), rocket.getVelocity(), rocket.getMass(), rocket.getPhase()});

        bool onFinalStage = (rocket.getCurrentStageIndex() == rocket.getTotalStages() - 1);
        if (onFinalStage) {
            bool reachedOrbit = rocket.hasEscapedGravity(0.0, 0.0, rocket.getVelocity(), rocket.getAltitude())
                              || rocket.getVelocity() >= rocket.getOrbitalVelocity(rocket.getAltitude());
            if (reachedOrbit) break;
        }

        if (rocket.getAltitude() <= 0.0 && rocket.getPhase() == FlightPhase::DESCENT) break;
        if (rocket.getPhase() == FlightPhase::LANDED) break;
    }

return trajectory;
}

inline void compareTrajectories(const std::vector<TelemetryPoint>& nominal,
                                 const std::vector<TelemetryPoint>& actual,
                                 double deviationThreshold = 0.05) {
    size_t pointCount = std::min(nominal.size(), actual.size());
    int anomalyCount = 0;

    for (size_t i = 0; i < pointCount; ++i) {
        const TelemetryPoint& nom = nominal[i];
        const TelemetryPoint& act = actual[i];

        double altDeviation = (nom.altitude != 0.0)
            ? std::abs(act.altitude - nom.altitude) / std::abs(nom.altitude)
            : 0.0;
        double velDeviation = (nom.velocity != 0.0)
            ? std::abs(act.velocity - nom.velocity) / std::abs(nom.velocity)
            : 0.0;

        if (altDeviation > deviationThreshold || velDeviation > deviationThreshold) {
            std::cout << "[ANOMALY] t=" << act.time
                      << "s | alt deviation=" << (altDeviation * 100.0) << "%"
                      << " | vel deviation=" << (velDeviation * 100.0) << "%"
                      << " | nominal alt=" << nom.altitude << " actual alt=" << act.altitude
                      << std::endl;
            anomalyCount++;
        }
    }

    std::cout << "\n[FAULT DETECTION SUMMARY] " << anomalyCount << " anomalies detected out of "
              << pointCount << " compared timesteps." << std::endl;
}

}