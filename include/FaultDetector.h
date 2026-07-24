#pragma once
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
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

struct MissionEvents {
    double stageSeparationTime = -1.0;
    double burnoutTime = -1.0;
    double orbitalInsertionTime = -1.0;
};

// Returns a copy of cfg with one stage's thrust reduced by percentReduction
// (e.g. 0.10 = 10% thrust loss). Used to simulate an engine underperforming,
// so we have a controlled "actual" run to test the fault detector against.
inline VehicleConfig injectThrustFault(const VehicleConfig& cfg, int stageIndex, double percentReduction) {
    VehicleConfig faulty = cfg;
    faulty.stages[stageIndex].thrust *= (1.0 - percentReduction);
    return faulty;
}

inline VehicleConfig injectIspFault(const VehicleConfig& cfg, int stageIndex, double percentReduction) {
    VehicleConfig faulty = cfg;
    faulty.stages[stageIndex].isp *= (1.0 - percentReduction);
    return faulty;
}

// Runs a full simulation from a vehicle config and returns the trajectory,
// one TelemetryPoint per timestep. Used for both nominal and actual runs
// in fault detection — same simulation loop, different input config.
inline std::vector<TelemetryPoint> runSimulation(const VehicleConfig& cfg, double dt = 0.1, double maxTime = 1000.0) {
    std::vector<TelemetryPoint> trajectory;

    AtmosphereModel atmo;
    Vehicle rocket(cfg.stages, cfg.payloadMass, cfg.bodyDiameter, cfg.fairingDiameter, cfg.noseLength,
                   cfg.finCount, cfg.finSpan, cfg.finRootChord, cfg.finTipChord, cfg.finSweepDistance, cfg.finPosition,
                   cfg.totalLength);

    double simTime = 0.0;
    while (simTime < maxTime) {
        AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density, props.speedOfSound, simTime);
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

// Isp degradation fault: engine loses specific impulse efficiency mid-flight,
// only once the vehicle reaches faultStartTime AND is burning on stageIndex.
inline std::vector<TelemetryPoint> runSimulationWithDelayedIspFault(const VehicleConfig& cfg, int stageIndex,
                                                                  double percentReduction, double faultStartTime,
                                                                  double dt = 0.1, double maxTime = 1000.0) {
    std::vector<TelemetryPoint> trajectory;
    AtmosphereModel atmo;
    Vehicle rocket(cfg.stages, cfg.payloadMass, cfg.bodyDiameter, cfg.fairingDiameter, cfg.noseLength,
                   cfg.finCount, cfg.finSpan, cfg.finRootChord, cfg.finTipChord, cfg.finSweepDistance, cfg.finPosition,
                   cfg.totalLength);

    double simTime = 0.0;
    bool faultApplied = false;

    while (simTime < maxTime) {
        if (!faultApplied && simTime >= faultStartTime && rocket.getCurrentStageIndex() == stageIndex) {
            rocket.applyIspMultiplier(stageIndex, 1.0 - percentReduction);
            faultApplied = true;
        }

        AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density, props.speedOfSound, simTime);
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

// Failed-separation fault: stage separation is delayed by extraSeconds once
// faultStartTime is reached. No config-copy injector needed since the delay
// is a runtime Vehicle member, not a Stage/VehicleConfig field — we mutate
// the live rocket object directly, same pattern as applyIspMultiplier above.
inline std::vector<TelemetryPoint> runSimulationWithDelayedSeparationFault(const VehicleConfig& cfg,
                                                                  double extraSeconds, double faultStartTime,
                                                                  double dt = 0.1, double maxTime = 1000.0) {
    std::vector<TelemetryPoint> trajectory;
    AtmosphereModel atmo;
    Vehicle rocket(cfg.stages, cfg.payloadMass, cfg.bodyDiameter, cfg.fairingDiameter, cfg.noseLength,
                   cfg.finCount, cfg.finSpan, cfg.finRootChord, cfg.finTipChord, cfg.finSweepDistance, cfg.finPosition,
                   cfg.totalLength);

    double simTime = 0.0;
    bool faultApplied = false;

    while (simTime < maxTime) {
        if (!faultApplied && simTime >= faultStartTime) {
            rocket.applyStageSeparationDelay(extraSeconds);
            faultApplied = true;
        }

        AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density, props.speedOfSound, simTime);
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

inline MissionEvents extractMissionEvents(const std::vector<TelemetryPoint>& trajectory) {
    MissionEvents events;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        if (events.burnoutTime < 0.0 && pt.phase == FlightPhase::BURNOUT) {
            events.burnoutTime = pt.time;
        }
    }
    if (!trajectory.empty()) {
        const auto& last = trajectory.back();
        double r = 6371000.0 + last.altitude;
        double orbitalV = std::sqrt(3.986004418e14 / r);
        if (last.velocity >= orbitalV) {
            events.orbitalInsertionTime = last.time;
        }
    }
    return events;
}

inline void compareMissionEvents(const MissionEvents& nominal, const MissionEvents& actual,
                                  double timeThreshold = 5.0) {
    std::cout << "\n[MISSION EVENT COMPARISON]" << std::endl;

    auto checkEvent = [&](const std::string& name, double nomTime, double actTime) {
        if (nomTime < 0.0 && actTime < 0.0) {
            std::cout << "  " << name << ": not reached in either run" << std::endl;
            return;
        }
        if (nomTime < 0.0 || actTime < 0.0) {
            std::cout << "  [FLAG] " << name << ": reached in one run but not the other "
                      << "(nominal=" << nomTime << "s, actual=" << actTime << "s)" << std::endl;
            return;
        }
        double delta = actTime - nomTime;
        if (std::abs(delta) > timeThreshold) {
            std::cout << "  [FLAG] " << name << ": nominal=" << nomTime << "s, actual=" << actTime
                      << "s, delta=" << delta << "s (exceeds " << timeThreshold << "s threshold)" << std::endl;
        } else {
            std::cout << "  " << name << ": nominal=" << nomTime << "s, actual=" << actTime
                      << "s, delta=" << delta << "s (within threshold)" << std::endl;
        }
    };

    checkEvent("Stage separation", nominal.stageSeparationTime, actual.stageSeparationTime);
    checkEvent("Burnout", nominal.burnoutTime, actual.burnoutTime);
    checkEvent("Orbital insertion", nominal.orbitalInsertionTime, actual.orbitalInsertionTime);
}

}