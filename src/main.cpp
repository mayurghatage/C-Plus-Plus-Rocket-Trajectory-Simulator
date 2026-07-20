#include <iostream>
#include <string>
#include <fstream>
#include "Atmosphere.h"
#include "Vehicle.h"
#include "ConfigLoader.h"
#include "Aerodynamics.h"
#include "FaultDetector.h"

std::string phaseToString(RTS::FlightPhase phase) {
    switch (phase) {
        case RTS::FlightPhase::PRE_LAUNCH:        return "PRE_LAUNCH";
        case RTS::FlightPhase::BOOST:             return "BOOST";
        case RTS::FlightPhase::MAX_Q:             return "MAX_Q";
        case RTS::FlightPhase::STAGE_SEPARATION:  return "STAGE_SEPARATION";
        case RTS::FlightPhase::BURNOUT:           return "BURNOUT";
        case RTS::FlightPhase::COAST:             return "COAST";
        case RTS::FlightPhase::APOGEE:            return "APOGEE";
        case RTS::FlightPhase::DESCENT:           return "DESCENT";
        case RTS::FlightPhase::TERMINAL_VELOCITY: return "TERMINAL_VELOCITY";
        case RTS::FlightPhase::LANDED:            return "LANDED";
        default:                                   return "UNKNOWN";
    }
}

int main(int argc, char* argv[]) {
    #ifdef _Win32
        std::system("cls");
    #else
        std::system("clear");
    #endif

    std::cout << "================================================================" << std::endl;
    std::cout << "    ASTRA // C++ ROCKET TRAJECTORY SIMULATOR -- BUILD 0.2.0     " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "              INITIALIZING SYSTEM SUBSYSTEMS..."                  << std::endl;
    std::cout << " ---------------------------------------------------------------" << std::endl;
    std::cout << " [INFO] Atmospheric profile module loaded. ISA 1976 active."      << std::endl;
    std::cout << " ---------------------------------------------------------------" << std::endl;
    std::cout << "================================================================" << std::endl;

    // Atmosphere test
    RTS::AtmosphereModel atmo;
    const double testAltitudes[] = {0, 5000, 11000, 20000, 35000, 50000};

    std::cout << "\n--- ATMOSPHERE MODULE TEST ---" << std::endl;
    for (double alt : testAltitudes) {
        RTS::AirProperties props = atmo.calculateState(alt);
        std::cout << "Alt: " << alt << " m"
                  << " | T: "   << props.temperature << " K"
                  << " | P: "   << props.pressure    << " Pa"
                  << " | rho: " << props.density     << " kg/m3" << std::endl;
    }

    std::cout << "\n--- VEHICLE MODULE TEST ---" << std::endl;

    VehicleConfig cfg;
    std::string configPath = (argc > 1) ? argv[1] : "configs/falcon9.json";
    try {
        cfg = loadVehicleConfig(configPath);
    } catch (const std::exception& e) {
        std::cerr << "Config load failed: " << e.what() << std::endl;
        return 1;
    }
    RTS::Vehicle rocket(cfg.stages, cfg.payloadMass, cfg.bodyDiameter, cfg.fairingDiameter, cfg.noseLength,
                        cfg.finCount, cfg.finSpan, cfg.finRootChord, cfg.finTipChord, cfg.finSweepDistance, cfg.finPosition,
                        cfg.totalLength);
    
    std::cout << "\n--- FAULT DETECTION TEST ---" << std::endl;

    std::vector<RTS::TelemetryPoint> nominalTrajectory = RTS::runSimulation(cfg);
    std::vector<RTS::TelemetryPoint> actualTrajectory = RTS::runSimulationWithDelayedIspFault(cfg, 0, 0.10, 60.0);

    std::cout << "Nominal run: " << nominalTrajectory.size() << " points | "
              << "Actual run: " << actualTrajectory.size() << " points" << std::endl;

    RTS::compareTrajectories(nominalTrajectory, actualTrajectory);
    std::cout << "--- END FAULT DETECTION TEST ---\n" << std::endl;


    double dt = 0.1;
    double simTime = 0.0;

    // CSV telemetry export
    std::ofstream telemetryFile("astra_telemetry.csv");
    telemetryFile << "time,x,y,altitude,velocity,mach,mass,phase\n";

    // Mission summary tracking
    double maxAltitude = 0.0;
    double maxVelocity = 0.0;
    double maxQ = 0.0;
    double burnoutTime = 0.0;
    double burnoutAltitude = 0.0;
    double burnoutVelocity = 0.0;
    int lastLoggedStage = 0; 
    bool missionSuccessExit = false; 

    while (simTime < 1000.0) {
        RTS::AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density, props.speedOfSound);
        simTime += dt;

        if (rocket.getCurrentStageIndex() > lastLoggedStage) {
            lastLoggedStage = rocket.getCurrentStageIndex();
            std::cout << "[STAGE SEPARATION] Stage " << lastLoggedStage
                      << " ignited at t=" << simTime << "s, alt=" << rocket.getAltitude() << " m" << std::endl;
        }

    double mach = std::abs(rocket.getVelocity()) / props.speedOfSound;

        // Write telemetry to CSV every frame
        telemetryFile << simTime << ","
                << rocket.getPositionX() << ","
                << rocket.getPositionY() << ","
                << rocket.getAltitude() << ","
                << rocket.getVelocity() << ","
                << mach << ","
                << rocket.getMass() << ","
                << phaseToString(rocket.getPhase()) << "\n";

        // Track mission statistics
        if (rocket.getAltitude() > maxAltitude) maxAltitude = rocket.getAltitude();
        if (std::abs(rocket.getVelocity()) > maxVelocity) maxVelocity = std::abs(rocket.getVelocity());

        double airDensityNow = props.density;
        double q = 0.5 * airDensityNow * rocket.getVelocity() * rocket.getVelocity();
        if (q > maxQ) maxQ = q;

        if (rocket.getPhase() == RTS::FlightPhase::BURNOUT && burnoutTime == 0.0) {
            burnoutTime = simTime;
            burnoutAltitude = rocket.getAltitude();
            burnoutVelocity = std::abs(rocket.getVelocity());
        }

        bool onFinalStage = (rocket.getCurrentStageIndex() == rocket.getTotalStages() - 1);
        if (onFinalStage) {
            bool reachedOrbit = rocket.hasEscapedGravity(0.0, 0.0, rocket.getVelocity(), rocket.getAltitude())
                              || rocket.getVelocity() >= rocket.getOrbitalVelocity(rocket.getAltitude());
            if (reachedOrbit) {
                std::cout << "[MISSION SUCCESS] Payload reached orbital/escape velocity at t="
                          << simTime << "s, alt=" << rocket.getAltitude() << " m" << std::endl;
                missionSuccessExit = true;
                break;
            }
        }

        if (rocket.getAltitude() <= 0.0 && rocket.getPhase() == RTS::FlightPhase::DESCENT) {
            std::cout << "[MISSION COMPLETE] Vehicle impacted ground at t=" << simTime << "s" << std::endl;
            break;
        }

        if (static_cast<int>(simTime * 10) % 10 == 0) {
            std::cout << "t=" << simTime << "s | alt=" << rocket.getAltitude()
                      << " m | v=" << rocket.getVelocity()
                      << " m/s | mass=" << rocket.getMass() 
                      << " kg | stage=" << (rocket.getCurrentStageIndex() + 1) << "/" << rocket.getTotalStages()
                      << " | phase=" << phaseToString(rocket.getPhase()) << std::endl;
        }

        if (rocket.getPhase() == RTS::FlightPhase::LANDED) {
            std::cout << "[MISSION COMPLETE] Vehicle landed." << std::endl;
            break;
        }
    }
    // Close CSV file
    telemetryFile.close();

    double finalVelocity = std::abs(rocket.getVelocity());
    double finalAltitude = rocket.getAltitude();

    double landingVelocity = rocket.getImpactVelocity();
    std::string landingStatus;
    if (missionSuccessExit) {
        landingStatus = "N/A - Final Stage Reached Orbit (Not Tracked Post-Insertion)";
    } else if (rocket.getPhase() != RTS::FlightPhase::LANDED) {
        landingStatus = "SIMULATION ENDED - DID NOT LAND (still falling)";
    } else if (landingVelocity < 5.0) {
        landingStatus = "SOFT LANDING";
    } else {
        landingStatus = "HARD IMPACT / CRASH";
    }

    double escapeVelocity = rocket.getOrbitalVelocity(finalAltitude) * std::sqrt(2.0);
    double orbitalVelocity = rocket.getOrbitalVelocity(finalAltitude);
    bool escaped = rocket.hasEscapedGravity(0.0, 0.0, finalVelocity, finalAltitude);
    std::string velocityVerdict = escaped ? "EXCEEDS ESCAPE VELOCITY"
                                : (finalVelocity >= orbitalVelocity) ? "EXCEEDS ORBITAL VELOCITY"
                                : "SUBORBITAL";

    // Mission summary
    std::cout << "\n================================================================" << std::endl;
    std::cout << "                    MISSION SUMMARY                               " << std::endl;
    std::cout << "==================================================================" << std::endl;
    std::cout << " Max Altitude    : " << maxAltitude    << " m"                      << std::endl;
    std::cout << " Max Velocity    : " << maxVelocity    << " m/s"                    << std::endl;
    std::cout << " Max-Q           : " << maxQ           << " Pa"                     << std::endl;
    std::cout << " Burnout Time    : " << burnoutTime    << " s"                      << std::endl;
    std::cout << " Burnout Altitude: " << burnoutAltitude << " m"                     << std::endl;
    std::cout << " Impact Velocity : " << landingVelocity << " m/s"                   << std::endl;
    std::cout << " Landing Status  : " << landingStatus                               << std::endl;
    std::cout << " Total Flight    : " << simTime        << " s"                      << std::endl;
    std::cout << " Escape Velocity : " << escapeVelocity  << " m/s"                   << std::endl;
    std::cout << " Orbital Velocity: " << orbitalVelocity << " m/s"                   << std::endl;
    std::cout << " Velocity Verdict: " << velocityVerdict                             << std::endl;
    std::cout << " Final Stage     : " << (rocket.getCurrentStageIndex() + 1) << "/" << rocket.getTotalStages() << std::endl;
    if (rocket.getFinCount() == 0) {
        std::cout << " Stability Margin: N/A (Actively Stabilized - TVC)" << std::endl;
    } else {
        std::cout << " Stability Margin: " << rocket.getStabilityMargin() << " calibers" << std::endl;
    }
    std::cout << "==================================================================" << std::endl;



    return 0;
}