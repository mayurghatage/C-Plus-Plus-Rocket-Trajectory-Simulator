#include <iostream>
#include <string>
#include "Atmosphere.h"
#include "Vehicle.h"

std::string phaseToString(RTS::FlightPhase phase) {
    switch (phase) {
        case RTS::FlightPhase::PRE_LAUNCH:        return "PRE_LAUNCH";
        case RTS::FlightPhase::BOOST:             return "BOOST";
        case RTS::FlightPhase::MAX_Q:             return "MAX_Q";
        case RTS::FlightPhase::BURNOUT:           return "BURNOUT";
        case RTS::FlightPhase::COAST:             return "COAST";
        case RTS::FlightPhase::APOGEE:            return "APOGEE";
        case RTS::FlightPhase::DESCENT:           return "DESCENT";
        case RTS::FlightPhase::TERMINAL_VELOCITY: return "TERMINAL_VELOCITY";
        case RTS::FlightPhase::LANDED:            return "LANDED";
        default:                                   return "UNKNOWN";
    }
}

int main() {
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
    double testAltitudes[] = {0, 5000, 11000, 20000, 35000, 50000};

    std::cout << "\n--- ATMOSPHERE MODULE TEST ---" << std::endl;
    for (double alt : testAltitudes) {
        RTS::AirProperties props = atmo.calculateState(alt);
        std::cout << "Alt: " << alt << " m"
                  << " | T: "   << props.temperature << " K"
                  << " | P: "   << props.pressure    << " Pa"
                  << " | rho: " << props.density     << " kg/m3" << std::endl;
    }

    std::cout << "\n--- VEHICLE MODULE TEST ---" << std::endl;

    RTS::Vehicle rocket(500.0, 1000.0, 15000.0, 250.0); // dryMass, propMass, thrust, Isp
    double dt = 0.1;
    double simTime = 0.0;

    while (simTime < 600.0) {
        RTS::AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density);
        simTime += dt;

        if (rocket.getAltitude() <= 0.0 && rocket.getPhase() == RTS::FlightPhase::DESCENT) {
            std::cout << "[MISSION COMPLETE] Vehicle impacted ground at t=" << simTime << "s" << std::endl;
            break;
        }

        if (static_cast<int>(simTime * 10) % 10 == 0) {
            std::cout << "t=" << simTime << "s | alt=" << rocket.getAltitude()
                      << " m | v=" << rocket.getVelocity()
                      << " m/s | mass=" << rocket.getMass() 
                      << " kg | phase=" << phaseToString(rocket.getPhase()) << std::endl;
        }

        if (rocket.getPhase() == RTS::FlightPhase::LANDED) {
            std::cout << "[MISSION COMPLETE] Vehicle landed." << std::endl;
            break;
        }
    }

    return 0;
}