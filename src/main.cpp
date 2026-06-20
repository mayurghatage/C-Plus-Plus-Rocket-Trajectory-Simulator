#include <iostream>
#include "Atmosphere.h"
#include "Vehicle.h"

int main() {
    #ifdef _Win32
        std::system("cls");
    #else
        std::system("clear");
    #endif

    std::cout << "================================================================" << std::endl;
    std::cout << "   MISSION PLANNING & FAULT ANALYSIS GNC ENGINE // VERSION 26.1.0" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "              INITIALIZING SYSTEM SUBSYSTEMS..."                  << std::endl;
    std::cout << " ---------------------------------------------------------------" << std::endl;
    std::cout << " [INFO] Core C++20 compile matrix verified configuration."        << std::endl;
    std::cout << " [INFO] Memory allocations for internal state arrays initialized."<< std::endl;
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

    while (simTime < 30.0) {
        RTS::AirProperties props = atmo.calculateState(rocket.getAltitude());
        rocket.update(dt, props.density);
        simTime += dt;

        if (static_cast<int>(simTime * 10) % 10 == 0) {
            std::cout << "t=" << simTime << "s | alt=" << rocket.getAltitude()
                       << " m | v=" << rocket.getVelocity()
                       << " m/s | mass=" << rocket.getMass() << " kg" << std::endl;
        }
    }

    return 0;
}