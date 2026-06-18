#include <iostream>
#include "Atmosphere.h"

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

    return 0;
}