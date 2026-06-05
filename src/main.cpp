#include <iostream>

int main() {
    #ifdef _Win32
        std::system("cls");
    #else
        std::system("clear");
    #endif

    std::cout << "=====================================================================" << std::endl;
    std::cout << "   MISSION PLANNING & FAULT ANALYSIS GNC ENGINE // VERSION 26.1.0"     << std::endl;
    std::cout << "=====================================================================" << std::endl;
    std::cout << "                 INITIALIZING SYSTEM SUBSYSTEMS..."                    << std::endl;
    std::cout << " --------------------------------------------------------------------" << std::endl;
    std::cout << " [INFO] Core C++20 compile matrix verified configuration."             << std::endl;
    std::cout << " [INFO] Memory allocations for internal state arrays initialized."     << std::endl;
    std::cout << " [WARN] Atmospheric profile data stream unresolved. Module pending."   << std::endl;
    std::cout << " --------------------------------------------------------------------" << std::endl;
    std::cout << " STATUS: Awaiting structural design constraints and vehicle profile."  << std::endl;
    std::cout << "=====================================================================" << std::endl;
    
    return 0;
}