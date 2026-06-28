#pragma once

namespace RTS {

    // Holds the full flight state at any point in time
    struct State {
        double altitude;  // meters
        double velocity;  // m/s
        double mass;      // kg
    };

    // Derivative struct — rates of change of the state
    // Used by RK4 to sample slopes at intermediate points
    struct Derivative {
        double dAltitude;  // velocity (m/s)
        double dVelocity;  // acceleration (m/s^2)
        double dMass;      // mass flow rate (kg/s)
    };

}