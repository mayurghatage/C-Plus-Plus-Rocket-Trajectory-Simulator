#pragma once

namespace RTS {

    struct State {
    double x, z;       // position (m) - x horizontal, z vertical
    double vx, vz;      // velocity (m/s)
    double mass;        // kg
    };

    struct Derivative {
        double dx, dz;       // = vx, vz
        double dvx, dvz;     // acceleration components
        double dMass;
    };

    // Tracks which phase of flight the vehicle is currently in
    enum class FlightPhase {
        PRE_LAUNCH,
        BOOST,
        MAX_Q,
        BURNOUT,
        COAST,
        APOGEE,
        DESCENT,
        TERMINAL_VELOCITY,
        LANDED
    };

}