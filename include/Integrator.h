#pragma once

namespace RTS {

    struct State {
    double x, y, z;       // position (m) - x downrange, y lateral, z vertical
    double vx, vy, vz;      // velocity (m/s)
    double mass;        // kg
    };

    struct Derivative {
        double dx, dy, dz;       // = vx, vy, vz
        double dvx, dvy, dvz;     // acceleration components
        double dMass;
    };

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