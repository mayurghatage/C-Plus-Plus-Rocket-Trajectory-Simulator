#pragma once

namespace RTS {

    struct State {
    double x = 0, y = 0, z = 0;
    double vx = 0, vy = 0, vz = 0;
    double mass = 0;
    };

    struct Derivative {
        double dx = 0, dy = 0, dz = 0;
        double dvx = 0, dvy = 0, dvz = 0;
        double dMass = 0;
    };

    enum class FlightPhase {
        PRE_LAUNCH,
        BOOST,
        MAX_Q,
        STAGE_SEPARATION,
        BURNOUT,
        COAST,
        APOGEE,
        DESCENT,
        TERMINAL_VELOCITY,
        LANDED
    };

}