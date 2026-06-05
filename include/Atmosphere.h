#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

namespace RTS {

    struct AirProperties {
        double density;     // kg/m^3
        double pressure;    // Pascals
        double temperature; // Kelvin
    };

    class AtmosphereModel {
    private:
        // sea-level thermodynamic constants
        const double R_AIR   = 287.05;
        const double G_ZERO  = 9.80665;
        const double P_ZERO  = 101325.0;
        const double T_ZERO  = 288.15;

    public:
        AtmosphereModel() = default;

        AirProperties calculateState(double altitudeMeters);
    };

}

#endif