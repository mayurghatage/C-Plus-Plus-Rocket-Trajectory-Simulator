#ifndef ATMOSPHERE_H  // "for - if ATMOSPHERE_H is NOT yet defined..."
#define ATMOSPHERE_H   // "for - ...define it right now"

namespace RTS {

    struct AirProperties {
        double density;     // kg/m^3
        double pressure;    // Pascals
        double temperature; // Kelvin
    };

    class AtmosphereModel {
    private:
        // sea-level thermodynamic constants
        const double R_AIR   = 287.05; // "J/(kg·K)"
        const double G_ZERO  = 9.80665; // "m/s²"
        const double P_ZERO  = 101325.0; // "Pa"
        const double T_ZERO  = 288.15; // "K"

    public:
        AtmosphereModel() = default;

        AirProperties calculateState(double geometricAltitudeMeters);
    };

}

#endif  // close the if-block