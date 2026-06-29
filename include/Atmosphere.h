#ifndef ATMOSPHERE_H  // "for - if ATMOSPHERE_H is NOT yet defined..."
#define ATMOSPHERE_H   // "for - ...define it right now"

namespace RTS {

    struct AirProperties {
        double density;     // kg/m^3
        double pressure;    // Pascals
        double temperature; // Kelvin
        double speedOfSound; // m/s
    };

    class AtmosphereModel {
    private:
        // sea-level thermodynamic constants
        const double R_AIR   = 287.05; // "J/(kg·K)" Specific gas constant for dry air
        const double G_ZERO  = 9.80665; // "m/s²" Standard gravitational acceleration at sea level
        const double P_ZERO  = 101325.0; // "Pa" Sea-level standard pressure
        const double T_ZERO  = 288.15; // "K" Sea-level standard temperature

    public:
        AtmosphereModel() = default;

        AirProperties calculateState(double geometricAltitudeMeters);
    };

}

#endif  // close the if-block