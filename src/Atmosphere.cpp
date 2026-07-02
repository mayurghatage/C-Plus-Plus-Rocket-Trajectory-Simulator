#include "Atmosphere.h"
#include <cmath>

namespace RTS {

    AirProperties AtmosphereModel::calculateState(double geometricAltitudeMeters) {

        // Earth Radius Constant in meters
        const double EARTH_RADIUS = 6371000.0;
    
        // Convert to geopotential altitude for the ISA 1976 dataset
        double altitudeMeters = (EARTH_RADIUS * geometricAltitudeMeters) / (EARTH_RADIUS + geometricAltitudeMeters);
        
        // ISA 1976 layer data: {base altitude in meters, base temperature in Kelvin, lapse rate in K/m}
        const double layers[8][3] = {
            {0.0,     288.15,  -0.0065},  // Troposphere
            {11000.0, 216.65,   0.0},     // Lower Stratosphere (isothermal)
            {20000.0, 216.65,   0.001},   // Upper Stratosphere
            {32000.0, 228.65,   0.0028},  // Stratosphere-Mesosphere transition
            {47000.0, 270.65,   0.0},     // Upper Stratosphere (isothermal)
            {51000.0, 270.65,  -0.0028},  // Lower Mesosphere
            {71000.0, 214.65,  -0.002},    // Upper Mesosphere
            {84852.0, 186.87,   0.0}
        };

        // Base pressure at bottom of each layer in Pascals
        const double basePressure[8] = {
            101325.0, // sea level pressure (1 atm)
            22632.1,
            5474.89,
            868.019,
            110.906,
            66.9389,
            3.95642,
            0.3734
        };

        // Find which layer we're in
        int layerIndex = 0;
        for (int i = 1; i < 8; i++) {
            if (altitudeMeters >= layers[i][0]) layerIndex = i;
            else break;
        }

        double h_base = layers[layerIndex][0];
        double T_base = layers[layerIndex][1];
        double L      = layers[layerIndex][2];
        double P_base = basePressure[layerIndex];

        AirProperties result;

        if (L != 0.0) {
            // Linear temperature layer — power law for pressure
            result.temperature = T_base + L * (altitudeMeters - h_base);
            result.pressure    = P_base * std::pow(result.temperature / T_base, -G_ZERO / (L * R_AIR));
        } else {
            // Isothermal layer — exponential decay for pressure
            result.temperature = T_base;
            result.pressure    = P_base * std::exp(-G_ZERO * (altitudeMeters - h_base) / (R_AIR * T_base));
        }

        // Ideal gas law for density
        result.density = result.pressure / (R_AIR * result.temperature);

        // Speed of sound
        const double GAMMA = 1.4;
        result.speedOfSound = std::sqrt(GAMMA * R_AIR * result.temperature);

        return result;
    }

}