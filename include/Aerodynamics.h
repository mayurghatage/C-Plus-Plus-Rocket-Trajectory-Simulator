#pragma once
#include <cmath>

namespace RTS {

    struct AeroCoefficients {
        double cnAlphaNose;   // per radian
        double noseArea;      // reference area, m^2 (body cross-section)
    };

    inline AeroCoefficients computeNoseAero(double bodyDiameter) {
        double radius = bodyDiameter / 2.0;
        double refArea = M_PI * radius * radius;

        AeroCoefficients aero;
        aero.cnAlphaNose = 2.0;      // Barrowman: constant for any nose shape
        aero.noseArea = refArea;
        return aero;
    }

}