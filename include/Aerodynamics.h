#pragma once
#include <cmath>

namespace RTS {

    struct AeroCoefficients {
        double cnAlphaNose;   // per radian
        double noseArea;      // reference area, m^2 (body cross-section)
    };

    inline double computeFinCnAlpha(int finCount, double finSpan, double finRootChord,
                                     double finTipChord, double finSweepDistance, double bodyDiameter) {
        double s = finSpan;
        double num = 4.0 * finCount * (s / bodyDiameter) * (s / bodyDiameter);
        double ratio = (2.0 * finSweepDistance) / (finRootChord + finTipChord);
        double den = 1.0 + std::sqrt(1.0 + ratio * ratio);
        return num / den;
    }

    inline AeroCoefficients computeNoseAero(double bodyDiameter) {
        double radius = bodyDiameter / 2.0;
        double refArea = M_PI * radius * radius;

        AeroCoefficients aero;
        aero.cnAlphaNose = 2.0;      // Barrowman: constant for any nose shape
        aero.noseArea = refArea;
        return aero;
    }

    inline double computeNormalForceCoefficient(double alpha, double cnAlphaNose, double cnAlphaFin) {
        double cnNose = cnAlphaNose * alpha;
        double cnFin = cnAlphaFin * alpha; 
        return cnNose + cnFin;
    }

}