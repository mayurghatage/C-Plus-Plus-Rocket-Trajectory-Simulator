#pragma once
#include <cmath>

namespace RTS {

    struct AeroCoefficients {
        double cnAlphaNose;   // per radian
        double noseArea;      // reference area, m^2 (body cross-section)
    };

    inline double computeFinCnAlpha(int finCount, double finSpan, double finRootChord,
                                 double finTipChord, double finSweepDistance, double bodyDiameter) {
    if (finCount == 0) return 0.0;
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

    inline double computeNoseCP(double noseLength) {
        return 0.466 * noseLength;  // ogive nose approximation
    }

    inline double computeFinCP(int finCount, double finPosition, double finRootChord, double finTipChord, double finSweepDistance) {
        if (finCount == 0 || (finRootChord + finTipChord) <= 0.0) {
            return 0.0;  // no fins -> no fin CP contribution
        }
        return finPosition
             + (finSweepDistance * (finRootChord + 2.0*finTipChord)) / (3.0 * (finRootChord + finTipChord))
             + (1.0/6.0) * (finRootChord + finTipChord - (finRootChord*finTipChord)/(finRootChord+finTipChord));
    }

    inline double computeCP(double cnAlphaNose, double xNose, double cnAlphaFin, double xFin) {
        return (cnAlphaNose * xNose + cnAlphaFin * xFin) / (cnAlphaNose + cnAlphaFin);
    }

    inline double computeStabilityMargin(double xCp, double xCg, double bodyDiameter) {
        return (xCp - xCg) / bodyDiameter;  // in calibers
    }

}