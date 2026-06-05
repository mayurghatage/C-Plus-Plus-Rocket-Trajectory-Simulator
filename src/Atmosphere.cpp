#include "Atmosphere.h"

namespace RTS {

    AirProperties AtmosphereModel::calculateState(double altitudeMeters) {
        AirProperties currentStatus;

        currentStatus.temperature = T_ZERO;
        currentStatus.pressure    = P_ZERO;
        currentStatus.density     = P_ZERO / (R_AIR * T_ZERO);

        return currentStatus;
    }

}