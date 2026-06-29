#pragma once
#include <vector>
#include <algorithm>

namespace RTS {

    class DragModel {
    public:
        DragModel() {
            // Generic rocket Cd curve (Mach, Cd)
            table = {
                {0.0,  0.15},
                {0.5,  0.18},
                {0.8,  0.35},
                {0.9,  0.40},
                {1.0,  0.45},
                {1.2,  0.38},
                {1.5,  0.30},
                {2.0,  0.25},
                {3.0,  0.20},
                {5.0,  0.18}
            };
        }

        double getCd(double velocity, double speedOfSound) const {
            if (speedOfSound <= 0.0) return table.front().second;
            
            double mach = std::abs(velocity) / speedOfSound;

            // Below table minimum
            if (mach <= table.front().first) return table.front().second;
            // Above table maximum
            if (mach >= table.back().first) return table.back().second;

            // Linear interpolation
            for (size_t i = 0; i < table.size() - 1; i++) {
                if (mach >= table[i].first && mach <= table[i+1].first) {
                    double t = (mach - table[i].first) / (table[i+1].first - table[i].first);
                    return table[i].second + t * (table[i+1].second - table[i].second);
                }
            }

            return table.back().second;
        }

    private:
        std::vector<std::pair<double, double>> table;
    };

}