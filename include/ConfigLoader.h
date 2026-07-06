#pragma once
#include "json.hpp"
#include "Vehicle.h"
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept> 

struct VehicleConfig {
    std::vector<RTS::Stage> stages;
};

inline VehicleConfig loadVehicleConfig(const std::string& filepath) {
    std::ifstream file(filepath);
    nlohmann::json data = nlohmann::json::parse(file);

    VehicleConfig config;
    for (const auto& s : data["stages"]) {
        RTS::Stage stage;
        stage.dryMass = s["dryMass"];
        stage.propellantMass = s["propellantMass"];
        stage.thrust = s["thrust"];
        stage.isp = s["isp"];
        stage.burnTime = s["burnTime"];

        if (stage.propellantMass <= 0 || stage.thrust <= 0 || stage.burnTime <= 0) {
            throw std::runtime_error("Invalid stage config: propellantMass, thrust, and burnTime must be positive");
        }

        config.stages.push_back(stage);
    }

    return config;
}