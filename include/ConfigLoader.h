#pragma once
#include "json.hpp"
#include <fstream>
#include <string>

struct VehicleConfig {
    double dryMass;
    double propellantMass;
    double thrust;
    double isp;
};

inline VehicleConfig loadVehicleConfig(const std::string& filepath) {
    std::ifstream file(filepath);
    nlohmann::json data = nlohmann::json::parse(file);

    VehicleConfig config;
    config.dryMass = data["dryMass"];
    config.propellantMass = data["propellantMass"];
    config.thrust = data["thrust"];
    config.isp = data["isp"];

    return config;
}