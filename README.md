# C++ Rocket Trajectory Simulator 

A physics-based rocket flight simulator written in C++.

## What's built so far

- ISA 1976 Standard Atmosphere model (7-layer, geopotential altitude correction)
- Vehicle dynamics module — variable mass (Tsiolkovsky), drag, thrust, burnout detection
- Euler integration for flight state propagation

## What's next

- Upgrade to RK4 numerical integration
- Multi-phase flight handling (boost/coast/freefall)
- Fault detection and trajectory optimization layers
- Support for multiple propulsion types — rocket motors, jet engines, propeller-driven systems
