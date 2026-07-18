# ASTRA — C++ Rocket Trajectory Simulator

**A physics-first rocket trajectory simulator built from scratch in C++**, modeling real launch vehicles using 4th-order Runge-Kutta integration, a 7-layer ISA 1976 atmosphere, inverse-square gravity, and Barrowman aerodynamic stability theory.

ASTRA isn't a wrapper around an existing physics engine — every subsystem (numerical integrator, atmosphere model, drag model, aerodynamic stability calculations, and orbital velocity checks) is implemented from first principles.

---

## Why ASTRA Exists

Most open-source rocket simulators (RocketPy, OpenRocket) are excellent tools, but they're black boxes you *import*, not build. ASTRA exists to prove the physics is understood well enough to *implement*, not just call — and to eventually go further than existing tools by adding capabilities they don't have: automated fault detection and inverse design optimization (see [Roadmap](#roadmap)).

---

## Features

### Flight Dynamics
- **4th-order Runge-Kutta (RK4) integration** for 3D motion (x, y, z position and velocity)
- **Multi-stage separation** with per-stage mass, thrust, Isp, and burn time
- **Inverse-square gravity** (`GM/r²`), not a constant-`g` approximation — validated against real escape velocity behavior
- **Mach-dependent drag coefficient** lookup table with linear interpolation
- **Pitch-over gravity turn** profile for realistic ascent trajectories

### Atmosphere
- **ISA 1976 standard atmosphere model**, 7 atmospheric layers plus isothermal cap at 84,852 m (fixes NaN/altitude-ceiling behavior common in simplified models)
- Returns temperature, pressure, density, and speed of sound at any altitude

### Aerodynamics & Stability (Barrowman Method)
- Nose cone normal-force coefficient and center of pressure (CP)
- Fin normal-force coefficient and CP (handles arbitrary fin count, span, chord, and sweep geometry)
- Combined vehicle CP via normal-force-weighted averaging
- Stability margin reported in calibers
- **Finless / TVC-stabilized vehicles handled correctly** — vehicles like GSLV, LVM3, and SSLV that use active thrust vector control (not passive fins) report `N/A (Actively Stabilized - TVC)` instead of a meaningless caliber number

### Mission Logic
- Flight phase state machine: `PRE_LAUNCH → BOOST → MAX_Q → BURNOUT → STAGE_SEPARATION → COAST → APOGEE → DESCENT → LANDED`
- Escape velocity and orbital velocity checks at any point in flight
- **Expendable vs. reusable vehicle logic** — expendable vehicles (GSLV, PSLV, LVM3, SSLV) end simulation on orbital/escape velocity achievement rather than tracking a physically meaningless post-mission descent; reusable vehicles (Falcon 9) continue tracking full descent and landing status
- Impact velocity tracking with soft-landing / hard-impact verdicts
- Config validation with descriptive errors for malformed vehicle data

### Vehicle Presets
Five real launch vehicles, defined in JSON, with real (or best-available public) mass, thrust, Isp, and geometry data:

| Vehicle | Stages | Stabilization | Origin |
|---|---|---|---|
| Falcon 9 | 2 | Reusable (stage 1 landing) | SpaceX |
| PSLV | 2 (simplified from 4) | Fin-stabilized | ISRO |
| GSLV Mk II | 3 | TVC | ISRO |
| LVM3 | 2 | TVC | ISRO |
| SSLV | 2 | TVC / spin-stabilized | ISRO |

---

## Architecture

```
include/
  Vehicle.h          — Vehicle class: state, stages, RK4 integration driver
  Aerodynamics.h      — Barrowman nose/fin CP, stability margin math
  Atmosphere.h        — ISA 1976 model
  DragModel.h         — Mach-dependent Cd lookup
  ConfigLoader.h      — JSON → VehicleConfig → Vehicle construction
  Integrator.h        — State/Derivative structs for RK4

src/
  Vehicle.cpp         — Core physics: computeDerivative(), update(), getStabilityMargin()
  main.cpp            — Simulation driver, mission summary, CSV telemetry export

configs/
  falcon9.json, pslv.json, gslv.json, lvm3.json, sslv.json
```

### Config → Simulation Data Flow
```
vehicle.json → ConfigLoader::loadVehicleConfig() → VehicleConfig struct
             → RTS::Vehicle constructor → RK4 simulation loop (main.cpp)
             → per-frame telemetry (CSV) + mission summary (stdout)
```

---

## Building

Requires C++17, CMake, and a compiler with C++17 support (developed against MinGW g++ on Windows).

```bash
cmake -B build
cmake --build build
./build/RocketSimulator.exe configs/falcon9.json
```

Any config in `configs/` can be passed as an argument. Defaults to `falcon9.json` if none given.

---

## Sample Output

```
--- VEHICLE MODULE TEST ---
t=57.7s | alt=60507.5 m | v=2869.57 m/s | mass=35682.5 kg | stage=1/3 | phase=MAX_Q
[STAGE SEPARATION] Stage 1 ignited at t=57.7s, alt=60507.5 m
...
================================================================
                    MISSION SUMMARY
==================================================================
 Max Altitude    : 466384 m
 Max Velocity    : 7635.48 m/s
 Escape Velocity : 11186.1 m/s
 Orbital Velocity: 7909.79 m/s
 Velocity Verdict: EXCEEDS ORBITAL VELOCITY
 Stability Margin: N/A (Actively Stabilized - TVC)
==================================================================
```

---

## Known Limitations

Being transparent about what's a physical model versus a placeholder:

- **Center of gravity is currently a placeholder estimate** (`0.55 × total length`), not a true mass-weighted calculation across stage dry mass, propellant, and payload. This causes stability margin sign errors in some configurations. A real per-component mass/position CG model is planned.
- **PSLV is modeled as 2 stages** as a simplification; the real vehicle is 4-stage (solid-liquid-solid-liquid).
- Post-orbital-insertion trajectory tracking for upper stages on reusable vehicles (e.g., Falcon 9 stage 2) can currently mislabel an escaping trajectory as "descent" — a fix analogous to the expendable-vehicle mission-end logic is planned.

---

## Roadmap

ASTRA is built in layers, each meant to differentiate it further from existing simulators:

- ✅ **Layer 1 — Barrowman Stability Analysis** (this build): CP/CG/stability margin, multi-stage dynamics, real atmosphere and gravity models
- 🔜 **Layer 2 — Fault Detection**: compare nominal vs. actual trajectory in real time and flag deviations — a capability neither RocketPy nor OpenRocket currently offer
- 🔜 **Layer 3 — Inverse Design Optimizer**: given a target altitude, ASTRA solves backward for the vehicle parameters needed to reach it

Beyond the three core layers: a Python Rich-based terminal UI, STK integration for orbital analysis (Phase 2), and a companion 6-DOF attitude dynamics/control simulator are planned.

---

## Tech Stack

- **Language:** C++17
- **Build system:** CMake
- **JSON parsing:** [nlohmann/json](https://github.com/nlohmann/json) (single-header)
- **Version control:** Git, conventional commits (`feat:`, `fix:`, `refactor:`)

---

## Author

Built by Mayur Ghatage as an independent aerospace engineering project, developed as a self-taught demonstration of computational rocketry, orbital mechanics, and numerical methods applied to real launch vehicle data.
