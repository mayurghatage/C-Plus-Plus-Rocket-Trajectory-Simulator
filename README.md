# ASTRA: C++ Rocket Trajectory Simulator
 
**A physics-first rocket trajectory simulator built from scratch in C++**, modelling real launch vehicles using 4th-order Runge-Kutta integration, a 7-layer ISA 1976 atmosphere, inverse-square gravity, and Barrowman aerodynamic stability theory.
 
ASTRA isn't a wrapper around an existing physics engine; every subsystem — numerical integrator, atmosphere model, drag model, aerodynamic stability calculations, multi-stage mass/propellant tracking, fault detection, and inverse design optimisation — is implemented from first principles.
 
---
 
## Why ASTRA Exists
 
Most open-source rocket simulators (RocketPy, OpenRocket) are excellent tools, but they're black boxes you *import*, not build. ASTRA exists to prove the underlying physics is understood well enough to *implement*, not just call — and to go further than existing tools with capabilities they don't offer: automated fault detection and inverse design optimisation (see [Roadmap](#roadmap)).
 
---
 
## Features
 
### Flight Dynamics
- **4th-order Runge-Kutta (RK4) integration** for 3D motion (position and velocity)
- **Multi-stage separation** with per-stage mass, thrust, Isp, and burn time — full stacked-vehicle mass accounted for at construction, not just the active stage
- **RK4-integrated propellant tracking** — mass flow per stage computed directly from the integrator's derivative terms, not a stale delta approximation
- **Inverse-square gravity** (`GM/r²`), not a constant-`g` approximation — validated against real escape velocity behaviour
- **Mach-dependent drag coefficient** lookup table with linear interpolation
- **Pitch-over gravity turn** profile for realistic ascent trajectories
### Atmosphere
- **ISA 1976 standard atmosphere model**, 7 atmospheric layers plus isothermal cap at 84,852 m (fixes NaN/altitude-ceiling behaviour common in simplified models)
- Returns temperature, pressure, density, and speed of sound at any altitude
### Aerodynamics & Stability (Barrowman Method)
- Nose cone normal-force coefficient and centre of pressure (CP)
- Fin normal-force coefficient and CP (handles arbitrary fin count, span, chord, and sweep geometry)
- Combined vehicle CP via normal-force-weighted averaging
- Stability margin reported in calibres
- **Dynamic mass-weighted centre of gravity** — recomputed every frame as propellant burns, using per-stage position/mass data (not a static estimate)
- **Finless / TVC-stabilised vehicles handled correctly** — vehicles like GSLV, LVM3, and SSLV that use active thrust vector control (not passive fins) report `N/A (Actively Stabilised - TVC)` instead of a meaningless calibre number
### Mission Logic
- Flight phase state machine: `PRE_LAUNCH → BOOST → MAX_Q → BURNOUT → STAGE_SEPARATION → COAST → APOGEE → DESCENT → LANDED`
- Escape velocity and orbital velocity checks at any point in flight
- **Expendable vs. reusable vehicle logic** — expendable vehicles (GSLV, PSLV, LVM3, SSLV) end simulation on orbital/escape velocity achievement rather than tracking a physically meaningless post-mission descent; reusable vehicles (Falcon 9) continue tracking full descent and landing status
- Impact velocity tracking with soft-landing / hard-impact verdicts
- Config validation with descriptive errors for malformed vehicle data
### Fault Detection
- Dual-run architecture: a nominal trajectory is simulated alongside a faulty one for direct comparison
- Four fault types: Isp degradation, delayed stage separation, thrust degradation, and sensor fault
- **Sensor fault** is the key differentiator — corrupts telemetry only, while the vehicle itself flies nominally, modelling the real avionics problem of distinguishing "did the rocket break" from "did our knowledge of the rocket break" (the class of failure behind incidents like Ariane 5's inertial reference fault). Includes a self-diagnosing consistency check (altitude-derived velocity vs. reported velocity) that needs no nominal reference run
- Two detection layers: pointwise telemetry deviation (% difference per timestep) and mission-event timing comparison (burnout, stage separation, orbital insertion delta) — the second layer exists because a fault can shift event timing by double-digit seconds while pointwise deviation stays under 1%, well below any reasonable anomaly threshold
- Interactive prompt for fault type, severity, target stage, and start time
### Inverse Design Optimiser
- Given a target orbital-insertion altitude, solves backwards via bisection search for either the required propellant mass or thrust
- Runs against the full multi-stage vehicle (not a truncated one) so the existing pitch/gravity-turn program stays valid and the search space stays monotonic
- Upper-bound bracket validation, 20-iteration bisection, 1000 m convergence tolerance
- Post-convergence sensitivity analysis (±5% parameter perturbation) to confirm the solution isn't fragile
### Vehicle Presets
Five real launch vehicles, defined in JSON, with real (or best-available public) mass, thrust, Isp, and geometry data:
 
| Vehicle | Stages | Stabilization | Origin |
|---|---|---|---|
| Falcon 9 | 2 | Reusable (stage 1 landing) | SpaceX |
| PSLV | 4 (solid–liquid–solid–liquid) | Fin-stabilized | ISRO |
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
  ConfigLoader.h       — JSON → VehicleConfig → Vehicle construction
  Integrator.h         — State/Derivative structs for RK4
  FaultDetector.h       — Dual-run trajectory comparison, fault injection
  InverseOptimizer.h    — Bisection-search propellant/thrust solver
 
src/
  Vehicle.cpp         — Core physics: computeDerivative(), update(), getStabilityMargin()
  main.cpp            — Simulation driver, mission summary, CSV telemetry export
 
configs/
  falcon9.json, pslv.json, gslv.json, lvm3.json, sslv.json
 
visualization/
  crt_test.py         — Python/moderngl/pygame CRT-style telemetry HUD (in progress)
```
 
### Config → Simulation Data Flow
```
vehicle.json → ConfigLoader::loadVehicleConfig() → VehicleConfig struct
             → RTS::Vehicle constructor → RK4 simulation loop (main.cpp)
             → per-frame telemetry (CSV) + mission summary (stdout)
```
 
---
 
## Building
 
Requires C++17, CMake, and a compiler with C++17 support (developed against MinGW/w64devkit on Windows).
 
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
 Stability Margin: N/A (Actively Stabilised - TVC)
==================================================================
```
 
---
 
## Known Limitations
 
Being transparent about what's a validated physical model versus a known open issue:
 
- **PSLV 4th stage (PS4) under-burns propellant** and the vehicle currently falls back to Earth instead of reaching orbit — likely a thrust-to-mass ratio or pitch-program issue specific to the final stage. Under investigation.
- **Mission summary can print a contradictory verdict** — "Impact Velocity: 0 m/s" alongside "SIMULATION ENDED - DID NOT LAND (still falling)" in the same run. These are mutually exclusive and shouldn't co-occur. Not yet fixed.
- Post-orbital-insertion trajectory tracking for upper stages on reusable vehicles (e.g., Falcon 9 stage 2) can currently mislabel an escaping trajectory as "descent" — a fix analogous to the expendable-vehicle mission-end logic is planned.
- Sensor fault currently only corrupts the altitude telemetry channel; velocity/mass channel corruption is a possible future extension. Detection threshold (10%) is a first-pass value, not yet tuned.
---
 
## Roadmap
 
ASTRA is built in layers, each meant to differentiate it further from existing simulators:
 
- ✅ **Layer 1 — Physics Engine**: RK4 integration, real atmosphere and gravity models, multi-stage mass/propellant tracking, Barrowman CP/stability margin, dynamic mass-weighted CG
- ✅ **Layer 2 — Fault Detection**: dual-run nominal-vs-faulty comparison, four fault types, pointwise + mission-event anomaly detection
- ✅ **Layer 3 — Inverse Design Optimiser**: bisection search for propellant mass or thrust to hit a target orbital-insertion altitude, with post-convergence sensitivity analysis
- 🔄 **Visualization Layer** *(in progress)*: retro CRT-style telemetry HUD — Python, moderngl/pygame, GLSL fragment shader for phosphor-green glow, barrel distortion, and scanlines. Currently renders a styled test HUD; wiring in real ASTRA CSV telemetry (time, position, altitude, velocity, Mach, mass, phase) is next
- ⏳ **Phase 2 — STK Integration**: feed ASTRA's C++ burnout state vector into STK for orbital propagation, access analysis, and ground track visualization
- ⏳ Companion 6-DOF attitude dynamics/control simulator (future)
---
 
## Interactive Fault Testing
After the nominal run, the program prompts for:
1. Fault type (1 = Isp degradation, 2 = stage separation delay, 3 = sensor fault, 4 = thrust degradation)
2. Fault-specific parameters (severity/magnitude, target stage, sensor subtype where applicable)
3. Fault start time (seconds)
Invalid inputs are rejected with a warning, and the fault test is skipped safely.
 
---
 
## Tech Stack
 
- **Language:** C++17
- **Build system:** CMake
- **JSON parsing:** [nlohmann/json](https://github.com/nlohmann/json) (single-header)
- **Visualization:** Python, moderngl, pygame, GLSL
- **Version control:** Git, conventional commits (`feat:`, `fix:`, `refactor:`, `docs:`)
---