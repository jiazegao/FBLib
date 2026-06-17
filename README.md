# FBLib

A ground-up C++ robotics library for VEX V5RC built on the PROS kernel. FBLib provides full-stack robotics capabilities: localization (odometry, MCL, RCL), motion control (PID, point-to-point, arcs, boomerang, RAMSETE, swing turns, velocity profiles), and an LVGL-based autonomous selector with on-screen dry-run path preview.

**Target:** VEX V5 Brain (ARM Cortex-A9, PROS kernel 4.2.2)  
**Language:** C++20 / gnu++26  
**Namespace:** `FBLIB`

## Quick Start

```cpp
#include "FBLib/FB_API.hpp"
using namespace FBLIB;

// — Drivetrain —
pros::MotorGroup leftMotors({-11, -12, -13});   // reverse direction ports
pros::MotorGroup rightMotors({18, 19, 20});
Drivetrain dt(&leftMotors, &rightMotors,
              10.4f,   // track width (inches, center-to-center)
              3.25f);  // wheel diameter (inches)

// — Sensors —
pros::Imu imu(15);
OdomSensors sensors;
sensors.imuCollection.push_back(&imu);
sensors.drivetrain = &dt;   // motor encoder fallback (optional)

// — Chassis —
Chassis chassis(dt, sensors);
chassis.calibrate();            // IMU calibration (~2s, robot must be still)
chassis.setPose({0, 0, 0});    // Pose.theta in radians (0 = +X axis)

// — Motion —
chassis.moveToPoint(24, 0, 2000);    // 24" forward, 2s timeout, blocking
chassis.turnToHeading(90, 1000);     // turn to 90° VEX, 1s timeout
chassis.moveDistance(-12, 0, {}, true);  // 12" backward, no timeout, async
```

See [example/](example/) for a complete project template.

### Heading Convention

All **user-facing** heading values use VEX heading units: **degrees 0–360, clockwise positive** (0 = forward, 90 = right, 180 = backward).

| API call | Heading unit |
|----------|-------------|
| `turnToHeading(thetaDeg, ...)` | VEX degrees |
| `swingToHeading(thetaDeg, ...)` | VEX degrees |
| `moveBoomerang(x, y, thetaDeg, ...)` | VEX degrees |
| `setHeading(thetaDeg)` | VEX degrees |
| `Pose.theta` (internal storage) | radians (0 = +X, CCW positive) |

All heading tolerances (`targetTolerance`, `headingTolerance`) are in VEX degrees.  
Use `vexToStdRad()` / `stdRadToVexDeg()` when working with `Pose.theta` directly.

### Timeout & Async

Every movement method accepts a `timeout` parameter (milliseconds). Set to `0` for no timeout. Pass `true` for the final `async` parameter to run non-blocking.

```cpp
chassis.moveDistance(24, 2000);               // 2s timeout, default params
chassis.moveToPoint(48, 24, 0, {...});        // no timeout, custom params
chassis.turnToHeading(180, 1500, {...}, true); // async with timeout

// Async management:
chassis.waitUntilSettled();    // block caller until motion finishes
chassis.cancelMotion();        // stop current + queued motions
bool done = chassis.isSettled(); // non-blocking check
```

The async system uses a persistent background task with a one-deep queue — calling an async motion while one is already running queues the second behind it.

## Features

### Tracking

| Module | Description |
|--------|-------------|
| **OdomTracking** | Arc-approximation odometry with 3-tier sensor fallback for vertical distance (tracking wheels → motor encoders → 0) and 2-tier for horizontal (tracking wheels → 0). Exposes a decomposed `OdomDelta {dVert, dHoriz, dTheta}` for per-axis noise modeling in MCL. |
| **MCL** | Monte Carlo localization — 1024-particle filter with systematic low-variance resampling. Gaussian sensor model with angle-dependent sigma scaling. Ray-casts against configurable field geometry (walls + season-specific targets). Per-sensor-type noise model (tracking wheels / IME / no-sensor random walk). IMU-heading particle clamping prevents orientation drift. |
| **RCL** | Ray/Ceiling-Line localization using V5 distance sensors. Independently averages X and Y coordinates from sensors constraining each axis (east/west walls → X, north/south walls → Y). Supports temporary/permanent obstacles and exponential-moving-average accumulation. |

### Movement Control

| Module | Description |
|--------|-------------|
| **PID** | Configurable PID with anti-windup clamping, low-pass filtered derivative (`α = 0.1`), and flip-reset (LemLib-style integral negation on direction reversal). |
| **MoveToPoint** | Straight-line / point-to-point motion with simultaneous lateral + angular PID. Supports both forward and backward driving (heading offset + lateral negation). |
| **TurnToPoint** | Rotational motion with configurable turn direction (CW, CCW, shortest path). |
| **Arc** | Circular arc motion with configurable radius (positive = CCW, negative = CW). |
| **Boomerang** | Curved approach controller with carrot-point guidance and lead decay. Blends approach and final heading smoothly as the robot nears the target. |
| **RAMSETE** | Nonlinear SE(2) trajectory tracking controller (`k_x`, `k_y`, `k_θ` gains) combined with pure-pursuit lookahead for following pre-computed paths. |
| **Swing** | Pivot turn around one stationary wheel set. Left swing = left motors stationary, right motors turn (CCW). Right swing = right motors stationary, left motors turn (CCW). |
| **Velocity Profiles** | Analytical 7-phase S-curve generator (jerk-limited acceleration → constant accel → jerk to cruise → constant velocity → jerk-limited decel → constant decel → jerk to zero). Falls back to trapezoidal for sub-inch moves. |

### Autonomous Selector

| Module | Description |
|--------|-------------|
| **AutonSelector** | LVGL touchscreen UI on the V5 Brain. Alliance selection (Red/Blue/None), autonomous routine cycling, skills mode toggle, IMU recalibration. Registers up to 16 auton routines by name + callback. |
| **PathPreview** | Renders autonomous routine paths on the brain screen via dry-run simulation. The robot's auton function is executed with motors disabled — the resulting `dryRunPath()` trajectory is drawn as line segments with waypoint markers and a robot orientation indicator. Supports field background images. |

### Utilities

| Module | Description |
|--------|-------------|
| **FastTrig** | 4 KB L1-cache-aligned sine LUT (1024 entries over [0, π/2]) with quadrant folding. Drop-in `sin()`/`cos()` for MCL's inner particle loop (~40 Hz × 1024 particles). |
| **`Pose` / `OdomDelta`** | 2D pose struct with arithmetic operators. Decomposed odometry delta for per-axis MCL noise. |
| **`wrapRad()` / `angleDiffRad()`** | Angle utilities: wrap to [-π, π], shortest signed/unsigned difference. |
| **`Field` namespace** | Field geometry constants (`FIELD_HALF_WALL = 70.2"`, etc.) parameterized for season changes. Includes screen-coordinate mappers for the V5 Brain display (480×240). |
| **ScaledIMU** | IMU wrapper with configurable scale factor to correct V5 IMU under-reporting (~354.25° per 360° physical turn). Also available via `OdomSensors::imuScaleFactor`. |

## Architecture

```
FBLib/
├── include/FBLib/
│   ├── FB_API.hpp              ← single-include public API
│   ├── Chassis.hpp              ← Chassis, Drivetrain, DriveCurve, ChassisConfig
│   ├── Tracking/
│   │   ├── Odom_Tracking.hpp   ← odometry (TrackingWheel + OdomSensors + solver)
│   │   ├── MCL_Tracking.hpp    ← Monte Carlo localization (particle filter)
│   │   └── RCL_Tracking.hpp    ← Ray/Ceiling-Line localization
│   ├── Movement_Control/
│   │   ├── MoveToPoint.hpp     ← point-to-point motion params
│   │   ├── TurnToPoint.hpp     ← turn + swing params & enums
│   │   ├── Arc.hpp             ← arc params
│   │   ├── Boomerang.hpp       ← boomerang params
│   │   ├── RAMSETE.hpp         ← RAMSETE params + lookahead utilities
│   │   └── Velocity_Profiles.hpp ← profile generation (trapezoidal + S-curve)
│   ├── Auton_Selector/
│   │   ├── GUI.hpp             ← LVGL auton selector
│   │   └── Path_Selector.hpp   ← path preview rendering
│   └── Util/
│       ├── Util.hpp            ← Pose, math, angle utils, Field constants
│       ├── Pid.hpp             ← PID controller
│       ├── FastTrig.hpp        ← fast sine/cosine LUT
│       └── ScaledIMU.hpp       ← IMU scale factor wrapper
├── src/FBLib/                   ← implementation files (mirrors include/ layout)
├── example/                     ← minimal PROS project template
└── README.md
```

## Build

Standard PROS project. Requires the [PROS CLI](https://pros.cs.purdue.edu/).

```bash
pros make          # compile
pros make clean    # clean build
pros upload        # upload to V5 Brain
```

The Makefile is configured with `IS_LIBRARY:=0` (application project). Set `IS_LIBRARY:=1` and `LIBNAME:=FBLib` to build as a reusable static library.

## Configuration

### Sensors

All sensor counts are flexible — any number of tracking wheels (ADI encoders and/or V5 Rotation sensors), up to 8 distance sensors, and any number of IMUs. Change `MAX_DISTANCE_SENSORS` in [Util.hpp](include/FBLib/Util/Util.hpp) to resize all sensor arrays system-wide (requires `pros make clean && pros make`).

Sensor fallback chain for vertical distance:
1. Average of all tracking wheels in `vertWheelCollection`
2. Average of drivetrain motor encoders (`drivetrain->averagePositionDeg()`)
3. `0` (IMU-heading-only dead reckoning — no distance measurement)

For horizontal distance: tracking wheels only (motor encoders cannot measure lateral movement on a tank drive).

### MCL

Per-axis noise model in [MCL_Tracking.hpp](include/FBLib/Tracking/MCL_Tracking.hpp#L53-L66):
- **Vertical axis (tracking wheel)**: `trackingWheelVariance = 0.08` (low)
- **Vertical axis (IME fallback)**: `imeVariance = 0.12` (higher — accounts for wheel slip)
- **Horizontal axis (tracking wheel)**: `trackingWheelVariance = 0.08`
- **Horizontal axis (no sensor)**: random walk — `horizDependentVarianceProp * |dVert|` + `horizConstantNoise`

Particle theta is clamped to within `maxThetaDeviation` (0.08 rad ≈ 4.6°) of the IMU heading — the IMU is authoritative for orientation.

### Field Geometry

Field constants are in the `Field` namespace in [Util.hpp](include/FBLib/Util/Util.hpp#L212-L234). Update these for each season's field layout. MCL wall/obstacle definitions live in [MCL_Tracking.hpp](include/FBLib/Tracking/MCL_Tracking.hpp#L228-L262) — modify `kFieldTargets`, `kFieldCircles`, and `kDisablingLines` per season.

## Design Decisions

- **IMU heading is authoritative.** `OdomTracking` uses the raw IMU heading as the pose theta — the IMU is the ground-truth for orientation. Sync corrections from MCL/RCL are applied to X/Y only.
- **Async by choice, not by default.** Every movement method supports both blocking and non-blocking modes. A single persistent `pros::Task` handles async motion sequencing with a one-deep queue.
- **Dry-run path preview.** The auton selector runs the selected auton function with motors disabled to record the full trajectory for on-screen preview — no manual waypoint registration needed.
- **LemLib-independent.** FBLib is a clean-room implementation. LemLib was used as reference only — no dependency.
- **Per-axis decomposition.** `OdomTracking::getLastDelta()` exposes raw `{dVert, dHoriz, dTheta}` from sensor channels (not geometrically decomposed from a fused pose), enabling statistically correct per-sensor-type noise in MCL.
