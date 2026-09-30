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

See [src/main.cpp](src/main.cpp) for a complete robot program using the library.

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

Even with `timeout = 0`, a motion ends once it stops making progress for `ChassisConfig::stallTimeoutMs` (default 1000 ms) — e.g. the robot is pinned, or friction stops it just short of its tolerance — so a stalled motion can't block the autonomous routine forever.

Motions with `minSpeed > 0` don't stop on the target: they end as soon as they pass it, so the next motion takes over without the robot braking (motion chaining). `earlyExitRange` ends a motion that far from its target.

### Custom motions

Every movement is a `Motion` subclass (see [Motion.hpp](include/FBLib/Movement_Control/Motion.hpp)) that turns the current pose into left/right commands. Your own subclasses get the same queueing, timeout, stall exit, cancellation and dry-run preview:

```cpp
chassis.runMotion(std::make_unique<MyMotion>(...), 2000);
```

## Features

### Tracking

| Module | Description |
|--------|-------------|
| **OdomTracking** | Arc-approximation odometry with 3-tier sensor fallback for vertical distance (tracking wheels → motor encoders → 0) and 2-tier for horizontal (tracking wheels → 0). Exposes a decomposed `OdomDelta {dVert, dHoriz, dTheta}` for per-axis noise modeling in MCL. |
| **MCL** | Monte Carlo localization — 1024-particle filter with systematic low-variance resampling. Gaussian sensor model with angle-dependent sigma scaling. Ray-casts against configurable field geometry (walls + season-specific targets). Per-sensor-type noise model (tracking wheels / IME / no-sensor random walk). IMU-heading particle clamping prevents orientation drift. |
| **RCL** | Ray/Ceiling-Line localization using V5 distance sensors. Independently averages X and Y coordinates from sensors constraining each axis (east/west walls → X, north/south walls → Y). Supports temporary/permanent obstacles and exponential-moving-average accumulation. |

### Movement Control

Each movement type is a `Motion` subclass in its own module; `Chassis` only queues and runs them.

| Module | Description |
|--------|-------------|
| **PID** | Configurable PID with anti-windup clamping, low-pass filtered derivative (`α = 0.1`, no derivative kick on the first update), and flip-reset (LemLib-style integral negation on direction reversal). Motions update it with the loop's `dt`. |
| **Motion** | Common interface (`start()` / `update(pose, dt)` → left/right commands, done, remaining travel) and the shared drive mixer (positive angular = counter-clockwise; saturation keeps the commanded curvature). |
| **MoveToPoint** | `MoveDistanceMotion` drives a signed distance holding the start heading. `MoveToPointMotion` steers at the target, drives on the signed projection of the error (overshoot backs up) and stops steering once aimed within 7.5" so it can't orbit the point. Both drive forwards or backwards. |
| **TurnToPoint** | `TurnToHeadingMotion`, `TurnToPointMotion` (optional clockwise `offset`, e.g. 180 = face away) and `SwingMotion` (pivot on a locked left or right side). CW/CCW direction is forced only until the robot first passes the target, so an overshoot is corrected the short way instead of with another full turn. |
| **Arc** | Follows the circle of the given radius through the start position and the target (positive = counter-clockwise, the shorter arc), with curvature feed-forward and correction back onto the circle. Forwards or backwards. |
| **Boomerang** | Curved approach to a pose: carrot point behind the target along its final heading, blending to the final heading inside 12" and holding it inside 7.5". Forwards or backwards. |
| **RAMSETE** | Nonlinear SE(2) tracking law along a pose path with a short arc-length lookahead, a trapezoidal speed profile over the path, and an exit when the robot crosses the end of the path. |
| **Velocity Profiles** | Time-sampled trapezoidal and 7-phase jerk-limited S-curve generators (peak speed lowered for moves too short to reach it) plus `trapezoidalVelocityAt()` for speed-at-position lookups. |

### Autonomous Selector

| Module | Description |
|--------|-------------|
| **AutonSelector** | LVGL touchscreen UI on the V5 Brain. Alliance selection (Red/Blue/None), autonomous routine cycling, skills mode toggle, IMU recalibration (the "Recal" button, once `enableDryRun()` has been called). Registers up to 16 auton routines by name + callback. |
| **PathPreview** | Renders autonomous routine paths on the brain screen via dry-run simulation. The robot's auton function is executed in dry-run mode — the resulting `dryRunPath()` trajectory is drawn as line segments with waypoint markers and a robot orientation indicator, on a square field area behind the selector's buttons. Supports field background images. |

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
│   ├── Chassis.hpp             ← coordination: Chassis (motion queue, tasks, timeouts,
│   │                              stall exit, cancel, dry-run), Drivetrain, DriveCurve, ChassisConfig
│   ├── Tracking/
│   │   ├── Odom_Tracking.hpp   ← odometry (TrackingWheel + OdomSensors + solver)
│   │   ├── MCL_Tracking.hpp    ← Monte Carlo localization (particle filter)
│   │   └── RCL_Tracking.hpp    ← Ray/Ceiling-Line localization
│   ├── Movement_Control/       ← all motion math, one module per movement type
│   │   ├── Motion.hpp          ← Motion interface, MotionContext/Output, drive mixer
│   │   ├── MoveToPoint.hpp     ← MoveDistanceMotion, MoveToPointMotion + params
│   │   ├── TurnToPoint.hpp     ← TurnToHeading/TurnToPoint/SwingMotion + params & enums
│   │   ├── Arc.hpp             ← ArcMotion + arc geometry helpers
│   │   ├── Boomerang.hpp       ← BoomerangMotion + params
│   │   ├── RAMSETE.hpp         ← RamseteMotion + path utilities
│   │   └── Velocity_Profiles.hpp ← profile generation (trapezoidal + S-curve)
│   ├── Auton_Selector/
│   │   ├── GUI.hpp             ← LVGL auton selector
│   │   └── Path_Selector.hpp   ← path preview rendering
│   └── Util/
│       ├── Util.hpp            ← Pose, math, angle utils, Field constants
│       ├── pid.hpp             ← PID controller
│       ├── FastTrig.hpp        ← fast sine/cosine LUT
│       └── ScaledIMU.hpp       ← IMU scale factor wrapper
├── src/FBLib/                  ← implementation files (mirrors include/ layout)
├── src/main.cpp                ← example robot program
└── README.md
```

A movement call such as `chassis.moveToPoint(x, y, …)` builds a `MoveToPointMotion` and queues it. The Chassis motion task calls `start()` once, then `update()` every 10 ms with the latest odometry pose, and applies the returned left/right commands to the drivetrain — or, while previewing, to a simulated pose. The motion ends when it reports `done`, on timeout, stall, or `cancelMotion()`.

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

Tracking wheel conventions (reverse the sensor if a wheel reads the wrong way):
- Vertical wheels read **positive driving forward**; their `offsetIn` is the lateral distance from the tracking center, **+ = right**.
- Horizontal wheels read **positive moving left**; their `offsetIn` is the longitudinal distance, **+ = ahead** of the center.

Offsets matter: a wheel away from the tracking center also rolls when the robot turns in place, and odometry subtracts that so turns don't register as travel.

### MCL

Per-axis noise model in [MCL_Tracking.hpp](include/FBLib/Tracking/MCL_Tracking.hpp#L53-L66):
- **Vertical axis (tracking wheel)**: `trackingWheelVariance = 0.08` (low)
- **Vertical axis (IME fallback)**: `imeVariance = 0.12` (higher — accounts for wheel slip)
- **Horizontal axis (tracking wheel)**: `trackingWheelVariance = 0.08`
- **Horizontal axis (no sensor)**: random walk — `horizDependentVarianceProp * |dVert|` + `horizConstantNoise`

Particle theta is clamped to within `maxThetaDeviation` (0.08 rad ≈ 4.6°) of the odometry heading — the IMU is authoritative for orientation, so MCL (like RCL) only corrects the odometry position, never its heading.

### Field Geometry

Field constants are in the `Field` namespace in [Util.hpp](include/FBLib/Util/Util.hpp#L212-L234). Update these for each season's field layout. MCL wall/obstacle definitions live in [MCL_Tracking.hpp](include/FBLib/Tracking/MCL_Tracking.hpp#L228-L262) — modify `kFieldTargets`, `kFieldCircles`, and `kDisablingLines` per season.

## Design Decisions

- **Motion math in modules, coordination in Chassis.** Controllers are pure `Motion` classes (pose in, drive commands out) that never touch hardware or tasks, so they run identically on the robot and in dry-run simulation. `Chassis` owns the queue, tasks, timeouts, stall exit, cancellation and the motors.
- **IMU is authoritative for rotation.** Every odometry update adds the IMU's change in (scaled, continuous) rotation to the pose heading, so `setPose()`/`setHeading()` choose which way the robot faces on the field and the IMU tracks every turn from there. Sync corrections from MCL/RCL are applied to X/Y only.
- **Async by choice, not by default.** Every movement method supports both blocking and non-blocking modes. A single persistent `pros::Task` handles async motion sequencing with a one-deep queue; `cancelMotion()` retires everything issued before it.
- **Dry-run path preview.** The auton selector runs the selected auton function in dry-run mode to record the full trajectory for on-screen preview — no manual waypoint registration needed. Dry-run is scoped to the task that enables it: that task's motions and `setPose()`/`getPose()` calls use a simulated pose, while real motions from other tasks (autonomous, driver control) keep driving the robot.
- **LemLib-independent.** FBLib is a clean-room implementation. LemLib was used as reference only — no dependency.
- **Per-axis decomposition.** `OdomTracking::consumeDelta()` exposes `{dVert, dHoriz, dTheta}` from sensor channels (not geometrically decomposed from a fused pose), enabling statistically correct per-sensor-type noise in MCL.
