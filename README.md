# FBLib

A C++ robotics library for VEX V5RC, built on the PROS kernel. It provides:

- **Localization:** odometry (IMU + tracking wheels or drive encoders), Monte Carlo localization (MCL) and wall-distance localization (RCL) with V5 distance sensors.
- **Motion control:** move a distance, move to a point, turn to a heading or point, swing turns, arcs, boomerang curves to a pose, and RAMSETE path following, all with blocking or async calls.
- **Autonomous selector:** an LVGL touchscreen menu that previews each routine's path on the brain screen by simulating it.

**Target:** VEX V5 Brain, PROS kernel 4.2.2 · **Language:** C++20 or newer (the PROS template builds with gnu++26) · **Namespace:** `FBLIB`

## Quick start

```cpp
#include "main.h"
#include "FBLib/FB_API.hpp"
using namespace FBLIB;

// Hardware. Give the motor groups their gearset: odometry uses it to turn
// motor rotations into wheel travel.
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);
pros::Imu imu(15);

// Track width 10.4", 3.25" wheels, 450 rpm at the wheels
Drivetrain drivetrain(&leftMotors, &rightMotors, 10.4f, 3.25f, 450.0f);

// IMU for heading, drive motor encoders for distance (no tracking wheels)
OdomSensors sensors{
    .vertWheelCollection = {},
    .horizWheelCollection = {},
    .imuCollection = {&imu},
    .drivetrain = &drivetrain,
};

Chassis chassis(drivetrain, sensors);  // starts the odometry and motion tasks

void initialize() {
    chassis.calibrate();  // ~2 s, the robot must be still
}

void autonomous() {
    chassis.setPose({0, 0, vexToStdRad(0)});    // at the origin, facing +Y (VEX 0°)
    chassis.moveToPoint(0, 24, 2000);           // 24" forward, 2 s timeout
    chassis.turnToHeading(90, 1000);            // turn right to face +X
    chassis.moveDistance(-12, 1500, {}, true);  // back up 12" without blocking
    chassis.waitUntilSettled();
}
```

[src/main.cpp](src/main.cpp) is a complete robot program using the library.

## Conventions

| Quantity | Unit and direction |
|---|---|
| Field coordinates | inches; you choose the origin with `setPose()` |
| `Pose.theta` | radians, standard math: 0 = +X, counter-clockwise positive |
| Headings you pass in (`turnToHeading`, `swingToHeading`, `moveBoomerang`, `setHeading`) | VEX degrees: 0° = +Y, 90° = +X, clockwise positive |
| Heading tolerances, `TurnToPointParams::offset` | VEX degrees |
| Drive commands, `maxSpeed`, `minSpeed` | motor units, -127 to 127 |
| Timeouts | milliseconds, 0 = none |

Convert with `vexToStdRad()` and `stdRadToVexDeg()` when you work with `Pose.theta` directly.

## Moving the robot

| Call | What it does |
|---|---|
| `moveDistance(dist, timeout, params, async)` | Drives `dist` inches along the heading the robot had when the motion started (negative = backwards). |
| `moveToPoint(x, y, …)` | Steers to a point. Inside 7.5" it turns in place until aimed, then holds that heading, so it can't orbit the point; an overshoot backs up. |
| `turnToHeading(deg, …)` | Turns in place to a VEX heading. |
| `turnToPoint(x, y, …)` | Turns in place to face a point. |
| `swingToHeading(deg, side, …)` | Turns by pivoting on one side: `SwingSide::Left` keeps the left wheels still. |
| `moveArc(x, y, radius, …)` | Follows the circle of `radius` through the start position and the target: positive = counter-clockwise, always the shorter arc, 0 = straight line. |
| `moveBoomerang(x, y, deg, …)` | Curves into a pose, arriving at heading `deg`. |
| `moveRAMSETE(path, …)` | Follows a list of poses; each `theta` is the direction of travel, in radians. |
| `runMotion(motion, timeout, async)` | Runs your own `Motion` subclass (see [Custom motions](#custom-motions)). |

### Parameters

Each call takes an optional params struct; designated initializers keep it short:

```cpp
chassis.moveToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = 80});
```

| Field | Default | Meaning | Used by |
|---|---|---|---|
| `forwards` | `true` | `false` drives backwards | distance, point, arc, boomerang |
| `maxSpeed` | 127 | output limit | all |
| `minSpeed` | 0 | if > 0, never slows below this and ends as soon as it passes the target (motion chaining) | all; for RAMSETE it is the slowest profiled speed |
| `targetTolerance` | 1" or 2° | settle window | all |
| `earlyExitRange` | 0 | ends the motion this far from the target | distance, point |
| `direction` | `SHORTEST` | `CW` or `CCW` forces the turn direction until the robot first passes the target | turns, swing |
| `offset` | 0 | faces this many VEX degrees clockwise of the point (180 = face away) | turn to point |
| `headingTolerance` | 2° | final heading window | boomerang, RAMSETE |
| `lead` | 0.5 | how far behind the target the boomerang's carrot point sits, as a fraction of the distance | boomerang |
| `leadDecay` | 0 | fraction of that offset dropped while far away | boomerang |
| `b`, `zeta` | 2.0, 0.7 | RAMSETE gains (meter-based textbook values; scaled to inches internally) | RAMSETE |
| `lookaheadDist` | 2" | reference point this far ahead along the path | RAMSETE |
| `useVelocityProfile`, `maxAccel` | `true`, 50 in/s² | accelerate into and slow down at the end of the path | RAMSETE |

`MoveToPointParams::lead` and `ArcParams::lead` have no effect. Use `moveBoomerang()` for a curved approach.

### Blocking, async and when a motion ends

A call blocks until its motion ends unless you pass `async = true`. A single background task runs motions in order, with at most one queued behind the running one. Calling a second async motion queues it, and a third waits until the queue has room.

```cpp
chassis.moveToPoint(48, 24, 3000, {}, true);  // returns immediately
chassis.waitUntilDist(12);                    // after 12" of travel (or when the motion ends)
intake.move(127);
chassis.waitUntilSettled();                   // until every queued motion is done
bool idle = chassis.isSettled();              // non-blocking check
chassis.cancelMotion();                       // stop the running motion and drop queued ones
```

A motion ends when the first of these happens:
1. It reaches its target (within tolerance), its `earlyExitRange`, or, with `minSpeed > 0`, passes the target.
2. Its timeout expires.
3. It stops making progress for `ChassisConfig::stallTimeoutMs` (default 1000 ms; 0 disables this). This covers a pinned robot, or friction stopping it just short of its tolerance, even when the timeout is 0.
4. `cancelMotion()` is called.

Joystick input to `tank()`, `arcade()` and `curvature()` is ignored while a motion is queued or running, so a macro triggered from driver control isn't fought by the sticks.

### Custom motions

Every movement is a `Motion` subclass ([Motion.hpp](include/FBLib/Movement_Control/Motion.hpp)): it receives the current pose every 10 ms and returns left/right commands. Your own subclasses get the same queueing, timeouts, cancellation and dry-run preview:

```cpp
class DriveForTime : public Motion {
public:
    explicit DriveForTime(float seconds) : mSeconds(seconds) {}

    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override {
        mElapsed += dt;
        MotionOutput out;
        out.left = 60;
        out.right = 60;
        out.done = mElapsed >= mSeconds;
        return out;  // leaving out.remaining unset opts out of the stall exit
    }

private:
    float mSeconds;
    float mElapsed = 0;
};

chassis.runMotion(std::make_unique<DriveForTime>(0.5f));
```

`MotionContext` gives a motion the shared lateral and angular PID controllers, the track width and the drivetrain's top speed. Set `MotionOutput::remaining` (wheel travel still needed, in inches) if you want the stall exit to apply. The helpers in Motion.hpp mix lateral and angular commands into left/right (`mixDrive`, positive angular = counter-clockwise) and apply a minimum speed (`applyMinSpeed`).

## Localization

### Odometry

Odometry runs in its own task at 100 Hz from the moment the `Chassis` is constructed.

- **Heading:** the IMU is authoritative for rotation. Each update adds the IMU's change in rotation to the pose heading, so `setPose()` and `setHeading()` decide which way the robot faces on the field, and the IMU tracks every turn from there. After `calibrate()` the heading follows the IMU: VEX 0° = +Y.
- **Forward distance:** the average of `vertWheelCollection`; without tracking wheels, the drive motor encoders (set `OdomSensors::drivetrain`); otherwise none.
- **Sideways distance:** the average of `horizWheelCollection`; a tank drive's motor encoders can't measure it.

Tracking wheels can use V5 Rotation sensors or ADI encoders:

```cpp
pros::Rotation vertRot(5);
pros::Rotation horizRot(-6);                        // negative port = reversed
TrackingWheel vertWheel(vertRot, 2.75f, 1.5f);      // 2.75" wheel, 1.5" right of center
TrackingWheel horizWheel(horizRot, 2.75f, -3.0f);   // 3" behind center

OdomSensors sensors{
    .vertWheelCollection = {&vertWheel},
    .horizWheelCollection = {&horizWheel},
    .imuCollection = {&imu},
};
```

- **Sign conventions:** vertical wheels must read positive driving forward, and horizontal wheels positive moving **left**. Reverse the sensor if one reads the wrong way.
- **Offsets:** a vertical wheel's offset is how far it sits right (+) or left (-) of the tracking center; a horizontal wheel's is how far ahead (+) or behind (-) it is. Odometry uses them to cancel the travel a wheel registers while the robot turns in place, so measure them.

V5 IMUs often under-report rotation (for example 354.25° for a full turn). Correct it in one of two ways, never both:
- `ScaledIMU imu(15, 360.0, 354.25);` in place of `pros::Imu`.
- `OdomSensors::imuScaleFactor = 360.0f / 354.25f;` with a plain `pros::Imu`.

### MCL and RCL

Both use V5 distance sensors to correct the odometry **position**. They never change the heading.

| | How it works |
|---|---|
| **MCL** | A 1024-particle filter at 40 Hz. Particles move with odometry and are weighted by comparing each distance reading with a ray cast against the field walls and any field elements you list. It gently pulls the odometry position toward the estimate (10% per update). |
| **RCL** | At 100 Hz, each sensor that points squarely at a wall gives one coordinate (east/west walls → X, north/south → Y), averaged per axis. It moves odometry toward that, at most 5" per update. |

```cpp
pros::Distance frontDist(1), leftDist(2);

ChassisConfig config{
    // up to MAX_DISTANCE_SENSORS (8); slots may be left empty
    .distanceSensors = {&frontDist, &leftDist},
    // matching mounts: x forward, y left, theta = pointing angle (rad)
    .sensorMounts = {{
        {6.0f, 0.0f, 0.0f},     // front: 6" ahead of center, pointing forward
        {0.0f, 3.5f, HALF_PI},  // left: 3.5" left of center, pointing left
    }},
    .useMclTracking = true,     // and/or .useRclTracking
};

Chassis chassis(drivetrain, sensors, config);

void autonomous() {
    chassis.setPose({-48, -60, vexToStdRad(0)});
    chassis.startTracking();   // starts MCL/RCL tasks for the enabled systems
    // ...
}
```

Field geometry:
- **Walls:** the 140.4" × 140.4" perimeter is built in.
- **MCL field elements:** for the current season's game, list them in `kFieldTargets`, `kFieldCircles` and `kDisablingLines` in [MCL_Tracking.hpp](include/FBLib/Tracking/MCL_Tracking.hpp). A sensor whose ray crosses a disabling line is ignored for that update.
- **RCL obstacles:** add sensor-blocking obstacles at runtime with `chassis.rcl().addLineObstacle(...)` or `addCircleObstacle(...)`. A lifetime in ms makes one temporary.

## Autonomous selector and path preview

```cpp
AutonSelector selector;
PathPreview preview;

void leftSide() {
    chassis.setPose({-48, -60, vexToStdRad(0)});
    chassis.moveToPoint(-48, -24, 2000);
    if (!chassis.isDryRun()) intake.move(127);  // don't run mechanisms during a preview
}

void initialize() {
    chassis.calibrate();
    selector.registerAuton("Left side", leftSide);  // up to 16 routines
    selector.init();                                // builds the selector screen
    preview.init();                                 // field area behind the buttons
    selector.enableDryRun(chassis, preview);        // preview on every selection change
}

void autonomous() {
    selector.runSelected();
}
```

On the brain screen, the selector has buttons for alliance color (none/red/blue), routine, Match/Skills mode and Recal. Recal recalibrates the IMU, which takes about 2 s; it needs the chassis passed to `enableDryRun()`.

`runSelected()` does nothing until an alliance is chosen, unless Skills mode is on.

To preview, the selector calls your routine in **dry-run mode**:
- **Simulated, not executed:** its drive motions run against a simulated robot, and `setPose()` and `getPose()` act on the simulated pose.
- **Recorded:** the path is recorded and drawn on a square field (240×240 px on the 480×240 screen).
- **Isolated:** dry-run applies only to the task running the preview, so a real motion running from autonomous or driver control is unaffected. Previews are skipped while a real motion is running.

The routine itself really executes, so:
- Anything that isn't a chassis motion, such as intake motors or pneumatics, really happens unless you guard it with `chassis.isDryRun()`.
- A `pros::delay()` really waits, and the screen freezes for that long.

`PathPreview::init(fieldImage)` takes an optional LVGL image to draw behind the path.

## Tuning

PID errors are in inches (lateral) and radians (angular); outputs are motor units. The defaults in `ChassisConfig` are lateral kP 10, kD 0.3 and angular kP 100, kD 5. They are reasonable starting points for a 450 rpm drive with 3.25" wheels.

1. **Tune the angular gains first,** with `turnToHeading`. Raise kP until turns overshoot, then raise kD until the overshoot is gone.
2. **Then tune the lateral gains,** with `moveDistance`.
3. **Add `kI` if a motion stops short.** A small kI with a `windupRange` helps when motions consistently stop a few degrees or inches early. Friction can stop the P term short of the tolerance; the motion then ends through the stall exit about 1 s later.
4. **Gain changes apply to the next motion.** `setLateralGains()` and `setAngularGains()` don't affect a motion that is already running.
5. **RAMSETE:** keep `lookaheadDist` short. Long lookaheads cut corners.
6. **Boomerang:** raise `lead` for wider curves. Raising `leadDecay` straightens the approach, but leaves less room to line up the final heading.

## Architecture

```
FBLib/
├── include/FBLib/
│   ├── FB_API.hpp                ← single include for the whole library
│   ├── Chassis.hpp               ← coordination: Chassis (motion queue, tasks, timeouts,
│   │                                stall exit, cancel, dry-run), Drivetrain, DriveCurve, ChassisConfig
│   ├── Movement_Control/         ← all motion math, one module per movement type
│   │   ├── Motion.hpp            ← Motion interface, MotionContext/MotionOutput, drive mixer
│   │   ├── MoveToPoint.hpp       ← MoveDistanceMotion, MoveToPointMotion
│   │   ├── TurnToPoint.hpp       ← TurnToHeadingMotion, TurnToPointMotion, SwingMotion
│   │   ├── Arc.hpp               ← ArcMotion + arc geometry helpers
│   │   ├── Boomerang.hpp         ← BoomerangMotion
│   │   ├── RAMSETE.hpp           ← RamseteMotion + path helpers
│   │   └── Velocity_Profiles.hpp ← trapezoidal and S-curve profiles
│   ├── Tracking/
│   │   ├── Odom_Tracking.hpp     ← TrackingWheel, OdomSensors, OdomTracking
│   │   ├── MCL_Tracking.hpp      ← Monte Carlo localization
│   │   └── RCL_Tracking.hpp      ← wall-distance localization
│   ├── Auton_Selector/
│   │   ├── GUI.hpp               ← AutonSelector
│   │   └── Path_Selector.hpp     ← PathPreview
│   └── Util/
│       ├── Util.hpp              ← Pose, angle and unit helpers, Field constants
│       ├── pid.hpp               ← PID controller
│       ├── FastTrig.hpp          ← lookup-table sin/cos for MCL
│       └── ScaledIMU.hpp         ← IMU with a scale correction
├── src/FBLib/                    ← implementations, same layout as include/FBLib
└── src/main.cpp                  ← example robot program
```

The motion math and the coordination are separate:
1. **Build:** `chassis.moveToPoint(...)` creates a `MoveToPointMotion` and queues it.
2. **Run:** the motion task calls the motion's `start()` once, then `update()` every 10 ms with the latest pose. It sends the returned commands to the motors, or to the simulated robot while previewing.
3. **End:** the task ends the motion when it reports done, or on timeout, stall or cancellation.

The motion classes never touch hardware or tasks, which is why they behave identically on the robot and in the preview.

The `Chassis` constructor starts two tasks:
- **Odometry**, at 100 Hz: the only caller of `OdomTracking::update()`.
- **Motion:** the only task that runs motions.

`startTracking()` adds the MCL (40 Hz) and RCL (100 Hz) tasks. The movement, pose, calibration and tracking calls lock the state they share, so you can make them from any task. The selector and preview use LVGL, so call those from one task.

## Build

A standard PROS project; it needs the [PROS CLI](https://pros.cs.purdue.edu/).

```bash
pros make          # compile
pros make clean    # clean build
pros upload        # upload to the V5 Brain
```

The Makefile builds an application (`IS_LIBRARY:=0`). To build a reusable library template instead, set `IS_LIBRARY:=1` and `LIBNAME:=FBLib`.

## Design decisions

- **Motion math in modules, coordination in Chassis.** Controllers are plain classes, pose in and drive commands out, so they can be read, tested and previewed on their own.
- **The IMU is authoritative for rotation.** MCL and RCL only correct position, so a distance sensor seeing a game element can never spin the robot's idea of its heading.
- **Async by choice, not by default.** Every movement can block or not, and one task runs them all in order.
- **Previews run the real routine.** No waypoints to keep in sync: the preview is whatever the routine actually does, simulated.
- **No hangs.** With the stall exit, a motion without a timeout still ends when the robot stops making progress.
- **LemLib-independent.** A clean-room implementation; LemLib was a reference only.
