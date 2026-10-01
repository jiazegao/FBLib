#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "FBLib/Util/Util.hpp"
#include "pros/rtos.hpp"

// Forward-declare LVGL types
struct _lv_obj_t;
typedef _lv_obj_t lv_obj_t;
struct _lv_timer_t;
typedef _lv_timer_t lv_timer_t;

namespace FBLIB {

class AutonSelector;

// ============================================================================
// PathPreview — renders autonomous movement path on the V5 Brain screen
// ============================================================================
//
// Two modes:
//   1. Static: registered waypoints drawn as lines/dots on the field
//   2. Dynamic (dry-run): robot's recorded trajectory, a 1:1 map of the
//      simulated movement
//
// Field: 144" x 144", center (0,0), +Y up → a square area as tall as the
// shorter screen side, centered (240x240 in the middle of the 480x240 brain
// screen), showing the field image passed to init(), or else the 24" tiles
// drawn as a grid. A yellow arrow marks the robot, a red dot the end of the
// path. The preview sits behind other widgets on the screen and takes no
// input.
//
// Threading: PROS runs LVGL in its own display task without a lock, and an
// LVGL call from another task while that task is drawing can hang the
// program. Only init() touches LVGL directly (call it from initialize());
// every other method just records what to show, and a timer on the display
// task draws it within ~30 ms. They are safe to call from any task.
// The object must outlive the screen: make it a global.
// ============================================================================

class PathPreview {
public:
    PathPreview();

    /// Create the preview on the active screen, behind the widgets already on
    /// it. fieldImage: optional LVGL image source for the background, framed
    /// like the field square (144" across, +Y up) and stretched to it. An
    /// image the square's size (240x240 on the V5) is drawn unscaled.
    void init(const void* fieldImage = nullptr,
              int screenWidth = 480, int screenHeight = 240);

    /// Set static path from pre-registered waypoints (shown by draw())
    void setPath(const std::vector<Pose>& waypoints);

    /// Set dynamic path from dry-run recording (shown by draw())
    void setDynamicPath(const std::vector<Pose>& poses);

    /// Remove the path from the screen
    void clear();

    /// Show the path, with the robot at its start
    void draw();

    /// Move the robot indicator to a pose
    void drawRobot(const Pose& pose);

    /// Field → screen coordinate conversion
    int fieldXToScreen(float fieldX) const;
    int fieldYToScreen(float fieldY) const;

    /// Play the path back by moving the robot along it (speed 10 = real time
    /// for a dry-run recording, 5 = half speed). Returns immediately; does
    /// nothing while already playing. The robot returns to the start at the end.
    void animate(int speed = 5);
    void stopAnimation();
    bool isAnimating() const { return mAnimating.load(); }

private:
    friend class AutonSelector;  // moves the preview onto the selector's screen

    static void timerCb(lv_timer_t* timer);
    // Display task only
    void refresh();
    void renderPath();
    void showRobot(const Pose& pose);
    void showStart();
    float toCanvasX(float fieldX) const;
    float toCanvasY(float fieldY) const;

    // Geometry, fixed by init()
    int mScreenWidth{480};
    int mScreenHeight{240};
    int mSide{240};       // field square, pixels
    int mOriginX{120};    // its top-left corner on the screen
    int mOriginY{0};

    // Requests from any task, guarded by mMutex
    pros::Mutex mMutex;
    std::vector<Pose> mPath;
    uint32_t mPathSerial{0};           // bumped whenever mPath changes
    bool mDrawRequested{false};
    bool mRobotRequested{false};
    Pose mRobotRequest{};
    std::atomic<bool> mDirty{false};   // a request waits for the display task
    std::atomic<bool> mAnimating{false};
    std::atomic<bool> mAnimRestart{false};
    std::atomic<int> mAnimSpeed{5};

    // Display task only (after init())
    const void* mFieldImage{nullptr};
    std::atomic<lv_obj_t*> mContainer{nullptr};
    lv_obj_t* mCanvas{nullptr};
    lv_obj_t* mRobot{nullptr};
    std::vector<uint8_t> mCanvasBuf;
    std::vector<uint8_t> mRobotBuf;
    std::vector<Pose> mShown;          // path currently on screen
    uint32_t mShownSerial{0};
    float mAnimPos{0.0f};              // index into mShown while playing
    uint32_t mAnimLastMs{0};
    bool mAnimRunning{false};
};

}  // namespace FBLIB
