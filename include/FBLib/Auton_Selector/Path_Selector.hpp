#pragma once

#include <vector>

#include "FBLib/Util/Util.hpp"

// Forward-declare LVGL types
struct _lv_obj_t;
typedef _lv_obj_t lv_obj_t;

namespace FBLIB {

// ============================================================================
// PathPreview — renders autonomous movement path on the V5 Brain screen
// ============================================================================
//
// Two modes:
//   1. Static: registered waypoints drawn as lines/dots on field background
//   2. Dynamic (dry-run): robot's recorded trajectory drawn in real-time,
//      producing a 1:1 map of actual movement
//
// Field: 144" x 144", center (0,0) → Screen: 480x240, origin top-left
// ============================================================================

class PathPreview {
public:
    PathPreview();

    /// Initialize with optional field image background
    void init(const void* fieldImage = nullptr,
              int screenWidth = 480, int screenHeight = 240);

    /// Set static path from pre-registered waypoints
    void setPath(const std::vector<Pose>& waypoints);

    /// Set dynamic path from dry-run recording
    void setDynamicPath(const std::vector<Pose>& poses);

    /// Clear the displayed path
    void clear();

    /// Draw/redraw the path
    void draw();

    /// Draw robot indicator at a pose
    void drawRobot(const Pose& pose);

    /// Field → screen coordinate conversion
    int fieldXToScreen(float fieldX) const;
    int fieldYToScreen(float fieldY) const;

    /// Animate path (move robot dot along recorded path)
    void animate(int speed = 5);
    void stopAnimation();
    bool isAnimating() const { return mAnimating; }

private:
    lv_obj_t* mCanvas{nullptr};
    const void* mFieldImage{nullptr};
    int mScreenWidth{480};
    int mScreenHeight{240};
    std::vector<Pose> mPath;
    bool mAnimating{false};
    int mAnimIndex{0};
    std::vector<uint8_t> mCanvasBuf;  // per-instance canvas buffer (was static, Bug 4)
};

}  // namespace FBLIB
