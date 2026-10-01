#include "FBLib/Auton_Selector/Path_Selector.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "FBLib/Util/Util.hpp"
#include "pros/rtos.hpp"

// LVGL headers (provided by PROS toolchain)
#include "liblvgl/lvgl.h"

namespace FBLIB {

// ============================================================================
// PathPreview — renders autonomous movement path on the V5 Brain screen
// ============================================================================

namespace {
constexpr uint32_t REFRESH_PERIOD_MS = 30;
constexpr int ROBOT_RADIUS = 10;        // robot marker canvas is (2R+1)^2 px
constexpr float ROBOT_SIZE = 8.0f;      // arrow length, px
constexpr float MIN_SEGMENT_PX = 1.5f;  // shorter path segments are merged
constexpr int TILES = 6;                // 24" tiles per field side

lv_point_precise_t point(float x, float y) {
    lv_point_precise_t p;
    p.x = static_cast<lv_value_precise_t>(std::lround(x));
    p.y = static_cast<lv_value_precise_t>(std::lround(y));
    return p;
}

void drawDot(lv_layer_t* layer, float x, float y, int half, uint32_t color) {
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color = lv_color_hex(color);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.radius = 3;
    int cx = static_cast<int>(std::lround(x));
    int cy = static_cast<int>(std::lround(y));
    lv_area_t area;
    lv_area_set(&area, cx - half, cy - half, cx + half, cy + half);
    lv_draw_rect(layer, &dsc, &area);
}
}  // namespace

PathPreview::PathPreview() = default;

void PathPreview::init(const void* fieldImage, int screenWidth, int screenHeight) {
    if (mContainer.load() != nullptr) return;  // already built

    mFieldImage = fieldImage;
    mScreenWidth = screenWidth;
    mScreenHeight = screenHeight;
    mSide = std::max(1, std::min(screenWidth, screenHeight));
    mOriginX = (screenWidth - mSide) / 2;
    mOriginY = (screenHeight - mSide) / 2;

    // The field square goes BEHIND everything else on the screen and never
    // takes input, so the AutonSelector's buttons stay visible and clickable.
    lv_obj_t* container = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, mOriginX, mOriginY);
    lv_obj_set_size(container, mSide, mSide);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_remove_flag(container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_to_index(container, 0);

    if (mFieldImage != nullptr) {
        lv_obj_t* bgImg = lv_image_create(container);
        lv_image_set_src(bgImg, mFieldImage);
        lv_obj_set_pos(bgImg, 0, 0);
        lv_obj_set_size(bgImg, mSide, mSide);
        lv_image_set_inner_align(bgImg, LV_IMAGE_ALIGN_STRETCH);
    }

    // Path layer. ARGB8888 so the field image shows through.
    mCanvas = lv_canvas_create(container);
    mCanvasBuf.resize(LV_CANVAS_BUF_SIZE(mSide, mSide, 32, LV_DRAW_BUF_STRIDE_ALIGN));
    lv_canvas_set_buffer(mCanvas, mCanvasBuf.data(), mSide, mSide, LV_COLOR_FORMAT_ARGB8888);
    lv_obj_set_pos(mCanvas, 0, 0);
    lv_obj_remove_flag(mCanvas, LV_OBJ_FLAG_CLICKABLE);

    // Robot marker: its own small canvas, moved around on top of the path, so
    // playing the path back never redraws the path itself.
    const int robotSide = 2 * ROBOT_RADIUS + 1;
    mRobot = lv_canvas_create(container);
    mRobotBuf.resize(LV_CANVAS_BUF_SIZE(robotSide, robotSide, 32, LV_DRAW_BUF_STRIDE_ALIGN));
    lv_canvas_set_buffer(mRobot, mRobotBuf.data(), robotSide, robotSide, LV_COLOR_FORMAT_ARGB8888);
    lv_obj_remove_flag(mRobot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(mRobot, LV_OBJ_FLAG_HIDDEN);

    renderPath();  // empty field
    mContainer = container;
    lv_timer_create(timerCb, REFRESH_PERIOD_MS, this);
    mDirty = true;  // show whatever was requested before init()
}

void PathPreview::setPath(const std::vector<Pose>& waypoints) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPath = waypoints;
    mPathSerial++;
}

void PathPreview::setDynamicPath(const std::vector<Pose>& poses) {
    setPath(poses);
}

void PathPreview::clear() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPath.clear();
    mPathSerial++;
    mDrawRequested = true;
    mAnimating = false;
    mAnimRestart = false;
    mDirty = true;
}

void PathPreview::draw() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mDrawRequested = true;
    mAnimating = false;
    mAnimRestart = false;
    mDirty = true;
}

void PathPreview::drawRobot(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mRobotRequest = pose;
    mRobotRequested = true;
    mDirty = true;
}

int PathPreview::fieldXToScreen(float fieldX) const {
    return static_cast<int>(std::lround(Field::fieldToScreenX(fieldX, mScreenWidth, mScreenHeight)));
}

int PathPreview::fieldYToScreen(float fieldY) const {
    return static_cast<int>(std::lround(Field::fieldToScreenY(fieldY, mScreenHeight, mScreenWidth)));
}

void PathPreview::animate(int speed) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    if (mAnimating.load() || mPath.size() < 2) return;
    mAnimSpeed = (speed > 0) ? speed : 5;
    mDrawRequested = true;  // play the latest path
    mAnimRestart = true;
    mAnimating = true;
    mDirty = true;
}

void PathPreview::stopAnimation() {
    mAnimating = false;
    mAnimRestart = false;
}

// ============================================================================
// Display task
// ============================================================================

void PathPreview::timerCb(lv_timer_t* timer) {
    static_cast<PathPreview*>(lv_timer_get_user_data(timer))->refresh();
}

void PathPreview::refresh() {
    if (mDirty.load()) {
        // Never wait on another task here: the display task would stall.
        std::unique_lock<pros::Mutex> lock(mMutex, std::try_to_lock);
        if (lock.owns_lock()) {
            mDirty = false;
            const bool drawRequested = mDrawRequested;
            const bool newPath = drawRequested && mPathSerial != mShownSerial;
            if (newPath) {
                mShown = mPath;
                mShownSerial = mPathSerial;
            }
            const bool robotRequested = mRobotRequested;
            const Pose robotPose = mRobotRequest;
            mDrawRequested = false;
            mRobotRequested = false;
            lock.unlock();

            if (newPath) renderPath();
            if (drawRequested) showStart();
            if (robotRequested) showRobot(robotPose);
        }
    }

    // Playback (only once the path it plays is on screen)
    if (!mDirty.load() && mAnimRestart.exchange(false)) {
        mAnimPos = 0.0f;
        mAnimLastMs = lv_tick_get();
        mAnimRunning = true;
    }
    if (mAnimRunning && mAnimating.load()) {
        const uint32_t now = lv_tick_get();
        // Recorded poses are 10 ms apart: speed 10 plays them in real time.
        mAnimPos += static_cast<float>(now - mAnimLastMs) * static_cast<float>(mAnimSpeed.load()) / 100.0f;
        mAnimLastMs = now;
        const size_t index = static_cast<size_t>(mAnimPos);
        if (index < mShown.size()) {
            showRobot(mShown[index]);
        } else {
            mAnimating = false;  // played to the end
        }
    }
    if (mAnimRunning && !mAnimating.load()) {
        mAnimRunning = false;
        showStart();
    }
}

float PathPreview::toCanvasX(float fieldX) const {
    return Field::fieldToScreenX(fieldX, mScreenWidth, mScreenHeight) - static_cast<float>(mOriginX);
}

float PathPreview::toCanvasY(float fieldY) const {
    return Field::fieldToScreenY(fieldY, mScreenHeight, mScreenWidth) - static_cast<float>(mOriginY);
}

void PathPreview::renderPath() {
    lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);

    lv_layer_t layer;
    lv_canvas_init_layer(mCanvas, &layer);

    lv_draw_line_dsc_t lineDsc;
    lv_draw_line_dsc_init(&lineDsc);

    // Field tiles and walls (a field image brings its own)
    if (mFieldImage == nullptr) {
        const float last = static_cast<float>(mSide - 1);
        lineDsc.color = lv_color_hex(0x2C2C2C);
        lineDsc.width = 1;
        for (int i = 1; i < TILES; i++) {
            float p = std::round(static_cast<float>(mSide * i) / TILES);
            lineDsc.p1 = point(p, 0.0f);
            lineDsc.p2 = point(p, last);
            lv_draw_line(&layer, &lineDsc);
            lineDsc.p1 = point(0.0f, p);
            lineDsc.p2 = point(last, p);
            lv_draw_line(&layer, &lineDsc);
        }
        lv_draw_rect_dsc_t wall;
        lv_draw_rect_dsc_init(&wall);
        wall.bg_opa = LV_OPA_TRANSP;
        wall.border_color = lv_color_hex(0x5A5A5A);
        wall.border_width = 2;
        wall.border_opa = LV_OPA_COVER;
        lv_area_t area;
        lv_area_set(&area, 0, 0, mSide - 1, mSide - 1);
        lv_draw_rect(&layer, &wall, &area);
    }

    if (!mShown.empty()) {
        // Path: one segment per pose would be ~1500 line draws for a 15 s
        // routine; merge points closer than a pixel and a half. The last
        // pose always ends the line.
        lineDsc.color = lv_color_hex(0x00FF00);
        lineDsc.width = 2;
        lineDsc.round_start = 1;
        lineDsc.round_end = 1;
        float px = toCanvasX(mShown[0].x);
        float py = toCanvasY(mShown[0].y);
        for (size_t i = 1; i < mShown.size(); i++) {
            float x = toCanvasX(mShown[i].x);
            float y = toCanvasY(mShown[i].y);
            if (i + 1 < mShown.size() && std::hypot(x - px, y - py) < MIN_SEGMENT_PX) continue;
            lineDsc.p1 = point(px, py);
            lineDsc.p2 = point(x, y);
            lv_draw_line(&layer, &lineDsc);
            px = x;
            py = y;
        }

        // Markers at even time intervals (~20 along a recording)
        const size_t count = mShown.size();
        const size_t interval = (count > 50) ? count / 20 : 1;
        for (size_t i = 0; i < count; i += interval) {
            drawDot(&layer, toCanvasX(mShown[i].x), toCanvasY(mShown[i].y), 2, 0x00FF00);
        }

        // End of the path
        drawDot(&layer, toCanvasX(mShown.back().x), toCanvasY(mShown.back().y), 3, 0xFF0000);
    }

    lv_canvas_finish_layer(mCanvas, &layer);
}

void PathPreview::showRobot(const Pose& pose) {
    lv_canvas_fill_bg(mRobot, lv_color_hex(0x000000), LV_OPA_TRANSP);

    lv_layer_t layer;
    lv_canvas_init_layer(mRobot, &layer);

    lv_draw_triangle_dsc_t triDsc;
    lv_draw_triangle_dsc_init(&triDsc);
    triDsc.bg_opa = LV_OPA_COVER;

    // Screen Y points down, so a counter-clockwise field heading is a
    // clockwise screen angle.
    const float a = -pose.theta;
    const float c = static_cast<float>(ROBOT_RADIUS);
    const lv_point_precise_t arrow[3] = {
        point(c + ROBOT_SIZE * std::cos(a), c + ROBOT_SIZE * std::sin(a)),
        point(c + ROBOT_SIZE * 0.5f * std::cos(a + 2.5f), c + ROBOT_SIZE * 0.5f * std::sin(a + 2.5f)),
        point(c + ROBOT_SIZE * 0.5f * std::cos(a - 2.5f), c + ROBOT_SIZE * 0.5f * std::sin(a - 2.5f)),
    };

    // A 1 px dark outline (the arrow shifted every way) keeps it visible on
    // the path and over a field image's yellow elements
    triDsc.bg_color = lv_color_hex(0x000000);
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            for (int i = 0; i < 3; i++) {
                triDsc.p[i] = arrow[i];
                triDsc.p[i].x += dx;
                triDsc.p[i].y += dy;
            }
            lv_draw_triangle(&layer, &triDsc);
        }
    }
    triDsc.bg_color = lv_color_hex(0xFFFF00);
    for (int i = 0; i < 3; i++) triDsc.p[i] = arrow[i];
    lv_draw_triangle(&layer, &triDsc);

    lv_canvas_finish_layer(mRobot, &layer);

    lv_obj_set_pos(mRobot, static_cast<int32_t>(std::lround(toCanvasX(pose.x))) - ROBOT_RADIUS,
                   static_cast<int32_t>(std::lround(toCanvasY(pose.y))) - ROBOT_RADIUS);
    lv_obj_remove_flag(mRobot, LV_OBJ_FLAG_HIDDEN);
}

void PathPreview::showStart() {
    if (mShown.empty()) {
        lv_obj_add_flag(mRobot, LV_OBJ_FLAG_HIDDEN);
    } else {
        showRobot(mShown.front());
    }
}

}  // namespace FBLIB
