#include "FBLib/Auton_Selector/Path_Selector.hpp"

#include <cmath>
#include <cstring>

#include "FBLib/Util/Util.hpp"
#include "pros/rtos.hpp"

// LVGL headers (provided by PROS toolchain)
#include "liblvgl/lvgl.h"

namespace FBLIB {

// ============================================================================
// PathPreview — renders autonomous movement path on the V5 Brain screen
// ============================================================================

PathPreview::PathPreview()
    : mCanvas(nullptr), mFieldImage(nullptr),
      mScreenWidth(480), mScreenHeight(240),
      mPath(), mAnimating(false), mAnimIndex(0) {}

void PathPreview::init(const void* fieldImage, int screenWidth, int screenHeight) {
    mFieldImage = fieldImage;
    mScreenWidth = screenWidth;
    mScreenHeight = screenHeight;

    // Create a container for the field area
    lv_obj_t* container = lv_obj_create(lv_screen_active());
    lv_obj_set_size(container, mScreenWidth, mScreenHeight);
    lv_obj_set_pos(container, 0, 0);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);

    // If a field image is provided, display it as background
    if (mFieldImage != nullptr) {
        lv_obj_t* bgImg = lv_image_create(container);
        lv_image_set_src(bgImg, mFieldImage);
        lv_obj_set_size(bgImg, mScreenWidth, mScreenHeight);
        lv_obj_align(bgImg, LV_ALIGN_CENTER, 0, 0);
    }

    // Create canvas for drawing path overlay
    mCanvas = lv_canvas_create(container);
    lv_obj_set_size(mCanvas, mScreenWidth, mScreenHeight);
    lv_obj_align(mCanvas, LV_ALIGN_CENTER, 0, 0);

    // Allocate per-instance canvas buffer: RGB565 = 16 bits per pixel
    mCanvasBuf.resize(LV_CANVAS_BUF_SIZE(480, 240, 16, LV_DRAW_BUF_STRIDE_ALIGN));
    lv_canvas_set_buffer(mCanvas, mCanvasBuf.data(), mScreenWidth, mScreenHeight,
                          LV_COLOR_FORMAT_RGB565);

    // Fill with transparent background
    lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
}

void PathPreview::setPath(const std::vector<Pose>& waypoints) {
    mPath = waypoints;
}

void PathPreview::setDynamicPath(const std::vector<Pose>& poses) {
    mPath = poses;
}

void PathPreview::clear() {
    mPath.clear();
    if (mCanvas != nullptr) {
        lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    }
}

void PathPreview::draw() {
    if (mCanvas == nullptr || mPath.empty()) return;

    // Clear canvas
    lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);

    // Begin canvas layer for drawing
    lv_layer_t layer;
    lv_canvas_init_layer(mCanvas, &layer);

    if (mPath.size() < 2) {
        lv_canvas_finish_layer(mCanvas, &layer);
        drawRobot(mPath[0]);
        return;
    }

    // ================================================================
    // Build array of screen-coordinate points
    // ================================================================
    int maxPoints = static_cast<int>(mPath.size());
    if (maxPoints > 4096) maxPoints = 4096;

    // ================================================================
    // Draw path as line segments between consecutive poses
    // ================================================================
    lv_draw_line_dsc_t lineDsc;
    lv_draw_line_dsc_init(&lineDsc);
    lineDsc.color = lv_color_hex(0x00FF00);
    lineDsc.width = 2;
    lineDsc.round_start = 1;
    lineDsc.round_end = 1;

    lv_point_precise_t prev;
    prev.x = static_cast<float>(fieldXToScreen(mPath[0].x));
    prev.y = static_cast<float>(fieldYToScreen(mPath[0].y));

    for (int i = 1; i < maxPoints; i++) {
        lv_point_precise_t curr;
        curr.x = static_cast<float>(fieldXToScreen(mPath[i].x));
        curr.y = static_cast<float>(fieldYToScreen(mPath[i].y));

        lineDsc.p1 = prev;
        lineDsc.p2 = curr;
        lv_draw_line(&layer, &lineDsc);

        prev = curr;
    }

    // ================================================================
    // Draw waypoint markers (small filled squares at intervals)
    // ================================================================
    lv_draw_rect_dsc_t dotDsc;
    lv_draw_rect_dsc_init(&dotDsc);
    dotDsc.bg_color = lv_color_hex(0x00FF00);
    dotDsc.bg_opa = LV_OPA_COVER;
    dotDsc.radius = 3;

    int dotInterval = (maxPoints > 50) ? (maxPoints / 20) : 1;
    for (int i = 0; i < maxPoints; i += dotInterval) {
        int sx = fieldXToScreen(mPath[i].x);
        int sy = fieldYToScreen(mPath[i].y);
        lv_area_t area;
        lv_area_set(&area, sx - 2, sy - 2, sx + 2, sy + 2);
        lv_draw_rect(&layer, &dotDsc, &area);
    }

    // Always draw end point (red)
    if (maxPoints > 0) {
        dotDsc.bg_color = lv_color_hex(0xFF0000);
        dotDsc.radius = 3;
        int sx = fieldXToScreen(mPath[maxPoints - 1].x);
        int sy = fieldYToScreen(mPath[maxPoints - 1].y);
        lv_area_t area;
        lv_area_set(&area, sx - 3, sy - 3, sx + 3, sy + 3);
        lv_draw_rect(&layer, &dotDsc, &area);
    }

    // End canvas layer
    lv_canvas_finish_layer(mCanvas, &layer);

    // Draw robot indicator at the start pose
    if (!mPath.empty()) {
        drawRobot(mPath[0]);
    }
}

void PathPreview::drawRobot(const Pose& pose) {
    if (mCanvas == nullptr) return;

    float cx = static_cast<float>(fieldXToScreen(pose.x));
    float cy = static_cast<float>(fieldYToScreen(pose.y));

    // Robot heading → screen angle (Y-flipped, CW positive on screen)
    float screenAngle = -pose.theta;
    float size = 8.0f;

    // Triangle points using precise (float) coordinates
    lv_layer_t layer;
    lv_canvas_init_layer(mCanvas, &layer);

    lv_draw_triangle_dsc_t triDsc;
    lv_draw_triangle_dsc_init(&triDsc);
    triDsc.bg_color = lv_color_hex(0xFFFF00);
    triDsc.bg_opa = LV_OPA_COVER;

    triDsc.p[0].x = cx + size * std::cos(screenAngle);
    triDsc.p[0].y = cy + size * std::sin(screenAngle);
    triDsc.p[1].x = cx + size * 0.5f * std::cos(screenAngle + 2.5f);
    triDsc.p[1].y = cy + size * 0.5f * std::sin(screenAngle + 2.5f);
    triDsc.p[2].x = cx + size * 0.5f * std::cos(screenAngle - 2.5f);
    triDsc.p[2].y = cy + size * 0.5f * std::sin(screenAngle - 2.5f);

    lv_draw_triangle(&layer, &triDsc);
    lv_canvas_finish_layer(mCanvas, &layer);
}

int PathPreview::fieldXToScreen(float fieldX) const {
    return static_cast<int>((fieldX + Field::FIELD_HALF) *
        (static_cast<float>(mScreenWidth) / Field::FIELD_LENGTH));
}

int PathPreview::fieldYToScreen(float fieldY) const {
    return static_cast<int>((Field::FIELD_HALF - fieldY) *
        (static_cast<float>(mScreenHeight) / Field::FIELD_WIDTH));
}

void PathPreview::animate(int speed) {
    if (mPath.size() < 2 || mAnimating) return;

    mAnimating = true;
    mAnimIndex = 0;

    int delayMs = (speed > 0) ? (100 / speed) : 20;

    while (mAnimating && mAnimIndex < static_cast<int>(mPath.size())) {
        if (mCanvas != nullptr) {
            lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
        }
        draw();
        drawRobot(mPath[mAnimIndex]);

        mAnimIndex++;
        pros::delay(delayMs);
    }

    mAnimating = false;

    // Restore final state with robot at the end
    if (mCanvas != nullptr) {
        lv_canvas_fill_bg(mCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    }
    draw();
}

void PathPreview::stopAnimation() {
    mAnimating = false;
}

}  // namespace FBLIB
