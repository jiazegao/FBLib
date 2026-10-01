#include "FBLib/Auton_Selector/GUI.hpp"
#include "FBLib/Auton_Selector/Path_Selector.hpp"

#include <cstdio>
#include <mutex>
#include <vector>

#include "FBLib/Chassis.hpp"
#include "FBLib/Util/Util.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

// LVGL headers (provided by PROS toolchain)
#include "liblvgl/lvgl.h"

namespace FBLIB {

// ============================================================================
// AutonSelector — LVGL-based autonomous routine selector for V5 Brain
// ============================================================================

namespace {
constexpr uint32_t REFRESH_PERIOD_MS = 50;
constexpr uint32_t PREVIEW_RECHECK_MS = 100;  // how often a waiting preview re-checks

// Layout: 120 px columns either side of the 240x240 field preview
constexpr int LEFT_X = 6;
constexpr int RIGHT_X = 366;
constexpr int COLUMN_W = 108;

constexpr uint32_t COLOR_NONE = 0x4A4A6A;
constexpr uint32_t COLOR_RED = 0x8B0000;
constexpr uint32_t COLOR_BLUE = 0x00008B;
constexpr uint32_t COLOR_MATCH = 0x3A3A5A;
constexpr uint32_t COLOR_SKILLS = 0x006400;

lv_obj_t* makeLabel(lv_obj_t* parent, const lv_font_t* font, uint32_t color) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}

lv_obj_t* makeButton(lv_obj_t* parent, int x, int y, int w, int h, uint32_t color, lv_event_cb_t cb,
                     void* user) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t* label = makeLabel(btn, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return btn;
}

/// The robot may run a preview: disabled in a match, or no competition
/// control at all (bench testing).
bool robotMayPreview() {
    return !pros::competition::is_connected() || pros::competition::is_disabled();
}
}  // namespace

AutonSelector::AutonSelector() = default;

void AutonSelector::registerAuton(const std::string& name, AutonFunc func) {
    const int n = mCount.load();
    if (n >= MAX_AUTONS) return;
    mAutons[n] = {name, func};
    mCount.store(n + 1);  // publish after the entry is written
    mLabelsDirty = true;
}

void AutonSelector::init() {
    if (mScreen != nullptr) return;  // already built

    // Build the screen off-screen, then show it
    mScreen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(mScreen, lv_color_hex(0x1A1A2E), 0);
    lv_obj_remove_flag(mScreen, LV_OBJ_FLAG_SCROLLABLE);

    // ================================================================
    // Left column: title, routine name, routine button, status
    // ================================================================
    lv_obj_t* title = makeLabel(mScreen, &lv_font_montserrat_12, 0x9AA0B8);
    lv_label_set_text(title, "Auton Selector");
    lv_obj_set_pos(title, LEFT_X, 6);

    mNameLabel = makeLabel(mScreen, &lv_font_montserrat_16, 0xFFFFFF);
    lv_label_set_long_mode(mNameLabel, LV_LABEL_LONG_DOT);
    lv_obj_set_pos(mNameLabel, LEFT_X, 26);
    lv_obj_set_size(mNameLabel, COLUMN_W, 76);

    mTypeBtn = makeButton(mScreen, LEFT_X, 108, COLUMN_W, 56, COLOR_NONE, toggleTypeCb, this);

    mStatusLabel = makeLabel(mScreen, &lv_font_montserrat_12, 0xC8CCDC);
    lv_label_set_long_mode(mStatusLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(mStatusLabel, LEFT_X, 174);
    lv_obj_set_width(mStatusLabel, COLUMN_W);
    lv_label_set_text(mStatusLabel, "");

    // ================================================================
    // Right column: alliance color, Match/Skills, Recal
    // ================================================================
    mColorBtn = makeButton(mScreen, RIGHT_X, 8, COLUMN_W, 56, COLOR_NONE, toggleColorCb, this);
    mSkillsBtn = makeButton(mScreen, RIGHT_X, 72, COLUMN_W, 56, COLOR_MATCH, toggleSkillsCb, this);
    mRecalBtn = makeButton(mScreen, RIGHT_X, 196, COLUMN_W, 36, COLOR_RED, recalibrateCb, this);
    lv_label_set_text(lv_obj_get_child(mRecalBtn, 0), "Recal");

    updateButtonLabels();
    lv_screen_load(mScreen);

    // Everything from here on happens on the display task
    lv_timer_create(timerCb, REFRESH_PERIOD_MS, this);
}

void AutonSelector::update() {
    mLabelsDirty = true;
}

const AutonEntry& AutonSelector::getSelected() const {
    const int index = mSelectedIndex.load();
    if (index >= mCount.load()) {
        static const AutonEntry empty;
        return empty;
    }
    return mAutons[index];
}

void AutonSelector::runSelected() {
    // Match mode: an alliance must be chosen (Skills mode needs none)
    if (!mSkillsMode.load() && mAlliance.load() == Alliance::NONE) return;
    forceRunSelected();
}

void AutonSelector::forceRunSelected() {
    const int index = mSelectedIndex.load();
    if (index < mCount.load() && mAutons[index].function) {
        mAutons[index].function();
    }
}

void AutonSelector::onSelectionChanged(PathPreviewCallback callback) {
    {
        std::lock_guard<pros::Mutex> lock(mCallbackMutex);
        mPathPreviewCallback = std::move(callback);
    }
    mCallbackPending = true;  // fire once from the display task
    requestPreview();
}

void AutonSelector::enableDryRun(Chassis& chassis, PathPreview& preview) {
    mChassis = &chassis;
    mPreview = &preview;
    if (mPreviewTask.load() == nullptr) {
        mPreviewTask = new pros::Task([this] { previewLoop(); }, TASK_PRIORITY_DEFAULT,
                                      TASK_STACK_DEPTH_DEFAULT, "FBLib preview");
    }
    requestPreview();  // preview the current selection now
}

// ============================================================================
// Preview task — runs routines in dry-run mode and recalibrates
// ============================================================================

void AutonSelector::requestPreview() {
    mPreviewRequest++;
    pros::Task* task = mPreviewTask.load();
    if (task != nullptr) task->notify();
}

void AutonSelector::previewLoop() {
    uint32_t previewed = 0;  // newest request already drawn
    while (true) {
        // Woken by a request; waiting requests re-check every 100 ms
        pros::Task::notify_take(true, PREVIEW_RECHECK_MS);

        Chassis* chassis = mChassis.load();
        const uint32_t request = mPreviewRequest.load();
        const bool recal = mRecalRequested.load();
        if (chassis == nullptr || (!recal && request == previewed)) {
            mStatus = Status::IDLE;
            continue;
        }

        // Never during a match period or while the robot is moving: the
        // routine's own (non-motion) code really runs, and calibration needs
        // the robot still.
        if (!robotMayPreview()) {
            mStatus = Status::WAIT_ENABLED;
            continue;
        }
        if (!chassis->isSettled()) {
            mStatus = Status::WAIT_MOTION;
            continue;
        }

        if (recal) {
            mStatus = Status::CALIBRATING;
            chassis->calibrate();
            mRecalRequested = false;
            continue;
        }

        mStatus = Status::PREVIEWING;
        std::vector<Pose> path;
        if (mCount.load() > 0) path = runDryRun(*chassis);
        previewed = request;
        if (mPreviewRequest.load() != request) continue;  // selection changed meanwhile: run again

        PathPreview* preview = mPreview.load();
        if (preview != nullptr) {
            preview->setDynamicPath(path);
            preview->draw();
        }
        mStatus = Status::IDLE;
    }
}

std::vector<Pose> AutonSelector::runDryRun(Chassis& chassis) {
    // Dry-run is scoped to this task: the routine's motions and setPose()
    // calls only drive the simulation, while the real robot's localization
    // and any other task's motions are untouched.
    chassis.setDryRun(true);
    chassis.setPose({0.0f, 0.0f, 0.0f});  // default: origin (simulated)
    forceRunSelected();                    // auton may call setPose to override origin
    chassis.waitUntilSettled();            // finish any async motions it left running
    std::vector<Pose> path = chassis.dryRunPath();
    chassis.setDryRun(false);
    chassis.resetDryRunPath();
    return path;
}

// ============================================================================
// Display task — button callbacks and the refresh timer
// ============================================================================

void AutonSelector::toggleColorCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    switch (self->mAlliance.load()) {
    case Alliance::NONE: self->mAlliance = Alliance::RED;  break;
    case Alliance::RED:  self->mAlliance = Alliance::BLUE; break;
    case Alliance::BLUE: self->mAlliance = Alliance::NONE; break;
    }
    self->selectionChanged();
}

void AutonSelector::toggleTypeCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    const int n = self->mCount.load();
    if (n == 0) return;
    self->mSelectedIndex = (self->mSelectedIndex.load() + 1) % n;
    self->selectionChanged();
}

void AutonSelector::toggleSkillsCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->mSkillsMode = !self->mSkillsMode.load();
    self->selectionChanged();
}

void AutonSelector::recalibrateCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    // Needs the chassis handed over by enableDryRun(). The preview task does
    // it (once the robot is disabled and still), so the screen stays live
    // during the ~2 s IMU calibration.
    if (self->mChassis.load() == nullptr) return;
    self->mRecalRequested = true;
    pros::Task* task = self->mPreviewTask.load();
    if (task != nullptr) task->notify();
}

void AutonSelector::selectionChanged() {
    // Routines may read the alliance or Skills mode (e.g. to mirror the
    // path), so every change re-runs the preview.
    updateButtonLabels();
    mCallbackPending = true;
    requestPreview();
}

void AutonSelector::timerCb(lv_timer_t* timer) {
    static_cast<AutonSelector*>(lv_timer_get_user_data(timer))->refresh();
}

void AutonSelector::refresh() {
    // Keep the preview on this screen, behind the buttons (it may have been
    // created before init(), on another screen).
    PathPreview* preview = mPreview.load();
    lv_obj_t* box = (preview != nullptr) ? preview->mContainer.load() : nullptr;
    if (box != nullptr && lv_obj_get_parent(box) != mScreen) {
        lv_obj_set_parent(box, mScreen);
        lv_obj_move_to_index(box, 0);
    }

    if (mLabelsDirty.exchange(false)) updateButtonLabels();

    if (mCallbackPending.load()) {
        // Never wait on another task here: the display task would stall.
        std::unique_lock<pros::Mutex> lock(mCallbackMutex, std::try_to_lock);
        if (lock.owns_lock()) {
            mCallbackPending = false;
            PathPreviewCallback callback = mPathPreviewCallback;
            lock.unlock();
            if (callback) callback();
        }
    }

    const char* status = statusText();
    if (status != mShownStatus) {
        mShownStatus = status;
        lv_label_set_text_static(mStatusLabel, status);
    }
}

const char* AutonSelector::statusText() const {
    switch (mStatus.load()) {
    case Status::CALIBRATING:  return "Calibrating IMU...";
    case Status::PREVIEWING:   return "Previewing...";
    case Status::WAIT_ENABLED: return "Paused: robot enabled";
    case Status::WAIT_MOTION:  return "Waiting for the robot to stop";
    case Status::IDLE:         break;
    }
    if (!mSkillsMode.load() && mAlliance.load() == Alliance::NONE) {
        return "Pick a color, or auton won't run";
    }
    return "";
}

void AutonSelector::updateButtonLabels() {
    const int n = mCount.load();
    const int index = mSelectedIndex.load();

    // Routine name and routine button
    lv_label_set_text(mNameLabel, n > 0 ? mAutons[index].name.c_str() : "No routines registered");
    char buf[32];
    if (n > 0) {
        std::snprintf(buf, sizeof(buf), "Routine %d/%d", index + 1, n);
    } else {
        std::snprintf(buf, sizeof(buf), "Routine -");
    }
    lv_label_set_text(lv_obj_get_child(mTypeBtn, 0), buf);

    // Color button
    const char* colorStr = "Color: NONE";
    uint32_t btnColor = COLOR_NONE;
    switch (mAlliance.load()) {
    case Alliance::RED:
        colorStr = "Color: RED";
        btnColor = COLOR_RED;
        break;
    case Alliance::BLUE:
        colorStr = "Color: BLUE";
        btnColor = COLOR_BLUE;
        break;
    case Alliance::NONE:
        break;
    }
    lv_label_set_text(lv_obj_get_child(mColorBtn, 0), colorStr);
    lv_obj_set_style_bg_color(mColorBtn, lv_color_hex(btnColor), 0);

    // Skills button
    const bool skills = mSkillsMode.load();
    lv_label_set_text(lv_obj_get_child(mSkillsBtn, 0), skills ? "Skills" : "Match");
    lv_obj_set_style_bg_color(mSkillsBtn, lv_color_hex(skills ? COLOR_SKILLS : COLOR_MATCH), 0);
}

}  // namespace FBLIB
