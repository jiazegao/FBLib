#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include "FBLib/Util/Util.hpp"
#include "pros/rtos.hpp"

// Forward-declare LVGL types (resolved when lvgl.h is included by user's main.h)
struct _lv_obj_t;
typedef _lv_obj_t lv_obj_t;
struct _lv_event_t;
typedef _lv_event_t lv_event_t;
struct _lv_timer_t;
typedef _lv_timer_t lv_timer_t;

namespace FBLIB {

// Forward declarations (full definitions in Chassis.hpp and Path_Selector.hpp)
class Chassis;
class PathPreview;

// ============================================================================
// AutonSelector — LVGL-based autonomous routine selector for V5 Brain
// ============================================================================
//
// Screen (480x240): the PathPreview field fills the middle 240x240 square.
// Left of it: routine name, routine button and a status line. Right of it:
// alliance color, Match/Skills and Recal buttons.
//
// Threading: PROS runs LVGL in its own display task without a lock, and an
// LVGL call from another task while that task is drawing can hang the
// program. init() builds the screen (call it from initialize()); after that
// the screen is only touched from the display task, and dry-run previews and
// Recal run in a separate preview task, so the screen stays responsive.
// Every other public method is safe to call from any task.
// The object must outlive the screen: make it a global.
// ============================================================================

enum class Alliance { RED, BLUE, NONE };

using AutonFunc = std::function<void()>;

struct AutonEntry {
    std::string name;
    AutonFunc function;
};

class AutonSelector {
public:
    AutonSelector();

    /// Register an autonomous routine (call before init(); up to 16)
    /// Paths are auto-generated via dry-run — no manual waypoints needed.
    void registerAuton(const std::string& name, AutonFunc func);

    /// Build the selector screen and show it
    void init();

    /// Not needed any more (the screen refreshes itself); kept for old code.
    void update();

    /// Accessors
    const AutonEntry& getSelected() const;
    int getSelectedIndex() const { return mSelectedIndex.load(); }
    Alliance getAlliance() const { return mAlliance.load(); }
    bool isSkills() const { return mSkillsMode.load(); }
    int count() const { return mCount.load(); }

    /// Run the selected autonomous. In Match mode nothing runs until an
    /// alliance color is chosen; Skills mode needs no color.
    void runSelected();

    /// Run the selected function directly — no alliance/skills gating.
    /// Used for dry-run path generation and testing.
    void forceRunSelected();

    /// Preview the selected routine on `preview` whenever the selection
    /// changes (a dry run in the preview task), starting now, and enable the
    /// Recal button. Previews wait while the robot is enabled under
    /// competition control and while a real motion is running.
    void enableDryRun(Chassis& chassis, PathPreview& preview);

    /// Callback fired on the display task after every selection change, and
    /// once after it is set.
    using PathPreviewCallback = std::function<void()>;
    void onSelectionChanged(PathPreviewCallback callback);

private:
    enum class Status { IDLE, PREVIEWING, WAIT_ENABLED, WAIT_MOTION, CALIBRATING };

    static void toggleColorCb(lv_event_t* e);
    static void toggleTypeCb(lv_event_t* e);
    static void toggleSkillsCb(lv_event_t* e);
    static void recalibrateCb(lv_event_t* e);
    static void timerCb(lv_timer_t* timer);

    // Display task
    void selectionChanged();
    void refresh();
    void updateButtonLabels();
    const char* statusText() const;

    // Preview task
    void requestPreview();
    void previewLoop();
    std::vector<Pose> runDryRun(Chassis& chassis);

    static constexpr int MAX_AUTONS = 16;
    std::array<AutonEntry, MAX_AUTONS> mAutons;
    std::atomic<int> mCount{0};
    std::atomic<int> mSelectedIndex{0};
    std::atomic<Alliance> mAlliance{Alliance::NONE};
    std::atomic<bool> mSkillsMode{false};

    pros::Mutex mCallbackMutex;
    PathPreviewCallback mPathPreviewCallback;
    std::atomic<bool> mCallbackPending{false};
    std::atomic<bool> mLabelsDirty{false};

    // Dry-run wiring (set by enableDryRun)
    std::atomic<Chassis*> mChassis{nullptr};
    std::atomic<PathPreview*> mPreview{nullptr};
    std::atomic<pros::Task*> mPreviewTask{nullptr};
    std::atomic<uint32_t> mPreviewRequest{0};  // bumped on every selection change
    std::atomic<bool> mRecalRequested{false};
    std::atomic<Status> mStatus{Status::IDLE};

    // LVGL objects (display task only, after init())
    lv_obj_t* mScreen{nullptr};
    lv_obj_t* mNameLabel{nullptr};
    lv_obj_t* mStatusLabel{nullptr};
    lv_obj_t* mColorBtn{nullptr};
    lv_obj_t* mTypeBtn{nullptr};
    lv_obj_t* mSkillsBtn{nullptr};
    lv_obj_t* mRecalBtn{nullptr};
    const char* mShownStatus{nullptr};
};

}  // namespace FBLIB
