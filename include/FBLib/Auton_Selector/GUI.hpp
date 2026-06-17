#pragma once

#include <array>
#include <functional>
#include <string>

#include "FBLib/Util/Util.hpp"

// Forward-declare LVGL types (resolved when lvgl.h is included by user's main.h)
struct _lv_obj_t;
typedef _lv_obj_t lv_obj_t;
struct _lv_event_t;
typedef _lv_event_t lv_event_t;

namespace FBLIB {

// Forward declarations (full definitions in Chassis.hpp and Path_Selector.hpp)
class Chassis;
class PathPreview;

// ============================================================================
// AutonSelector — LVGL-based autonomous routine selector for V5 Brain
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

    /// Register an autonomous routine (must be called before init())
    /// Paths are auto-generated via dry-run — no manual waypoints needed.
    void registerAuton(const std::string& name, AutonFunc func);

    /// Build the LVGL screen
    void init();

    /// Refresh UI periodically during disabled state
    void update();

    /// Accessors
    const AutonEntry& getSelected() const;
    int getSelectedIndex() const { return mSelectedIndex; }
    Alliance getAlliance() const { return mAlliance; }
    bool isSkills() const { return mSkillsMode; }
    int count() const { return mCount; }

    /// Run the selected autonomous (or skills if enabled)
    void runSelected();

    /// Run the selected function directly — no alliance/skills gating.
    /// Used for dry-run path generation and testing.
    void forceRunSelected();

    /// Enable automatic dry-run path generation on selection change.
    /// After calling this, no manual onSelectionChanged wiring is needed.
    void enableDryRun(Chassis& chassis, PathPreview& preview);

    /// Callback fired when selection changes (in addition to dry-run, if enabled)
    using PathPreviewCallback = std::function<void()>;
    void onSelectionChanged(PathPreviewCallback callback);

private:
    static void toggleColorCb(lv_event_t* e);
    static void toggleTypeCb(lv_event_t* e);
    static void toggleSkillsCb(lv_event_t* e);
    static void recalibrateCb(lv_event_t* e);

    void toggleColor();
    void toggleType();
    void toggleSkills();
    void updateButtonLabels();

    static constexpr int MAX_AUTONS = 16;
    std::array<AutonEntry, MAX_AUTONS> mAutons;
    int mCount{0};
    int mSelectedIndex{0};
    Alliance mAlliance{Alliance::NONE};
    bool mSkillsMode{false};
    PathPreviewCallback mPathPreviewCallback;

    // Dry-run wiring (set by enableDryRun)
    Chassis* mChassis{nullptr};
    PathPreview* mPreview{nullptr};
    void runDryRun();

    // LVGL object pointers
    lv_obj_t* mScreen{nullptr};
    lv_obj_t* mColorBtn{nullptr};
    lv_obj_t* mTypeBtn{nullptr};
    lv_obj_t* mSkillsBtn{nullptr};
    lv_obj_t* mRecalBtn{nullptr};
};

}  // namespace FBLIB
