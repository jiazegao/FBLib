#include "FBLib/Auton_Selector/GUI.hpp"
#include "FBLib/Auton_Selector/Path_Selector.hpp"

#include <cstdio>

#include "FBLib/Chassis.hpp"
#include "FBLib/Util/Util.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

// LVGL headers (provided by PROS toolchain)
#include "liblvgl/lvgl.h"

namespace FBLIB {

// ============================================================================
// AutonSelector — LVGL-based autonomous routine selector for V5 Brain
// ============================================================================

AutonSelector::AutonSelector()
    : mAutons(), mCount(0), mSelectedIndex(0),
      mAlliance(Alliance::NONE), mSkillsMode(false) {}

void AutonSelector::registerAuton(const std::string& name, AutonFunc func) {
    if (mCount >= MAX_AUTONS) return;
    mAutons[mCount] = {name, func};
    mCount++;
}

void AutonSelector::init() {
    // Create the LVGL screen
    mScreen = lv_obj_create(nullptr);
    lv_screen_load(mScreen);
    lv_obj_set_style_bg_color(mScreen, lv_color_hex(0x1A1A2E), 0);

    // ================================================================
    // Title label (top of screen)
    // ================================================================
    lv_obj_t* title = lv_label_create(mScreen);
    lv_label_set_text(title, "FBLib — Auton Selector");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // ================================================================
    // Auton name display (center)
    // ================================================================
    lv_obj_t* autonLabel = lv_label_create(mScreen);
    lv_obj_set_style_text_color(autonLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(autonLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(autonLabel, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_user_data(autonLabel, this);

    if (mCount > 0) {
        lv_label_set_text(autonLabel, mAutons[mSelectedIndex].name.c_str());
    } else {
        lv_label_set_text(autonLabel, "No autons registered");
    }

    // ================================================================
    // Alliance/Color toggle button (left side)
    // ================================================================
    mColorBtn = lv_button_create(mScreen);
    lv_obj_set_size(mColorBtn, 140, 50);
    lv_obj_align(mColorBtn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(mColorBtn, lv_color_hex(0x4A4A6A), 0);
    lv_obj_add_event_cb(mColorBtn, toggleColorCb, LV_EVENT_CLICKED, this);

    lv_obj_t* colorLabel = lv_label_create(mColorBtn);
    lv_label_set_text(colorLabel, "Color: NONE");
    lv_obj_center(colorLabel);
    lv_obj_set_style_text_color(colorLabel, lv_color_hex(0xFFFFFF), 0);

    // ================================================================
    // Auton type toggle button (right side)
    // ================================================================
    mTypeBtn = lv_button_create(mScreen);
    lv_obj_set_size(mTypeBtn, 140, 50);
    lv_obj_align(mTypeBtn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(mTypeBtn, lv_color_hex(0x4A4A6A), 0);
    lv_obj_add_event_cb(mTypeBtn, toggleTypeCb, LV_EVENT_CLICKED, this);

    lv_obj_t* typeLabel = lv_label_create(mTypeBtn);
    lv_label_set_text(typeLabel, "Routine: 1");
    lv_obj_center(typeLabel);
    lv_obj_set_style_text_color(typeLabel, lv_color_hex(0xFFFFFF), 0);

    // ================================================================
    // Skills toggle button (center bottom)
    // ================================================================
    mSkillsBtn = lv_button_create(mScreen);
    lv_obj_set_size(mSkillsBtn, 100, 40);
    lv_obj_align(mSkillsBtn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(mSkillsBtn, lv_color_hex(0x3A3A5A), 0);
    lv_obj_add_event_cb(mSkillsBtn, toggleSkillsCb, LV_EVENT_CLICKED, this);

    lv_obj_t* skillsLabel = lv_label_create(mSkillsBtn);
    lv_label_set_text(skillsLabel, "Match");
    lv_obj_center(skillsLabel);
    lv_obj_set_style_text_color(skillsLabel, lv_color_hex(0xFFFFFF), 0);

    // ================================================================
    // Recalibrate button (top right corner)
    // ================================================================
    mRecalBtn = lv_button_create(mScreen);
    lv_obj_set_size(mRecalBtn, 80, 30);
    lv_obj_align(mRecalBtn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(mRecalBtn, lv_color_hex(0x8B0000), 0);
    lv_obj_add_event_cb(mRecalBtn, recalibrateCb, LV_EVENT_CLICKED, this);

    lv_obj_t* recalLabel = lv_label_create(mRecalBtn);
    lv_label_set_text(recalLabel, "Recal");
    lv_obj_center(recalLabel);
    lv_obj_set_style_text_color(recalLabel, lv_color_hex(0xFFFFFF), 0);
}

void AutonSelector::update() {
    // Periodic refresh during disabled state — update labels if state changed
    updateButtonLabels();
}

const AutonEntry& AutonSelector::getSelected() const {
    if (mCount == 0) {
        static AutonEntry empty;
        return empty;
    }
    return mAutons[mSelectedIndex];
}

void AutonSelector::runSelected() {
    if (mSkillsMode) {
        // Skills mode: run selected autonom without alliance constraints
        if (mSelectedIndex < mCount && mAutons[mSelectedIndex].function) {
            mAutons[mSelectedIndex].function();
        }
        return;
    }

    if (mAlliance == Alliance::NONE || mCount == 0) return;
    if (mSelectedIndex < mCount && mAutons[mSelectedIndex].function) {
        mAutons[mSelectedIndex].function();
    }
}

void AutonSelector::forceRunSelected() {
    if (mSelectedIndex < mCount && mAutons[mSelectedIndex].function) {
        mAutons[mSelectedIndex].function();
    }
}

void AutonSelector::onSelectionChanged(PathPreviewCallback callback) {
    mPathPreviewCallback = callback;
    // Immediately fire so the user callback + dry-run are triggered
    if (mPathPreviewCallback) {
        mPathPreviewCallback();
    }
    runDryRun();
}

void AutonSelector::enableDryRun(Chassis& chassis, PathPreview& preview) {
    mChassis = &chassis;
    mPreview = &preview;
    // Run dry-run immediately for the current selection
    runDryRun();
}

void AutonSelector::runDryRun() {
    if (mChassis == nullptr || mPreview == nullptr) return;

    mPreview->clear();
    mChassis->setPose({0.0f, 0.0f, 0.0f});   // default: origin
    mChassis->setDryRun(true);
    forceRunSelected();                        // auton may call setPose to override origin
    mPreview->setDynamicPath(mChassis->dryRunPath());
    mPreview->draw();
    mChassis->setDryRun(false);
    mChassis->resetDryRunPath();
}

// ============================================================================
// Button callbacks
// ============================================================================

void AutonSelector::toggleColorCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->toggleColor();
    if (self->mPathPreviewCallback) {
        self->mPathPreviewCallback();
    }
    self->runDryRun();
}

void AutonSelector::toggleTypeCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->toggleType();
    if (self->mPathPreviewCallback) {
        self->mPathPreviewCallback();
    }
    self->runDryRun();
}

void AutonSelector::toggleSkillsCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->toggleSkills();
}

void AutonSelector::recalibrateCb(lv_event_t* e) {
    // Placeholder: recalibration is handled externally by the user
    // This button signals intent via the callback mechanism
    (void)e;
}

// ============================================================================
// Internal state toggles
// ============================================================================

void AutonSelector::toggleColor() {
    switch (mAlliance) {
    case Alliance::NONE: mAlliance = Alliance::RED;  break;
    case Alliance::RED:  mAlliance = Alliance::BLUE; break;
    case Alliance::BLUE: mAlliance = Alliance::NONE; break;
    }
    updateButtonLabels();
}

void AutonSelector::toggleType() {
    if (mCount == 0) return;
    mSelectedIndex = (mSelectedIndex + 1) % mCount;
    updateButtonLabels();
}

void AutonSelector::toggleSkills() {
    mSkillsMode = !mSkillsMode;
    updateButtonLabels();
}

void AutonSelector::updateButtonLabels() {
    // Update color button
    if (mColorBtn != nullptr) {
        lv_obj_t* label = lv_obj_get_child(mColorBtn, 0);
        if (label != nullptr) {
            const char* colorStr = "Color: NONE";
            lv_color_t btnColor = lv_color_hex(0x4A4A6A);
            switch (mAlliance) {
            case Alliance::RED:
                colorStr = "Color: RED";
                btnColor = lv_color_hex(0x8B0000);
                break;
            case Alliance::BLUE:
                colorStr = "Color: BLUE";
                btnColor = lv_color_hex(0x00008B);
                break;
            case Alliance::NONE:
            default:
                break;
            }
            lv_label_set_text(label, colorStr);
            lv_obj_set_style_bg_color(mColorBtn, btnColor, 0);
        }
    }

    // Update type/routine button
    if (mTypeBtn != nullptr) {
        lv_obj_t* label = lv_obj_get_child(mTypeBtn, 0);
        if (label != nullptr && mCount > 0) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%s",
                          mAutons[mSelectedIndex].name.c_str());
            lv_label_set_text(label, buf);
        }
    }

    // Update skills button
    if (mSkillsBtn != nullptr) {
        lv_obj_t* label = lv_obj_get_child(mSkillsBtn, 0);
        if (label != nullptr) {
            lv_label_set_text(label, mSkillsMode ? "Skills" : "Match");
            lv_obj_set_style_bg_color(mSkillsBtn,
                mSkillsMode ? lv_color_hex(0x006400) : lv_color_hex(0x3A3A5A), 0);
        }
    }

    // Update center auton label
    lv_obj_t* screen = lv_screen_active();
    if (screen != nullptr && mCount > 0) {
        // Find the auton label by iterating children
        for (uint32_t i = 0; i < lv_obj_get_child_count(screen); i++) {
            lv_obj_t* child = lv_obj_get_child(screen, i);
            if (child != nullptr && lv_obj_get_user_data(child) == this) {
                lv_label_set_text(child, mAutons[mSelectedIndex].name.c_str());
                break;
            }
        }
    }
}

}  // namespace FBLIB
