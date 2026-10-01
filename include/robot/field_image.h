#pragma once

// ============================================================================
// Field image for the path preview
// ============================================================================
//
// Top-down image of the Override (2026-27) field, 240 x 240 px: 144" across,
// field centre in the middle, +X right and +Y up, the frame PathPreview draws
// in. Pass it to PathPreview::init(). The data is in src/robot/field_image.c.
// ============================================================================

#include "liblvgl/lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_image_dsc_t robot_field_image;

#ifdef __cplusplus
}
#endif
