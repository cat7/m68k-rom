/*
 * A host camera, as a source of video frames for an emulated video input
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_HOSTCAM_H
#define HW_DISPLAY_HOSTCAM_H

typedef struct HostCam HostCam;

/*
 * Open the host camera @name: "default", or the start of a camera's name.
 * Frames arrive in the background once the user has allowed the access.
 */
HostCam *hostcam_open(const char *name, Error **errp);

/*
 * The newest frame, cropped to 4:3 and scaled to @w x @h, as 0x00RRGGBB
 * pixels. False when no frame has arrived (yet).
 */
bool hostcam_grab(HostCam *c, uint32_t *dst, int w, int h);

void hostcam_close(HostCam *c);

#endif
