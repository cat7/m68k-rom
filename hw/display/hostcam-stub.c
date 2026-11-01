/*
 * A host camera: not available on this host
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hostcam.h"

HostCam *hostcam_open(const char *name, Error **errp)
{
    error_setg(errp, "host cameras are only supported on macOS and Windows");
    return NULL;
}

bool hostcam_grab(HostCam *c, uint32_t *dst, int w, int h)
{
    return false;
}

void hostcam_close(HostCam *c)
{
}
