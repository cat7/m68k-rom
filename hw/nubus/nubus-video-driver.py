#!/usr/bin/env python3
#
# Turn the assembled nubus-video-driver.s into nubus-video-driver.h
#
# SPDX-License-Identifier: GPL-2.0-or-later

import sys

data = open(sys.argv[1], 'rb').read()

print('''/*
 * Video driver for the QEMU NuBus video card, assembled from
 * nubus-video-driver.s.  Generated file, do not edit: see the
 * instructions at the top of nubus-video-driver.s to rebuild it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NUBUS_NUBUS_VIDEO_DRIVER_H
#define HW_NUBUS_NUBUS_VIDEO_DRIVER_H

static const uint8_t nubus_video_driver[] = {''')
for i in range(0, len(data), 12):
    print('    ' + ' '.join('0x%02x,' % b for b in data[i:i + 12]))
print('''};

#endif''')
