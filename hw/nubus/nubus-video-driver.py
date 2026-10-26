#!/usr/bin/env python3
#
# Turn the assembled nubus-video-driver.s (or nubus-video-init.s, with
# the array name nubus_video_init) into a C header
#
# SPDX-License-Identifier: GPL-2.0-or-later

import sys

data = open(sys.argv[1], 'rb').read()
name = sys.argv[2] if len(sys.argv) > 2 else 'nubus_video_driver'
what = {'nubus_video_driver': 'Video driver',
        'nubus_video_init': 'PrimaryInit code'}[name]
src = name.replace('_', '-') + '.s'
if name == 'nubus_video_driver':
    guard = 'HW_NUBUS_NUBUS_VIDEO_DRIVER_H'
else:
    guard = 'HW_NUBUS_NUBUS_VIDEO_INIT_H'

print(f'''/*
 * {what} for the QEMU NuBus video card, assembled from
 * {src}.  Generated file, do not edit: see the
 * instructions at the top of {src} to rebuild it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef {guard}
#define {guard}

static const uint8_t {name}[] = {{''')
for i in range(0, len(data), 12):
    print('    ' + ' '.join('0x%02x,' % b for b in data[i:i + 12]))
print('''};

#endif''')
