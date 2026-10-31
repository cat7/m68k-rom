/*
 * A host camera through AVFoundation (macOS)
 *
 * The capture session delivers frames on its own queue; the newest one is
 * kept, and the device takes it (cropped and scaled) when it needs one.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hostcam.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>

struct HostCam {
    QemuMutex lock;
    uint8_t *frame;             /* BGRA */
    int width, height, stride;
    bool fresh;
    AVCaptureSession *session;
    id delegate;
};

@interface HostCamDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property (nonatomic, assign) HostCam *cam;
@end

@implementation HostCamDelegate
- (void)captureOutput:(AVCaptureOutput *)output
    didOutputSampleBuffer:(CMSampleBufferRef)sample
           fromConnection:(AVCaptureConnection *)connection
{
    HostCam *c = self.cam;
    CVImageBufferRef img = CMSampleBufferGetImageBuffer(sample);
    size_t w, h, stride, y;
    const uint8_t *src;

    if (!img || CVPixelBufferLockBaseAddress(img, kCVPixelBufferLock_ReadOnly)) {
        return;
    }
    w = CVPixelBufferGetWidth(img);
    h = CVPixelBufferGetHeight(img);
    stride = CVPixelBufferGetBytesPerRow(img);
    src = CVPixelBufferGetBaseAddress(img);
    qemu_mutex_lock(&c->lock);
    if (c->width != w || c->height != h) {
        g_free(c->frame);
        c->frame = g_malloc(w * h * 4);
        c->width = w;
        c->height = h;
        c->stride = w * 4;
    }
    for (y = 0; y < h; y++) {
        memcpy(c->frame + y * c->stride, src + y * stride, w * 4);
    }
    c->fresh = true;
    qemu_mutex_unlock(&c->lock);
    CVPixelBufferUnlockBaseAddress(img, kCVPixelBufferLock_ReadOnly);
}
@end

static NSArray<AVCaptureDevice *> *hostcam_devices(void)
{
    NSMutableArray *types = [NSMutableArray array];

    [types addObject:AVCaptureDeviceTypeBuiltInWideAngleCamera];
    if (@available(macOS 14.0, *)) {
        [types addObject:AVCaptureDeviceTypeExternal];
        [types addObject:AVCaptureDeviceTypeContinuityCamera];
    } else {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
        [types addObject:AVCaptureDeviceTypeExternalUnknown];
#pragma clang diagnostic pop
    }
    return [AVCaptureDeviceDiscoverySession
            discoverySessionWithDeviceTypes:types
                                  mediaType:AVMediaTypeVideo
                                   position:AVCaptureDevicePositionUnspecified]
           .devices;
}

static AVCaptureDevice *hostcam_find(const char *name)
{
    if (!strcmp(name, "default")) {
        return [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    }
    for (AVCaptureDevice *d in hostcam_devices()) {
        if ([d.localizedName hasPrefix:[NSString stringWithUTF8String:name]]) {
            return d;
        }
    }
    return nil;
}

/* "a", "b" for an error message */
static char *hostcam_names(void)
{
    GString *names = g_string_new("");

    for (AVCaptureDevice *d in hostcam_devices()) {
        g_string_append_printf(names, "%s\"%s\"", names->len ? ", " : "",
                               d.localizedName.UTF8String);
    }
    return g_string_free(names, false);
}

static void hostcam_start(HostCam *c, AVCaptureDevice *dev)
{
    AVCaptureSession *session = [[AVCaptureSession alloc] init];
    AVCaptureVideoDataOutput *out = [[AVCaptureVideoDataOutput alloc] init];
    HostCamDelegate *delegate = [[HostCamDelegate alloc] init];
    NSError *err = nil;
    AVCaptureDeviceInput *in = [AVCaptureDeviceInput deviceInputWithDevice:dev
                                                                     error:&err];
    dispatch_queue_t queue;

    if (!in) {
        warn_report("host camera: %s",
                    err.localizedDescription.UTF8String ?: "cannot open");
        [session release];
        [out release];
        [delegate release];
        return;
    }
    [session beginConfiguration];
    if ([session canSetSessionPreset:AVCaptureSessionPreset640x480]) {
        session.sessionPreset = AVCaptureSessionPreset640x480;
    }
    if ([session canAddInput:in]) {
        [session addInput:in];
    }
    out.videoSettings = @{
        (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
    };
    out.alwaysDiscardsLateVideoFrames = YES;
    delegate.cam = c;
    queue = dispatch_queue_create("qemu.hostcam", NULL);
    [out setSampleBufferDelegate:delegate queue:queue];
    dispatch_release(queue);
    if ([session canAddOutput:out]) {
        [session addOutput:out];
    }
    [out release];
    [session commitConfiguration];
    [session startRunning];
    c->session = session;
    c->delegate = delegate;
}

HostCam *hostcam_open(const char *name, Error **errp)
{
    HostCam *c;

    @autoreleasepool {
        AVCaptureDevice *dev = hostcam_find(name);

        if (!dev) {
            g_autofree char *names = hostcam_names();

            error_setg(errp, "no host camera '%s' (the cameras: %s)", name,
                       *names ? names : "none");
            return NULL;
        }
        c = g_new0(HostCam, 1);
        qemu_mutex_init(&c->lock);
        switch ([AVCaptureDevice
                 authorizationStatusForMediaType:AVMediaTypeVideo]) {
        case AVAuthorizationStatusAuthorized:
            hostcam_start(c, dev);
            break;
        case AVAuthorizationStatusNotDetermined:
            /* the user is asked; start once allowed */
            [dev retain];
            [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                     completionHandler:^(BOOL granted) {
                @autoreleasepool {
                    if (granted) {
                        hostcam_start(c, dev);
                    } else {
                        warn_report("host camera: access denied");
                    }
                    [dev release];
                }
            }];
            break;
        default:
            warn_report("host camera: access denied (System Settings > "
                        "Privacy & Security > Camera)");
            break;
        }
    }
    return c;
}

bool hostcam_grab(HostCam *c, uint32_t *dst, int w, int h)
{
    int x, y, cw, ch, x0, y0;
    const uint8_t *row, *p;

    qemu_mutex_lock(&c->lock);
    if (!c->frame) {
        qemu_mutex_unlock(&c->lock);
        return false;
    }
    /* the middle 4:3 of the picture */
    cw = c->width;
    ch = c->height;
    if (cw * 3 > ch * 4) {
        cw = ch * 4 / 3;
    } else {
        ch = cw * 3 / 4;
    }
    x0 = (c->width - cw) / 2;
    y0 = (c->height - ch) / 2;
    for (y = 0; y < h; y++) {
        row = c->frame + (y0 + y * ch / h) * c->stride + x0 * 4;
        for (x = 0; x < w; x++) {
            p = row + (x * cw / w) * 4;
            *dst++ = p[2] << 16 | p[1] << 8 | p[0];
        }
    }
    c->fresh = false;
    qemu_mutex_unlock(&c->lock);
    return true;
}

void hostcam_close(HostCam *c)
{
    if (c->session) {
        [c->session stopRunning];
        [c->session release];
    }
    [c->delegate release];
    g_free(c->frame);
    qemu_mutex_destroy(&c->lock);
    g_free(c);
}
