/*
 * A host camera through Media Foundation (Windows)
 *
 * A thread reads frames from the camera, converted to RGB32; the newest
 * one is kept, and the device takes it (cropped and scaled) when it needs
 * one.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define COBJMACROS
#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hostcam.h"

#include <initguid.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>

#define WANT_W 640
#define WANT_H 480

struct HostCam {
    QemuMutex lock;
    uint8_t *frame;             /* BGRA */
    int width, height, stride;
    bool fresh;

    char *name;
    QemuThread thread;
    QemuSemaphore started;
    char *error;                /* why the camera did not start */
    bool stop;
};

static char *hostcam_utf8(IMFActivate *a, REFGUID key)
{
    WCHAR *w = NULL;
    UINT32 len;
    char *s;

    if (FAILED(IMFActivate_GetAllocatedString(a, key, &w, &len))) {
        return g_strdup("");
    }
    s = g_utf16_to_utf8((const gunichar2 *)w, -1, NULL, NULL, NULL);
    CoTaskMemFree(w);
    return s ? s : g_strdup("");
}

/* The camera @name ("default" or the start of its name), and all names */
static IMFActivate *hostcam_find(const char *name, char **names)
{
    IMFAttributes *attr = NULL;
    IMFActivate **devs = NULL, *found = NULL;
    UINT32 n = 0, i;
    GString *all = g_string_new("");

    if (SUCCEEDED(MFCreateAttributes(&attr, 1)) &&
        SUCCEEDED(IMFAttributes_SetGUID(attr,
                                        &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                        &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID)) &&
        SUCCEEDED(MFEnumDeviceSources(attr, &devs, &n))) {
        for (i = 0; i < n; i++) {
            g_autofree char *dn =
                hostcam_utf8(devs[i], &MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME);

            g_string_append_printf(all, "%s\"%s\"", all->len ? ", " : "", dn);
            if (!found && (!strcmp(name, "default") ||
                           g_str_has_prefix(dn, name))) {
                found = devs[i];
                IMFActivate_AddRef(found);
            }
            IMFActivate_Release(devs[i]);
        }
        CoTaskMemFree(devs);
    }
    if (attr) {
        IMFAttributes_Release(attr);
    }
    *names = g_string_free(all, false);
    return found;
}

static void hostcam_frame_size(IMFMediaType *t, UINT32 *w, UINT32 *h)
{
    UINT64 size = 0;

    IMFMediaType_GetUINT64(t, &MF_MT_FRAME_SIZE, &size);
    *w = size >> 32;
    *h = (UINT32)size;
}

/* The camera's own format nearest to 640x480, then RGB32 from it */
static HRESULT hostcam_set_format(IMFSourceReader *r)
{
    const DWORD stream = MF_SOURCE_READER_FIRST_VIDEO_STREAM;
    IMFMediaType *t, *best = NULL, *rgb = NULL;
    UINT32 w, h, bw = 0, bh = 0;
    DWORD i;
    HRESULT hr;

    for (i = 0; SUCCEEDED(IMFSourceReader_GetNativeMediaType(r, stream, i, &t));
         i++) {
        hostcam_frame_size(t, &w, &h);
        if (w && h &&
            (!best || abs((int)(w * h) - WANT_W * WANT_H) <
                      abs((int)(bw * bh) - WANT_W * WANT_H))) {
            if (best) {
                IMFMediaType_Release(best);
            }
            best = t;
            bw = w;
            bh = h;
        } else {
            IMFMediaType_Release(t);
        }
    }
    if (best) {
        IMFSourceReader_SetCurrentMediaType(r, stream, NULL, best);
        IMFMediaType_Release(best);
    }
    hr = MFCreateMediaType(&rgb);
    if (FAILED(hr)) {
        return hr;
    }
    IMFMediaType_SetGUID(rgb, &MF_MT_MAJOR_TYPE, &MFMediaType_Video);
    IMFMediaType_SetGUID(rgb, &MF_MT_SUBTYPE, &MFVideoFormat_RGB32);
    if (bw && bh) {
        IMFMediaType_SetUINT64(rgb, &MF_MT_FRAME_SIZE, (UINT64)bw << 32 | bh);
    }
    hr = IMFSourceReader_SetCurrentMediaType(r, stream, NULL, rgb);
    IMFMediaType_Release(rgb);
    return hr;
}

static void hostcam_keep(HostCam *c, IMFSample *sample, UINT32 w, UINT32 h,
                         LONG stride)
{
    IMFMediaBuffer *buf = NULL;
    BYTE *data;
    DWORD len;
    const uint8_t *src;
    UINT32 y;

    if (FAILED(IMFSample_ConvertToContiguousBuffer(sample, &buf))) {
        return;
    }
    if (SUCCEEDED(IMFMediaBuffer_Lock(buf, &data, NULL, &len))) {
        if (len >= (DWORD)abs(stride) * h) {
            /* a negative stride is a bottom-up picture */
            src = stride < 0 ? data + (h - 1) * -stride : data;
            qemu_mutex_lock(&c->lock);
            if (c->width != w || c->height != h) {
                g_free(c->frame);
                c->frame = g_malloc(w * h * 4);
                c->width = w;
                c->height = h;
                c->stride = w * 4;
            }
            for (y = 0; y < h; y++) {
                memcpy(c->frame + y * c->stride, src + (ptrdiff_t)y * stride,
                       w * 4);
            }
            c->fresh = true;
            qemu_mutex_unlock(&c->lock);
        }
        IMFMediaBuffer_Unlock(buf);
    }
    IMFMediaBuffer_Release(buf);
}

static void *hostcam_thread(void *opaque)
{
    HostCam *c = opaque;
    IMFActivate *dev;
    IMFMediaSource *src = NULL;
    IMFAttributes *attr = NULL;
    IMFSourceReader *reader = NULL;
    IMFMediaType *cur = NULL;
    g_autofree char *names = NULL;
    UINT32 w = 0, h = 0, ustride = 0;
    LONG stride;
    HRESULT hr;

    CoInitializeEx(NULL, COINIT_MULTITHREADED);
    hr = MFStartup(MF_VERSION, MFSTARTUP_LITE);
    if (FAILED(hr)) {
        c->error = g_strdup_printf("Media Foundation: error 0x%lx", hr);
        goto out;
    }
    dev = hostcam_find(c->name, &names);
    if (!dev) {
        c->error = g_strdup_printf("no host camera '%s' (the cameras: %s)",
                                   c->name, *names ? names : "none");
        goto shutdown;
    }
    hr = IMFActivate_ActivateObject(dev, &IID_IMFMediaSource, (void **)&src);
    IMFActivate_Release(dev);
    if (SUCCEEDED(hr)) {
        hr = MFCreateAttributes(&attr, 1);
    }
    if (SUCCEEDED(hr)) {
        IMFAttributes_SetUINT32(attr, &MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING,
                                TRUE);
        hr = MFCreateSourceReaderFromMediaSource(src, attr, &reader);
    }
    if (SUCCEEDED(hr)) {
        hr = hostcam_set_format(reader);
    }
    if (SUCCEEDED(hr)) {
        hr = IMFSourceReader_GetCurrentMediaType(
            reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, &cur);
    }
    if (FAILED(hr)) {
        /* E_ACCESSDENIED: Settings > Privacy & security > Camera */
        c->error = g_strdup_printf("cannot open the host camera (error 0x%lx)%s",
                                   hr, hr == E_ACCESSDENIED ?
                                   ": access denied (Settings > Privacy & "
                                   "security > Camera)" : "");
        goto release;
    }
    hostcam_frame_size(cur, &w, &h);
    stride = w * 4;
    if (SUCCEEDED(IMFMediaType_GetUINT32(cur, &MF_MT_DEFAULT_STRIDE,
                                         &ustride))) {
        stride = (LONG)ustride;
    }
    IMFMediaType_Release(cur);
    qemu_sem_post(&c->started);

    while (!qatomic_read(&c->stop)) {
        IMFSample *sample = NULL;
        DWORD flags = 0;
        LONGLONG ts;

        hr = IMFSourceReader_ReadSample(reader,
                                        MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0,
                                        NULL, &flags, &ts, &sample);
        if (FAILED(hr) || (flags & (MF_SOURCE_READERF_ERROR |
                                    MF_SOURCE_READERF_ENDOFSTREAM))) {
            if (!qatomic_read(&c->stop)) {
                warn_report("host camera: stopped (error 0x%lx)", hr);
            }
            if (sample) {
                IMFSample_Release(sample);
            }
            break;
        }
        if (sample) {
            hostcam_keep(c, sample, w, h, stride);
            IMFSample_Release(sample);
        }
    }

release:
    if (reader) {
        IMFSourceReader_Release(reader);
    }
    if (attr) {
        IMFAttributes_Release(attr);
    }
    if (src) {
        IMFMediaSource_Shutdown(src);
        IMFMediaSource_Release(src);
    }
shutdown:
    MFShutdown();
out:
    CoUninitialize();
    if (c->error) {
        qemu_sem_post(&c->started);
    }
    return NULL;
}

HostCam *hostcam_open(const char *name, Error **errp)
{
    HostCam *c = g_new0(HostCam, 1);

    qemu_mutex_init(&c->lock);
    qemu_sem_init(&c->started, 0);
    c->name = g_strdup(name);
    qemu_thread_create(&c->thread, "hostcam", hostcam_thread, c,
                       QEMU_THREAD_JOINABLE);
    qemu_sem_wait(&c->started);
    if (c->error) {
        error_setg(errp, "%s", c->error);
        qemu_thread_join(&c->thread);
        g_free(c->error);
        g_free(c->name);
        qemu_sem_destroy(&c->started);
        qemu_mutex_destroy(&c->lock);
        g_free(c);
        return NULL;
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
    qatomic_set(&c->stop, true);
    qemu_thread_join(&c->thread);
    g_free(c->frame);
    g_free(c->name);
    qemu_sem_destroy(&c->started);
    qemu_mutex_destroy(&c->lock);
    g_free(c);
}
