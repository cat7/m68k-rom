/*
 * Virtio printer: pages rendered by the guest, printed on the host
 *
 * The guest sends each page as rows of 32-bit pixels. They are compressed
 * straight into a PDF, one image per page, which at the end of the job goes
 * to a host printer or is left in a folder. CUPS prints the PDF; on Windows
 * the page images are drawn on the printer through GDI.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/iov.h"
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "hw/virtio/virtio.h"
#include "hw/core/qdev-properties.h"
#include "hw/virtio/virtio-print.h"
#include <zlib.h>
#ifdef CONFIG_CUPS
#include <cups/cups.h>
#define HOST_PRINTING
#elif defined(_WIN32)
#include <winspool.h>
#define HOST_PRINTING
#endif

#define MAX_PIXELS 20000
#define MAX_MEDIA 32
#define INFO_TTL_NS (10 * NANOSECONDS_PER_SECOND)

/* What the guest is told about the printer, filled in by a thread */
typedef struct PrintInfo {
    QemuMutex lock;
    QemuCond cond;
    QemuThread thread;
    bool valid, busy, quit, want;
    int64_t when;
    char name[64];
    char message[128];
    uint32_t state, color, def;
    uint32_t n_media;
    VPrintMedia media[MAX_MEDIA];
} PrintInfo;

/* Where a page's image is in the PDF */
typedef struct PageImage {
    long start, length;
    uint32_t w, h;              /* pixels */
    uint32_t media_w, media_h;  /* 1/100 pt */
} PageImage;

static PrintInfo *info_of(VirtIOPrint *s)
{
    return (PrintInfo *)s->info;
}

/* Mac Roman, NUL-terminated, at most @size bytes with the NUL */
static void to_macroman(char *dst, size_t size, const char *utf8)
{
    g_autofree char *s = g_convert_with_fallback(utf8, -1, "MACINTOSH",
                                                 "UTF-8", "?", NULL, NULL,
                                                 NULL);
    const char *src = s ? s : utf8;
    size_t i;

    for (i = 0; i + 1 < size && src[i]; i++) {
        dst[i] = s ? src[i] : (src[i] & 0x80 ? '?' : src[i]);
    }
    dst[i] = 0;
}

static char *from_macroman(const char *mr)
{
    char *s = g_convert(mr, -1, "UTF-8", "MACINTOSH", NULL, NULL, NULL);
    return s ? s : g_strdup(mr);
}

#ifdef HOST_PRINTING
/* 1/100 mm to 1/100 pt */
static uint32_t hmm_to_hpt(int hmm)
{
    return hmm > 0 ? (uint32_t)((int64_t)hmm * 7200 / 2540) : 0;
}
#endif

static void set_media(VPrintMedia *m, const char *name, uint32_t w, uint32_t h,
                      uint32_t margin)
{
    to_macroman(m->name, sizeof(m->name), name);
    m->size[0] = w;
    m->size[1] = h;
    m->size[2] = m->size[3] = m->size[4] = m->size[5] = margin;
}

/* Without a host printer: the common sizes, with quarter-inch margins */
static void default_media(PrintInfo *pi)
{
    pi->n_media = 0;
    set_media(&pi->media[pi->n_media++], "US Letter", 61200, 79200, 1800);
    set_media(&pi->media[pi->n_media++], "US Legal", 61200, 100800, 1800);
    set_media(&pi->media[pi->n_media++], "A4", 59528, 84189, 1800);
    set_media(&pi->media[pi->n_media++], "A5", 41953, 59528, 1800);
    set_media(&pi->media[pi->n_media++], "B5", 49890, 70866, 1800);
    pi->def = 0;
}

#ifdef CONFIG_CUPS
/* "na_letter_8.5x11in" and the like, for people */
static char *media_name(const char *pwg)
{
    static const char *const known[][2] = {
        {"na_letter_", "US Letter"}, {"na_legal_", "US Legal"},
        {"na_executive_", "Executive"}, {"na_ledger_", "Tabloid"},
        {"na_invoice_", "Statement"}, {"na_govt-letter_", "8 x 10 in"},
        {"na_index-3x5_", "Index card 3 x 5"},
        {"na_index-4x6_", "Index card 4 x 6"},
        {"na_index-5x8_", "Index card 5 x 8"},
        {"na_number-10_", "Envelope #10"}, {"na_monarch_", "Envelope Monarch"},
        {"iso_a3_", "A3"}, {"iso_a4_", "A4"}, {"iso_a5_", "A5"},
        {"iso_a6_", "A6"}, {"iso_b5_", "B5"}, {"jis_b4_", "B4 (JIS)"},
        {"jis_b5_", "B5 (JIS)"}, {"iso_dl_", "Envelope DL"},
        {"iso_c5_", "Envelope C5"}, {"iso_c6_", "Envelope C6"},
        {"om_folio_", "Folio"}, {"oe_photo-l_", "Photo L"},
    };
    const char *first = strchr(pwg, '_'), *last = strrchr(pwg, '_');
    g_autofree char *kind = NULL;
    g_autofree char *dims = NULL;
    char *x;
    size_t i;

    for (i = 0; i < G_N_ELEMENTS(known); i++) {
        if (g_str_has_prefix(pwg, known[i][0])) {
            return g_strdup(known[i][1]);
        }
    }
    if (!first || first == last) {
        return g_strdup(pwg);
    }
    /* the kind of paper, then its size: "Photo 4x6 (4 x 6 in)" */
    kind = g_strndup(first + 1, last - first - 1);
    for (x = kind; *x; x++) {
        if (*x == '-') {
            *x = ' ';
        }
    }
    kind[0] = g_ascii_toupper(kind[0]);
    dims = g_strdup(last + 1);
    x = strchr(dims, 'x');
    if (x) {
        char *unit = x + 1;

        while (*unit && (g_ascii_isdigit(*unit) || *unit == '.')) {
            unit++;
        }
        *x = 0;
        return g_strdup_printf("%s (%s x %.*s %s)", kind, dims,
                               (int)(unit - x - 1), x + 1, unit);
    }
    return g_strdup(kind);
}

static void cups_info(PrintInfo *pi, const char *queue)
{
    cups_dest_t *dest = cupsGetNamedDest(CUPS_HTTP_DEFAULT,
                                         queue && *queue ? queue : NULL,
                                         NULL);
    cups_dinfo_t *dinfo;
    cups_size_t size, def;
    const char *v, *reasons;
    int i, n;

    if (!dest) {
        to_macroman(pi->name, sizeof(pi->name),
                    queue && *queue ? queue : "No host printer");
        to_macroman(pi->message, sizeof(pi->message),
                    "The host has no such printer");
        pi->state = VPRINT_STOPPED;
        pi->color = 1;
        default_media(pi);
        return;
    }

    v = cupsGetOption("printer-info", dest->num_options, dest->options);
    to_macroman(pi->name, sizeof(pi->name), v && *v ? v : dest->name);

    v = cupsGetOption("printer-state", dest->num_options, dest->options);
    pi->state = v ? atoi(v) : VPRINT_IDLE;
    reasons = cupsGetOption("printer-state-reasons", dest->num_options,
                            dest->options);
    v = cupsGetOption("printer-state-message", dest->num_options,
                      dest->options);
    if (v && *v) {
        to_macroman(pi->message, sizeof(pi->message), v);
    } else if (reasons && *reasons && strcmp(reasons, "none")) {
        to_macroman(pi->message, sizeof(pi->message), reasons);
    } else {
        pi->message[0] = 0;
    }

    pi->n_media = 0;
    pi->def = 0;
    pi->color = 1;
    dinfo = cupsCopyDestInfo(CUPS_HTTP_DEFAULT, dest);
    if (dinfo) {
        pi->color = cupsCheckDestSupported(CUPS_HTTP_DEFAULT, dest, dinfo,
                                           "print-color-mode", "color");
        memset(&def, 0, sizeof(def));
        cupsGetDestMediaDefault(CUPS_HTTP_DEFAULT, dest, dinfo,
                                CUPS_MEDIA_FLAGS_DEFAULT, &def);
        n = cupsGetDestMediaCount(CUPS_HTTP_DEFAULT, dest, dinfo,
                                  CUPS_MEDIA_FLAGS_DEFAULT);
        for (i = 0; i < n && pi->n_media < MAX_MEDIA; i++) {
            VPrintMedia *m = &pi->media[pi->n_media];
            const char *ln;
            uint32_t j;
            bool dup = false;

            if (!cupsGetDestMediaByIndex(CUPS_HTTP_DEFAULT, dest, dinfo, i,
                                         CUPS_MEDIA_FLAGS_DEFAULT, &size) ||
                size.width <= 0 || size.length <= 0) {
                continue;
            }
            /* one entry per paper size: the borderless variants go */
            for (j = 0; j < pi->n_media; j++) {
                if (pi->media[j].size[0] == hmm_to_hpt(size.width) &&
                    pi->media[j].size[1] == hmm_to_hpt(size.length)) {
                    dup = true;
                    break;
                }
            }
            if (dup) {
                continue;
            }
            ln = cupsLocalizeDestMedia(CUPS_HTTP_DEFAULT, dest, dinfo,
                                       CUPS_MEDIA_FLAGS_DEFAULT, &size);
            if (ln && *ln && strcmp(ln, size.media)) {
                to_macroman(m->name, sizeof(m->name), ln);
            } else {
                g_autofree char *nice = media_name(size.media);

                to_macroman(m->name, sizeof(m->name), nice);
            }
            m->size[0] = hmm_to_hpt(size.width);
            m->size[1] = hmm_to_hpt(size.length);
            m->size[2] = hmm_to_hpt(size.left);
            m->size[3] = hmm_to_hpt(size.top);
            m->size[4] = hmm_to_hpt(size.right);
            m->size[5] = hmm_to_hpt(size.bottom);
            if (!strcmp(size.media, def.media)) {
                pi->def = pi->n_media;
            }
            pi->n_media++;
        }
        cupsFreeDestInfo(dinfo);
    }
    if (!pi->n_media) {
        default_media(pi);
    }
    cupsFreeDests(1, dest);
}
#endif

#ifdef _WIN32
/* 1/10 mm to 1/100 pt */
static uint32_t tmm_to_hpt(LONG tmm)
{
    return hmm_to_hpt(tmm * 10);
}

static char *from_wide(const wchar_t *w)
{
    char *s = g_utf16_to_utf8((const gunichar2 *)w, -1, NULL, NULL, NULL);
    return s ? s : g_strdup("?");
}

/* The queue's name, or the default printer's; NULL when there is none */
static wchar_t *win_printer(const char *queue)
{
    DWORD n = 0;
    wchar_t *w;

    if (queue && *queue) {
        return (wchar_t *)g_utf8_to_utf16(queue, -1, NULL, NULL, NULL);
    }
    GetDefaultPrinterW(NULL, &n);
    if (!n) {
        return NULL;
    }
    w = g_new0(wchar_t, n);
    if (!GetDefaultPrinterW(w, &n)) {
        g_free(w);
        return NULL;
    }
    return w;
}

/* The printer's own settings, which a job starts from */
static DEVMODEW *win_devmode(HANDLE h, wchar_t *name)
{
    LONG n = DocumentPropertiesW(NULL, h, name, NULL, NULL, 0);
    DEVMODEW *dm;

    if (n <= 0) {
        return NULL;
    }
    dm = g_malloc0(n);
    if (DocumentPropertiesW(NULL, h, name, dm, NULL, DM_OUT_BUFFER) != IDOK) {
        g_free(dm);
        return NULL;
    }
    return dm;
}

/* What the printer cannot reach, from a DC on its default paper */
static void win_margins(wchar_t *name, uint32_t margin[4])
{
    HDC dc = CreateDCW(L"WINSPOOL", name, NULL, NULL);
    int dx, dy, ox, oy;

    margin[0] = margin[1] = margin[2] = margin[3] = 1800;
    if (!dc) {
        return;
    }
    dx = GetDeviceCaps(dc, LOGPIXELSX);
    dy = GetDeviceCaps(dc, LOGPIXELSY);
    ox = GetDeviceCaps(dc, PHYSICALOFFSETX);
    oy = GetDeviceCaps(dc, PHYSICALOFFSETY);
    if (dx > 0 && dy > 0) {
        margin[0] = (uint32_t)((int64_t)ox * 7200 / dx);
        margin[1] = (uint32_t)((int64_t)oy * 7200 / dy);
        margin[2] = (uint32_t)((int64_t)MAX(0, GetDeviceCaps(dc, PHYSICALWIDTH) -
                                ox - GetDeviceCaps(dc, HORZRES)) * 7200 / dx);
        margin[3] = (uint32_t)((int64_t)MAX(0, GetDeviceCaps(dc, PHYSICALHEIGHT) -
                                oy - GetDeviceCaps(dc, VERTRES)) * 7200 / dy);
    }
    DeleteDC(dc);
}

static void win_info(PrintInfo *pi, const char *queue)
{
    g_autofree wchar_t *wname = win_printer(queue);
    g_autofree PRINTER_INFO_2W *p2 = NULL;
    g_autofree DEVMODEW *dm = NULL;
    g_autofree char *name = NULL;
    HANDLE h = NULL;
    DWORD need = 0;
    uint32_t margin[4];
    int i, n;

    if (!wname || !OpenPrinterW(wname, &h, NULL)) {
        to_macroman(pi->name, sizeof(pi->name),
                    queue && *queue ? queue : "No host printer");
        to_macroman(pi->message, sizeof(pi->message),
                    "The host has no such printer");
        pi->state = VPRINT_STOPPED;
        pi->color = 1;
        default_media(pi);
        return;
    }
    name = from_wide(wname);
    to_macroman(pi->name, sizeof(pi->name), name);
    pi->state = VPRINT_IDLE;
    pi->message[0] = 0;

    GetPrinterW(h, 2, NULL, 0, &need);
    if (need) {
        p2 = g_malloc0(need);
        if (GetPrinterW(h, 2, (LPBYTE)p2, need, &need)) {
            DWORD st = p2->Status;
            const char *why = NULL;

            if (st & PRINTER_STATUS_PAUSED) {
                why = "The printer is paused";
            } else if (st & PRINTER_STATUS_OFFLINE) {
                why = "The printer is offline";
            } else if (st & PRINTER_STATUS_PAPER_OUT) {
                why = "The printer is out of paper";
            } else if (st & PRINTER_STATUS_PAPER_JAM) {
                why = "The paper is jammed";
            } else if (st & (PRINTER_STATUS_ERROR | PRINTER_STATUS_NOT_AVAILABLE |
                             PRINTER_STATUS_USER_INTERVENTION)) {
                why = "The printer needs attention";
            }
            if (why) {
                pi->state = VPRINT_STOPPED;
                to_macroman(pi->message, sizeof(pi->message), why);
            } else if (p2->cJobs || (st & PRINTER_STATUS_PRINTING)) {
                pi->state = VPRINT_PROCESSING;
            }
        }
    }
    pi->color = DeviceCapabilitiesW(wname, NULL, DC_COLORDEVICE, NULL,
                                    NULL) != 0;

    pi->n_media = 0;
    pi->def = 0;
    dm = win_devmode(h, wname);
    win_margins(wname, margin);
    n = DeviceCapabilitiesW(wname, NULL, DC_PAPERS, NULL, NULL);
    if (n > 0 && n < 1000) {
        g_autofree WORD *ids = g_new0(WORD, n);
        g_autofree POINT *sz = g_new0(POINT, n);
        g_autofree wchar_t *names = g_new0(wchar_t, (size_t)n * 64 + 1);

        if (DeviceCapabilitiesW(wname, NULL, DC_PAPERS, (LPWSTR)ids,
                                NULL) == n &&
            DeviceCapabilitiesW(wname, NULL, DC_PAPERSIZE, (LPWSTR)sz,
                                NULL) == n &&
            DeviceCapabilitiesW(wname, NULL, DC_PAPERNAMES, names,
                                NULL) == n) {
            for (i = 0; i < n && pi->n_media < MAX_MEDIA; i++) {
                VPrintMedia *m = &pi->media[pi->n_media];
                wchar_t nm[65];
                g_autofree char *u = NULL;
                uint32_t j;
                bool dup = false;

                if (sz[i].x <= 0 || sz[i].y <= 0) {
                    continue;
                }
                for (j = 0; j < pi->n_media; j++) {
                    if (pi->media[j].size[0] == tmm_to_hpt(sz[i].x) &&
                        pi->media[j].size[1] == tmm_to_hpt(sz[i].y)) {
                        dup = true;
                        break;
                    }
                }
                if (dup) {
                    continue;
                }
                memcpy(nm, names + (size_t)i * 64, 64 * sizeof(wchar_t));
                nm[64] = 0;
                u = from_wide(nm);
                to_macroman(m->name, sizeof(m->name), u);
                m->size[0] = tmm_to_hpt(sz[i].x);
                m->size[1] = tmm_to_hpt(sz[i].y);
                memcpy(&m->size[2], margin, sizeof(margin));
                if (dm && (dm->dmFields & DM_PAPERSIZE) &&
                    dm->dmPaperSize == ids[i]) {
                    pi->def = pi->n_media;
                }
                pi->n_media++;
            }
        }
    }
    if (!pi->n_media) {
        default_media(pi);
    }
    ClosePrinter(h);
}
#endif

static void *info_thread(void *opaque)
{
    VirtIOPrint *s = opaque;
    PrintInfo *pi = info_of(s);
    PrintInfo fresh;

    qemu_mutex_lock(&pi->lock);
    while (!pi->quit) {
        if (!pi->want) {
            qemu_cond_wait(&pi->cond, &pi->lock);
            continue;
        }
        pi->want = false;
        pi->busy = true;
        qemu_mutex_unlock(&pi->lock);

        memset(&fresh, 0, sizeof(fresh));
        if (s->save_to && *s->save_to) {
            g_autofree char *dir = g_path_get_basename(s->save_to);
            g_autofree char *n = g_strdup_printf("Save as PDF in %s", dir);

            to_macroman(fresh.name, sizeof(fresh.name), n);
            fresh.state = VPRINT_IDLE;
            fresh.color = 1;
            default_media(&fresh);
        } else {
#ifdef CONFIG_CUPS
            cups_info(&fresh, s->printer);
#elif defined(_WIN32)
            win_info(&fresh, s->printer);
#else
            to_macroman(fresh.name, sizeof(fresh.name), "No host printing");
            to_macroman(fresh.message, sizeof(fresh.message),
                        "Printing needs save-to= on this host");
            fresh.state = VPRINT_STOPPED;
            fresh.color = 1;
            default_media(&fresh);
#endif
        }

        qemu_mutex_lock(&pi->lock);
        memcpy(pi->name, fresh.name, sizeof(pi->name));
        memcpy(pi->message, fresh.message, sizeof(pi->message));
        pi->state = fresh.state;
        pi->color = fresh.color;
        pi->def = fresh.def;
        pi->n_media = fresh.n_media;
        memcpy(pi->media, fresh.media, sizeof(pi->media));
        pi->valid = true;
        pi->busy = false;
        pi->when = get_clock();
        qemu_cond_broadcast(&pi->cond);
    }
    qemu_mutex_unlock(&pi->lock);
    return NULL;
}

/* Copies of the latest info; the first call waits for it a little */
static void get_info(VirtIOPrint *s, PrintInfo *out)
{
    PrintInfo *pi = info_of(s);
    int64_t deadline = get_clock() + 3 * NANOSECONDS_PER_SECOND;

    qemu_mutex_lock(&pi->lock);
    if (!pi->busy && (!pi->valid || get_clock() - pi->when > INFO_TTL_NS)) {
        pi->want = true;
        qemu_cond_broadcast(&pi->cond);
    }
    while (!pi->valid && get_clock() < deadline) {
        qemu_cond_timedwait(&pi->cond, &pi->lock, 200);
    }
    if (pi->valid) {
        memcpy(out->name, pi->name, sizeof(out->name));
        memcpy(out->message, pi->message, sizeof(out->message));
        out->state = pi->state;
        out->color = pi->color;
        out->def = pi->def;
        out->n_media = pi->n_media;
        memcpy(out->media, pi->media, sizeof(out->media));
    } else {
        memset(out, 0, sizeof(*out));
        to_macroman(out->name, sizeof(out->name), "Host printer");
        to_macroman(out->message, sizeof(out->message),
                    "The host printer does not answer");
        out->state = VPRINT_PROCESSING;
        out->color = 1;
        default_media(out);
    }
    qemu_mutex_unlock(&pi->lock);
}

/* PDF writing */

static uint32_t pdf_new_obj(VirtIOPrint *s)
{
    long zero = 0;

    g_array_append_val(s->offsets, zero);
    return s->offsets->len - 1;
}

static void pdf_begin_obj(VirtIOPrint *s, uint32_t n)
{
    g_array_index(s->offsets, long, n) = ftell(s->pdf);
    fprintf(s->pdf, "%u 0 obj\n", n);
}

static void pdf_pt(FILE *f, uint32_t hpt)
{
    fprintf(f, "%u.%02u", hpt / 100, hpt % 100);
}

static void job_discard(VirtIOPrint *s)
{
    if (s->zs) {
        deflateEnd(s->zs);
        g_free(s->zs);
        s->zs = NULL;
    }
    if (s->pdf) {
        fclose(s->pdf);
        s->pdf = NULL;
    }
    if (s->path) {
        unlink(s->path);
        g_free(s->path);
        s->path = NULL;
    }
    g_clear_pointer(&s->offsets, g_array_unref);
    g_clear_pointer(&s->pages, g_array_unref);
    g_clear_pointer(&s->images, g_array_unref);
    g_clear_pointer(&s->title, g_free);
    g_clear_pointer(&s->rowbuf, g_free);
    g_clear_pointer(&s->zbuf, g_free);
    s->in_job = s->in_page = false;
}

static bool zwrite(VirtIOPrint *s, int flush)
{
    z_stream *zs = s->zs;
    int r;

    do {
        zs->next_out = s->zbuf;
        zs->avail_out = 65536;
        r = deflate(zs, flush);
        if (r == Z_STREAM_ERROR) {
            return false;
        }
        fwrite(s->zbuf, 1, 65536 - zs->avail_out, s->pdf);
    } while (zs->avail_out == 0);
    return true;
}

static bool page_rows(VirtIOPrint *s, const uint8_t *argb, uint32_t n,
                      uint32_t stride)
{
    z_stream *zs = s->zs;
    uint32_t r, x;

    for (r = 0; r < n && s->rows_done < s->page_h; r++, s->rows_done++) {
        const uint8_t *p = argb ? argb + (size_t)r * stride : NULL;
        uint8_t *q = s->rowbuf;

        if (p) {
            for (x = 0; x < s->page_w; x++, p += 4) {
                *q++ = p[1];
                *q++ = p[2];
                *q++ = p[3];
            }
        } else {
            memset(q, 0xff, s->page_w * 3);
        }
        zs->next_in = s->rowbuf;
        zs->avail_in = s->page_w * 3;
        if (!zwrite(s, Z_NO_FLUSH)) {
            return false;
        }
    }
    return true;
}

static uint32_t do_begin_job(VirtIOPrint *s, const VirtIOPrintReq *req,
                             const char *title_mr)
{
    g_autofree char *dir = NULL;
    int fd;
    long zero = 0;

    job_discard(s);
    s->copies = req->a[0] ? MIN(req->a[0], 999) : 1;
    s->title = from_macroman(title_mr);

    if (s->save_to && *s->save_to) {
        g_autofree char *base = g_strdup(s->title);
        char *c;

        for (c = base; *c; c++) {
            if (*c == '/' || *c == ':') {
                *c = '-';
            }
        }
        if (!*base) {
            g_free(base);
            base = g_strdup("Untitled");
        }
        s->path = g_strdup_printf("%s/%s.pdf", s->save_to, base);
        for (int i = 2; g_file_test(s->path, G_FILE_TEST_EXISTS) && i < 1000;
             i++) {
            g_free(s->path);
            s->path = g_strdup_printf("%s/%s %d.pdf", s->save_to, base, i);
        }
        s->pdf = fopen(s->path, "wb");
    } else {
        GError *err = NULL;

        fd = g_file_open_tmp("qemu-print-XXXXXX.pdf", &s->path, &err);
        if (fd < 0) {
            error_report("virtio-print: %s", err->message);
            g_error_free(err);
            return VPRINT_ERR;
        }
        s->pdf = fdopen(fd, "wb");
    }
    if (!s->pdf) {
        error_report("virtio-print: cannot write %s", s->path);
        g_clear_pointer(&s->path, g_free);
        return VPRINT_ERR;
    }

    s->offsets = g_array_new(false, false, sizeof(long));
    s->pages = g_array_new(false, false, sizeof(uint32_t));
    s->images = g_array_new(false, false, sizeof(PageImage));
    g_array_append_val(s->offsets, zero);   /* object 0 */
    pdf_new_obj(s);                         /* 1: catalog */
    pdf_new_obj(s);                         /* 2: page tree */
    fputs("%PDF-1.4\n%\xe2\xe3\xcf\xd3\n", s->pdf);
    s->in_job = true;
    return VPRINT_OK;
}

static uint32_t do_begin_page(VirtIOPrint *s, const VirtIOPrintReq *req)
{
    uint32_t img, len;
    z_stream *zs;

    if (!s->in_job || s->in_page) {
        return VPRINT_BAD_STATE;
    }
    if (!req->a[0] || !req->a[1] || req->a[0] > MAX_PIXELS ||
        req->a[1] > MAX_PIXELS || !req->a[2] || !req->a[3]) {
        return VPRINT_ERR;
    }
    s->page_w = req->a[0];
    s->page_h = req->a[1];
    s->media_w = req->a[2];
    s->media_h = req->a[3];
    s->rows_done = 0;

    img = pdf_new_obj(s);
    len = pdf_new_obj(s);
    pdf_begin_obj(s, img);
    fprintf(s->pdf, "<< /Type /XObject /Subtype /Image /Width %u /Height %u"
            " /ColorSpace /DeviceRGB /BitsPerComponent 8"
            " /Filter /FlateDecode /Length %u 0 R >>\nstream\n",
            s->page_w, s->page_h, len);
    s->stream_start = ftell(s->pdf);

    zs = g_new0(z_stream, 1);
    if (deflateInit(zs, 6) != Z_OK) {
        g_free(zs);
        return VPRINT_ERR;
    }
    s->zs = zs;
    s->rowbuf = g_malloc((size_t)s->page_w * 3);
    s->zbuf = g_malloc(65536);
    s->in_page = true;
    return VPRINT_OK;
}

/*
 * Rows of pixels: a0 of them, a1 bytes apart. Optionally only part of each
 * row (a3 pixels from a2 across, the rest white) and from a row further
 * down (a4 - 1; the rows skipped are white).
 */
static uint32_t do_band(VirtIOPrint *s, const VirtIOPrintReq *req,
                        VirtQueueElement *elem)
{
    uint32_t rows = req->a[0], stride = req->a[1], r;
    uint32_t x = req->a[2], n = req->a[3] ? req->a[3] : s->page_w;
    g_autofree uint8_t *row = NULL;
    size_t off = sizeof(VirtIOPrintReq);

    if (!s->in_page) {
        return VPRINT_BAD_STATE;
    }
    if (rows > MAX_PIXELS || x > s->page_w || n > MAX_PIXELS ||
        stride < (size_t)n * 4) {
        return VPRINT_ERR;
    }
    if (req->a[4] && req->a[4] - 1 > s->rows_done &&
        !page_rows(s, NULL, req->a[4] - 1 - s->rows_done, 0)) {
        return VPRINT_ERR;
    }
    if (n > s->page_w - x) {
        n = s->page_w - x;
    }
    row = g_malloc((size_t)s->page_w * 4);
    for (r = 0; r < rows; r++, off += stride) {
        memset(row, 0xff, (size_t)s->page_w * 4);
        if (iov_to_buf(elem->out_sg, elem->out_num, off, row + (size_t)x * 4,
                       (size_t)n * 4) != (size_t)n * 4) {
            return VPRINT_ERR;
        }
        if (!page_rows(s, row, 1, 0)) {
            return VPRINT_ERR;
        }
    }
    return VPRINT_OK;
}

static uint32_t do_end_page(VirtIOPrint *s)
{
    uint32_t len_obj, content, page;
    long length;
    g_autofree char *ops = NULL;

    if (!s->in_page) {
        return VPRINT_BAD_STATE;
    }
    /* rows the guest never sent are white */
    if (!page_rows(s, NULL, s->page_h - s->rows_done, 0) ||
        !zwrite(s, Z_FINISH)) {
        return VPRINT_ERR;
    }
    deflateEnd(s->zs);
    g_clear_pointer(&s->zs, g_free);
    g_clear_pointer(&s->rowbuf, g_free);
    g_clear_pointer(&s->zbuf, g_free);

    length = ftell(s->pdf) - s->stream_start;
    {
        PageImage im = { s->stream_start, length, s->page_w, s->page_h,
                         s->media_w, s->media_h };

        g_array_append_val(s->images, im);
    }
    fputs("\nendstream\nendobj\n", s->pdf);
    len_obj = s->offsets->len - 1;
    pdf_begin_obj(s, len_obj);
    fprintf(s->pdf, "%ld\nendobj\n", length);

    content = pdf_new_obj(s);
    ops = g_strdup_printf("q %u.%02u 0 0 %u.%02u 0 0 cm /Im0 Do Q",
                          s->media_w / 100, s->media_w % 100,
                          s->media_h / 100, s->media_h % 100);
    pdf_begin_obj(s, content);
    fprintf(s->pdf, "<< /Length %zu >>\nstream\n%s\nendstream\nendobj\n",
            strlen(ops), ops);

    page = pdf_new_obj(s);
    pdf_begin_obj(s, page);
    fputs("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 ", s->pdf);
    pdf_pt(s->pdf, s->media_w);
    fputc(' ', s->pdf);
    pdf_pt(s->pdf, s->media_h);
    fprintf(s->pdf, "] /Resources << /XObject << /Im0 %u 0 R >> >>"
            " /Contents %u 0 R >>\nendobj\n", len_obj - 1, content);
    g_array_append_val(s->pages, page);
    s->in_page = false;
    return VPRINT_OK;
}

#ifdef _WIN32
/* The paper of this size the printer knows, else a custom one */
static void win_set_paper(DEVMODEW *dm, wchar_t *name, uint32_t w_hpt,
                          uint32_t h_hpt)
{
    bool land = w_hpt > h_hpt;
    LONG pw = (LONG)((int64_t)(land ? h_hpt : w_hpt) * 254 / 7200);
    LONG ph = (LONG)((int64_t)(land ? w_hpt : h_hpt) * 254 / 7200);
    int i, n = DeviceCapabilitiesW(name, NULL, DC_PAPERS, NULL, NULL);

    dm->dmFields |= DM_ORIENTATION | DM_PAPERSIZE;
    dm->dmOrientation = land ? DMORIENT_LANDSCAPE : DMORIENT_PORTRAIT;
    if (n > 0 && n < 1000) {
        g_autofree WORD *ids = g_new0(WORD, n);
        g_autofree POINT *sz = g_new0(POINT, n);

        if (DeviceCapabilitiesW(name, NULL, DC_PAPERS, (LPWSTR)ids,
                                NULL) == n &&
            DeviceCapabilitiesW(name, NULL, DC_PAPERSIZE, (LPWSTR)sz,
                                NULL) == n) {
            for (i = 0; i < n; i++) {
                if (labs(sz[i].x - pw) <= 20 && labs(sz[i].y - ph) <= 20) {
                    dm->dmPaperSize = ids[i];
                    dm->dmFields &= ~(DM_PAPERWIDTH | DM_PAPERLENGTH);
                    return;
                }
            }
        }
    }
    dm->dmPaperSize = DMPAPER_USER;
    dm->dmPaperWidth = pw;
    dm->dmPaperLength = ph;
    dm->dmFields |= DM_PAPERWIDTH | DM_PAPERLENGTH;
}

/* One page image, inflated a strip at a time onto the whole sheet */
static bool win_draw_page(HDC dc, FILE *f, const PageImage *im)
{
    int ox = GetDeviceCaps(dc, PHYSICALOFFSETX);
    int oy = GetDeviceCaps(dc, PHYSICALOFFSETY);
    int pw = GetDeviceCaps(dc, PHYSICALWIDTH);
    int ph = GetDeviceCaps(dc, PHYSICALHEIGHT);
    size_t src = (size_t)im->w * 3, dst = (src + 3) & ~(size_t)3;
    uint32_t strip = MAX(1, (4 << 20) / dst), row = 0, r;
    g_autofree uint8_t *in = g_malloc(65536);
    g_autofree uint8_t *rgb = g_malloc(src);
    g_autofree uint8_t *bits = g_malloc0(dst * strip);
    BITMAPINFO bmi;
    z_stream zs;
    long left = im->length;
    bool ok = true;

    memset(&bmi, 0, sizeof(bmi));
    bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
    bmi.bmiHeader.biWidth = im->w;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;
    memset(&zs, 0, sizeof(zs));
    if (fseek(f, im->start, SEEK_SET) || inflateInit(&zs) != Z_OK) {
        return false;
    }
    SetStretchBltMode(dc, COLORONCOLOR);

    while (ok && row < im->h) {
        uint32_t n = MIN(strip, im->h - row);
        int y0, y1;

        for (r = 0; ok && r < n; r++) {
            uint8_t *q = bits + (size_t)r * dst;
            uint32_t x;

            zs.next_out = rgb;
            zs.avail_out = src;
            while (zs.avail_out) {
                int z;

                if (!zs.avail_in) {
                    size_t got = fread(in, 1, MIN(65536, left), f);

                    if (!got) {
                        ok = false;
                        break;
                    }
                    left -= got;
                    zs.next_in = in;
                    zs.avail_in = got;
                }
                z = inflate(&zs, Z_NO_FLUSH);
                if (z != Z_OK && !(z == Z_STREAM_END && !zs.avail_out)) {
                    ok = false;
                    break;
                }
            }
            for (x = 0; x < im->w; x++) {
                q[x * 3] = rgb[x * 3 + 2];
                q[x * 3 + 1] = rgb[x * 3 + 1];
                q[x * 3 + 2] = rgb[x * 3];
            }
        }
        if (!ok) {
            break;
        }
        bmi.bmiHeader.biHeight = -(LONG)n;     /* top-down */
        y0 = (int)((int64_t)row * ph / im->h);
        y1 = (int)((int64_t)(row + n) * ph / im->h);
        r = StretchDIBits(dc, -ox, y0 - oy, pw, y1 - y0, 0, 0, im->w, n,
                          bits, &bmi, DIB_RGB_COLORS, SRCCOPY);
        if (r == 0 || r == (uint32_t)GDI_ERROR) {
            ok = false;
        }
        row += n;
    }
    inflateEnd(&zs);
    return ok;
}

static uint32_t win_print(VirtIOPrint *s, char *message, uint32_t *job)
{
    g_autofree wchar_t *wname = win_printer(s->printer);
    g_autofree wchar_t *wtitle = NULL;
    g_autofree DEVMODEW *dm = NULL;
    g_autofree char *name = NULL;
    const char *err = NULL;
    HANDLE h = NULL;
    HDC dc = NULL;
    FILE *f = NULL;
    DOCINFOW di;
    uint32_t reps = 1, rep, i;
    int id = 0, max_copies;
    const PageImage *cur = NULL;

    if (!wname || !OpenPrinterW(wname, &h, NULL)) {
        to_macroman(message, 128, "The host has no such printer");
        return VPRINT_ERR;
    }
    name = from_wide(wname);
    dm = win_devmode(h, wname);
    if (!dm) {
        err = "The printer's settings cannot be read";
        goto out;
    }
    max_copies = DeviceCapabilitiesW(wname, NULL, DC_COPIES, NULL, NULL);
    dm->dmFields |= DM_COPIES | DM_COLLATE;
    if (max_copies >= (int)s->copies) {
        dm->dmCopies = s->copies;
    } else {
        dm->dmCopies = 1;
        reps = s->copies;
    }
    dm->dmCollate = DMCOLLATE_TRUE;
    cur = &g_array_index(s->images, PageImage, 0);
    win_set_paper(dm, wname, cur->media_w, cur->media_h);
    DocumentPropertiesW(NULL, h, wname, dm, dm, DM_IN_BUFFER | DM_OUT_BUFFER);

    f = fopen(s->path, "rb");
    dc = f ? CreateDCW(L"WINSPOOL", wname, NULL, dm) : NULL;
    if (!dc) {
        err = f ? "The printer cannot be opened" : "The job cannot be read";
        goto out;
    }
    wtitle = (wchar_t *)g_utf8_to_utf16(s->title && *s->title ? s->title :
                                        "Untitled", -1, NULL, NULL, NULL);
    memset(&di, 0, sizeof(di));
    di.cbSize = sizeof(di);
    di.lpszDocName = wtitle;
    id = StartDocW(dc, &di);
    if (id <= 0) {
        err = "The printer did not take the job";
        goto out;
    }
    for (rep = 0; rep < reps && !err; rep++) {
        for (i = 0; i < s->images->len && !err; i++) {
            const PageImage *im = &g_array_index(s->images, PageImage, i);

            if (im->media_w != cur->media_w || im->media_h != cur->media_h) {
                win_set_paper(dm, wname, im->media_w, im->media_h);
                DocumentPropertiesW(NULL, h, wname, dm, dm,
                                    DM_IN_BUFFER | DM_OUT_BUFFER);
                ResetDCW(dc, dm);
                cur = im;
            }
            if (StartPage(dc) <= 0 || !win_draw_page(dc, f, im) ||
                EndPage(dc) <= 0) {
                err = "The printer stopped taking the job";
            }
        }
    }
    if (err) {
        AbortDoc(dc);
        id = 0;
    } else if (EndDoc(dc) <= 0) {
        err = "The printer did not finish the job";
        id = 0;
    }

out:
    if (dc) {
        DeleteDC(dc);
    }
    if (f) {
        fclose(f);
    }
    ClosePrinter(h);
    if (err) {
        to_macroman(message, 128, err);
        return VPRINT_ERR;
    }
    {
        g_autofree char *m = g_strdup_printf("Sent to %s", name);

        to_macroman(message, 128, m);
    }
    *job = id;
    return VPRINT_OK;
}
#endif

static uint32_t do_end_job(VirtIOPrint *s, char *message, uint32_t *job)
{
    long xref;
    uint32_t i;

    *job = 0;
    if (!s->in_job || s->in_page) {
        return VPRINT_BAD_STATE;
    }
    if (!s->pages->len) {
        job_discard(s);
        return VPRINT_OK;
    }
    pdf_begin_obj(s, 1);
    fputs("<< /Type /Catalog /Pages 2 0 R >>\nendobj\n", s->pdf);
    pdf_begin_obj(s, 2);
    fputs("<< /Type /Pages /Kids [", s->pdf);
    for (i = 0; i < s->pages->len; i++) {
        fprintf(s->pdf, " %u 0 R", g_array_index(s->pages, uint32_t, i));
    }
    fprintf(s->pdf, " ] /Count %u >>\nendobj\n", s->pages->len);
    xref = ftell(s->pdf);
    fprintf(s->pdf, "xref\n0 %u\n0000000000 65535 f \n", s->offsets->len);
    for (i = 1; i < s->offsets->len; i++) {
        fprintf(s->pdf, "%010ld 00000 n \n",
                g_array_index(s->offsets, long, i));
    }
    fprintf(s->pdf, "trailer\n<< /Size %u /Root 1 0 R >>\nstartxref\n%ld\n"
            "%%%%EOF\n", s->offsets->len, xref);
    if (fclose(s->pdf) != 0) {
        s->pdf = NULL;
        to_macroman(message, 128, "The host could not write the job");
        job_discard(s);
        return VPRINT_ERR;
    }
    s->pdf = NULL;

    if (s->save_to && *s->save_to) {
        g_autofree char *base = g_path_get_basename(s->path);
        g_autofree char *m = g_strdup_printf("Saved as %s", base);

        to_macroman(message, 128, m);
        g_clear_pointer(&s->path, g_free);     /* keep the file */
        job_discard(s);
        return VPRINT_OK;
    }

#ifdef CONFIG_CUPS
    {
        cups_dest_t *dest = cupsGetNamedDest(CUPS_HTTP_DEFAULT,
                                             s->printer && *s->printer ?
                                             s->printer : NULL, NULL);
        cups_option_t *opts = NULL;
        int nopts = 0, id = 0;
        g_autofree char *copies = g_strdup_printf("%u", s->copies);

        if (dest) {
            nopts = cupsAddOption("copies", copies, nopts, &opts);
            id = cupsPrintFile(dest->name, s->path, s->title, nopts, opts);
            cupsFreeOptions(nopts, opts);
        }
        if (id > 0) {
            g_autofree char *m = NULL;
            const char *pname = cupsGetOption("printer-info",
                                              dest->num_options,
                                              dest->options);

            m = g_strdup_printf("Sent to %s", pname && *pname ? pname :
                                dest->name);
            to_macroman(message, 128, m);
            *job = id;
        } else {
            to_macroman(message, 128, dest ? cupsLastErrorString() :
                        "The host has no such printer");
        }
        if (dest) {
            cupsFreeDests(1, dest);
        }
        job_discard(s);         /* CUPS has its own copy of the file */
        return id > 0 ? VPRINT_OK : VPRINT_ERR;
    }
#elif defined(_WIN32)
    {
        uint32_t r = win_print(s, message, job);

        job_discard(s);
        return r;
    }
#else
    to_macroman(message, 128, "This host cannot print; use save-to=");
    job_discard(s);
    return VPRINT_ERR;
#endif
}

static void handle_request(VirtIOPrint *s, VirtQueueElement *elem)
{
    VirtIOPrintReq req;
    VirtIOPrintResp resp;
    char payload[256];
    size_t plen = 0;
    PrintInfo pi;
    uint32_t i;

    memset(&resp, 0, sizeof(resp));
    memset(payload, 0, sizeof(payload));
    if (iov_to_buf(elem->out_sg, elem->out_num, 0, &req, sizeof(req)) !=
        sizeof(req)) {
        resp.status = cpu_to_le32(VPRINT_ERR);
        goto out;
    }
    req.op = le32_to_cpu(req.op);
    for (i = 0; i < 7; i++) {
        req.a[i] = le32_to_cpu(req.a[i]);
    }

    switch (req.op) {
    case VPRINT_INFO:
        get_info(s, &pi);
        memcpy(payload, pi.name, 64);
        memcpy(payload + 64, pi.message, 128);
        plen = 192;
        resp.b[0] = pi.state;
        resp.b[1] = pi.n_media;
        resp.b[2] = pi.def;
        resp.b[3] = pi.color;
        resp.status = VPRINT_OK;
        break;
    case VPRINT_MEDIA:
        get_info(s, &pi);
        if (req.a[0] >= pi.n_media) {
            resp.status = VPRINT_NO_MEDIA;
            break;
        }
        memcpy(payload, pi.media[req.a[0]].name, 64);
        plen = 64;
        for (i = 0; i < 6; i++) {
            resp.b[i] = pi.media[req.a[0]].size[i];
        }
        resp.status = VPRINT_OK;
        break;
    case VPRINT_BEGIN_JOB: {
        char title[256];

        memset(title, 0, sizeof(title));
        iov_to_buf(elem->out_sg, elem->out_num, sizeof(req), title,
                   sizeof(title) - 1);
        resp.status = do_begin_job(s, &req, title);
        break;
    }
    case VPRINT_BEGIN_PAGE:
        resp.status = do_begin_page(s, &req);
        break;
    case VPRINT_BAND:
        resp.status = do_band(s, &req, elem);
        break;
    case VPRINT_END_PAGE:
        resp.status = do_end_page(s);
        break;
    case VPRINT_END_JOB:
        resp.status = do_end_job(s, payload, &resp.b[0]);
        plen = 128;
        break;
    case VPRINT_ABORT:
        job_discard(s);
        resp.status = VPRINT_OK;
        break;
    default:
        resp.status = VPRINT_ERR;
        break;
    }
    if (resp.status != VPRINT_OK && resp.status != VPRINT_NO_MEDIA &&
        req.op >= VPRINT_BEGIN_PAGE && req.op <= VPRINT_END_JOB &&
        s->in_job) {
        /* a broken job is not worth finishing */
        if (req.op != VPRINT_END_JOB) {
            job_discard(s);
        }
    }

out:
    resp.status = cpu_to_le32(resp.status);
    for (i = 0; i < 7; i++) {
        resp.b[i] = cpu_to_le32(resp.b[i]);
    }
    iov_from_buf(elem->in_sg, elem->in_num, 0, &resp, sizeof(resp));
    if (plen) {
        iov_from_buf(elem->in_sg, elem->in_num, sizeof(resp), payload, plen);
    }
    virtqueue_push(s->vq, elem, MIN(iov_size(elem->in_sg, elem->in_num),
                                    sizeof(resp) + plen));
}

static void handle_output(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOPrint *s = VIRTIO_PRINT(vdev);
    VirtQueueElement *elem;

    while ((elem = virtqueue_pop(vq, sizeof(VirtQueueElement)))) {
        handle_request(s, elem);
        g_free(elem);
    }
    virtio_notify(vdev, vq);
}

static uint64_t get_features(VirtIODevice *vdev, uint64_t f, Error **errp)
{
    return f;
}

static void virtio_print_reset(VirtIODevice *vdev)
{
    job_discard(VIRTIO_PRINT(vdev));
}

static void virtio_print_realize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VirtIOPrint *s = VIRTIO_PRINT(dev);
    PrintInfo *pi;

    if (s->save_to && *s->save_to &&
        !g_file_test(s->save_to, G_FILE_TEST_IS_DIR)) {
        error_setg(errp, "virtio-print: save-to=%s is not a folder",
                   s->save_to);
        return;
    }
#ifndef HOST_PRINTING
    if (!s->save_to || !*s->save_to) {
        warn_report("virtio-print: this host cannot print; use save-to=");
    }
#endif
    virtio_init(vdev, VIRTIO_ID_PRINT, 0);
    s->vq = virtio_add_queue(vdev, 64, handle_output);

    pi = g_new0(PrintInfo, 1);
    s->info = pi;
    qemu_mutex_init(&pi->lock);
    qemu_cond_init(&pi->cond);
    pi->want = true;            /* ask the printer early */
    qemu_thread_create(&pi->thread, "virtio-print", info_thread, s,
                       QEMU_THREAD_JOINABLE);
}

static void virtio_print_unrealize(DeviceState *dev)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VirtIOPrint *s = VIRTIO_PRINT(dev);
    PrintInfo *pi = info_of(s);

    job_discard(s);
    qemu_mutex_lock(&pi->lock);
    pi->quit = true;
    qemu_cond_broadcast(&pi->cond);
    qemu_mutex_unlock(&pi->lock);
    qemu_thread_join(&pi->thread);
    qemu_cond_destroy(&pi->cond);
    qemu_mutex_destroy(&pi->lock);
    g_free(pi);
    s->info = NULL;
    virtio_del_queue(vdev, 0);
    virtio_cleanup(vdev);
}

static const VMStateDescription vmstate_virtio_print = {
    .name = "virtio-print",
    .minimum_version_id = 1,
    .version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VIRTIO_DEVICE,
        VMSTATE_END_OF_LIST()
    },
};

static const Property virtio_print_properties[] = {
    DEFINE_PROP_STRING("printer", VirtIOPrint, printer),
    DEFINE_PROP_STRING("save-to", VirtIOPrint, save_to),
};

static void virtio_print_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VirtioDeviceClass *vdc = VIRTIO_DEVICE_CLASS(klass);

    device_class_set_props(dc, virtio_print_properties);
    dc->vmsd = &vmstate_virtio_print;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    vdc->realize = virtio_print_realize;
    vdc->unrealize = virtio_print_unrealize;
    vdc->get_features = get_features;
    vdc->reset = virtio_print_reset;
}

static const TypeInfo virtio_print_info = {
    .name = TYPE_VIRTIO_PRINT,
    .parent = TYPE_VIRTIO_DEVICE,
    .instance_size = sizeof(VirtIOPrint),
    .class_init = virtio_print_class_init,
};

static void virtio_print_register_types(void)
{
    type_register_static(&virtio_print_info);
}

type_init(virtio_print_register_types)
