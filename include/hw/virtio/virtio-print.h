/*
 * Virtio printer: pages rendered by the guest, printed on the host
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VIRTIO_PRINT_H
#define HW_VIRTIO_PRINT_H

#include "hw/virtio/virtio.h"
#include "qom/object.h"

#define TYPE_VIRTIO_PRINT "virtio-print-device"
OBJECT_DECLARE_SIMPLE_TYPE(VirtIOPrint, VIRTIO_PRINT)

/* Not a standard virtio device ID */
#define VIRTIO_ID_PRINT 0x80

/*
 * Each request is a header, then the request's data; the device answers
 * with a response, then the response's data. All little-endian.
 */
enum {
    VPRINT_INFO = 0,        /* -> name[64], message[128] */
    VPRINT_MEDIA = 1,       /* a0 index -> media name[64] */
    VPRINT_BEGIN_JOB = 2,   /* a0 copies, data title[256] */
    VPRINT_BEGIN_PAGE = 3,  /* a0 width px, a1 height px, a2 w, a3 h (1/100 pt) */
    VPRINT_BAND = 4,        /* a0 rows, a1 bytes per row, data the rows;
                               a2 x, a3 pixels (0: all), a4 row + 1 (0: next) */
    VPRINT_END_PAGE = 5,
    VPRINT_END_JOB = 6,     /* -> message[128] */
    VPRINT_ABORT = 7,
};

enum {
    VPRINT_OK = 0,
    VPRINT_ERR = 1,         /* message[128] says why, where there is one */
    VPRINT_NO_MEDIA = 2,
    VPRINT_BAD_STATE = 3,
};

/* printer states, as IPP */
enum {
    VPRINT_IDLE = 3,
    VPRINT_PROCESSING = 4,
    VPRINT_STOPPED = 5,
};

typedef struct VirtIOPrintReq {
    uint32_t op;
    uint32_t a[7];
} VirtIOPrintReq;

typedef struct VirtIOPrintResp {
    uint32_t status;
    uint32_t b[7];
} VirtIOPrintResp;

/*
 * INFO: b0 state, b1 media count, b2 default media index,
 *       b3 color (1) or not (0)
 * MEDIA: b0 width, b1 height, b2 left, b3 top, b4 right, b5 bottom
 *        (all 1/100 pt; the margins are what the printer cannot reach)
 * END_JOB: b0 host job id (0 when saved to a file)
 */

typedef struct VPrintMedia {
    char name[64];          /* Mac Roman */
    uint32_t size[6];       /* as MEDIA's b0..b5 */
} VPrintMedia;

struct VirtIOPrint {
    VirtIODevice parent_obj;
    VirtQueue *vq;

    char *printer;          /* host queue name; NULL/"" = the default */
    char *save_to;          /* folder: save PDFs there instead of printing */
    void *info;             /* the printer as last asked */

    /* the job in progress */
    bool in_job, in_page;
    char *title;
    uint32_t copies;
    char *path;             /* the PDF being written */
    FILE *pdf;
    GArray *offsets;        /* of each PDF object */
    GArray *pages;          /* object numbers of the page objects */
    GArray *images;         /* where each page's image is */
    uint32_t page_w, page_h;        /* pixels */
    uint32_t media_w, media_h;      /* 1/100 pt */
    uint32_t rows_done;
    void *zs;               /* z_stream of the page image */
    long stream_start;
    uint8_t *rowbuf;
    uint8_t *zbuf;
};

#endif
