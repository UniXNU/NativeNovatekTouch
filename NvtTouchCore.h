// SPDX-License-Identifier: Apache-2.0
// Finger-touch processing for the NT36532E THP matrix: baseline, delta, peaks, centroid and slot tracking.
// Constants and axis mapping follow xiaomi-sheng-thp (Apache-2.0); see NOTICE. Plain C, no allocation.
#ifndef NVT_TOUCH_CORE_H
#define NVT_TOUCH_CORE_H
#include <stdint.h>
#include <stddef.h>

#define NVT_ROWS 40
#define NVT_COLUMNS 60
#define NVT_NODES (NVT_ROWS*NVT_COLUMNS)
#define NVT_SUPER_RESOLUTION 10
// Steps below 4 * NVT_FUZZ count as the positional noise of a resting or slow finger (jitter_*), in coordinate units (0.1 panel px).
#define NVT_FUZZ 60
// A resting finger is locked where it rests: it unlocks once it is NVT_UNLOCK away, and a moving finger locks again when its last
// NVT_REST_FRAMES positions span less than NVT_REST_SPAN (all in 0.1 px). int-271: resting fingers wandered 5 px per frame (24 px at most),
// so SpringBoard saw 100..180 pt/s at lift-off and opened the app switcher once in a dozen pauses.
#define NVT_UNLOCK 200
#define NVT_REST_FRAMES 6
#define NVT_REST_SPAN 200
// The centroid weighs a node by how far it is above this share of its blob's peak (percent), so the fringe nodes that come and go with
// noise do not pull the position around.
#define NVT_CENTROID_FLOOR 30
// Frames a tracked finger may go unseen before it counts as lifted (8.3 ms each at 120 Hz). The finger is reported where it was meanwhile.
#define NVT_HOLD_FRAMES 3
#define NVT_SLOTS 12
#define NVT_STARTUP_FRAMES 72
// Output axes (short axis follows rows, long axis follows columns), before any display orientation.
#define NVT_LONG_AXIS_MAX 30480
#define NVT_SHORT_AXIS_MAX 20320

#define NVT_NOISE_THRESHOLD 100
#define NVT_PEAK_THRESHOLD 220
#define NVT_DOWN_THRESHOLD 300
#define NVT_UP_THRESHOLD 220
#define NVT_COORDINATE_REDUCE 25
#define NVT_PALM_MIN_NODES 60

struct nvt_contact {
    int slot, tracking_id;
    int x, y;        // x along the long axis, y along the short axis, 0 .. max-1
    int peak, area, weight;
};

struct nvt_slot {
    int active, tracking_id, age, x, y;
    int rx, ry;      // the position last reported, after the fuzz filter (x, y stay raw: tracking uses them)
    int moving;      // following the finger; 0 while it is locked at rest
    int hx[NVT_REST_FRAMES], hy[NVT_REST_FRAMES], hn;   // its last raw positions
    int missed;      // consecutive frames without a matching blob (held while <= NVT_HOLD_FRAMES)
    int miss_reason; // why the first of those frames had no match (NVT_MISS_*)
};

// Why a tracked finger found no blob: none at all, the nearest beyond the tracking radius, below the up threshold, palm-sized, or already
// taken by another finger.
enum {NVT_MISS_NONE, NVT_MISS_FAR, NVT_MISS_WEAK, NVT_MISS_PALM, NVT_MISS_TAKEN, NVT_MISS_KINDS};

struct nvt_touch_core {
    int16_t reference[NVT_NODES];
    int have_reference;
    unsigned startup_count;
    int16_t startup[NVT_STARTUP_FRAMES][NVT_NODES];   // collected until the median reference is formed
    // Raw frame-to-frame movement of tracked contacts that moved less than 4 * NVT_FUZZ (a resting or slow finger): frames, summed
    // and largest step per axis, in coordinate units: the noise the rest lock has to cover. Rests: times a moving finger was locked;
    // unlocks: times a locked one started moving.
    unsigned long jitter_frames, jitter_sum;
    int jitter_max;
    unsigned long rests, unlocks;
    // Fingers that went unseen and came back within NVT_HOLD_FRAMES, fingers lifted per miss reason, and frames by type (2 / 4).
    unsigned long holds_bridged, ends[NVT_MISS_KINDS], frames_by_type[2];
    struct nvt_slot slots[NVT_SLOTS];
    // Scratch for nvt_touch_process: 19 KB, kept here because kernel threads have 16 KiB stacks (a stack probe in the
    // function prologue faulted on hardware when these were locals).
    int delta[NVT_NODES];
    int label[NVT_NODES];
};

void nvt_touch_reset(struct nvt_touch_core *core);
// Feed one touch matrix (row-major, 40x60 int16). Returns the number of visible contacts written to out[0..max)
// (-1 while the startup baseline is still being collected). frame_type 2 = baseline frame.
int nvt_touch_process(struct nvt_touch_core *core, const int16_t *matrix, unsigned frame_type,
                      struct nvt_contact *out, int max);
// Edge compensation applied to a raw centroid coordinate.
int nvt_touch_map_edge_axis(int value, int maximum, int low_edge_bias);
// Row/column centre tables (before super resolution).
int nvt_touch_row_center(int row);
int nvt_touch_column_center(int column);
#endif
