/*
 * Speaker tracking and counting over time (decision 1c: direction histogram + tracker).
 * Port of daq-sim/daq_sim/track.py (the specification is in its docstring); computed in double so that
 * replay CSVs match the Python reference (tools/localize_wav.py --compare).
 *
 * Modes (settings tracker_mode): AZ = azimuth-only histogram and distance, AZEL = azimuth x elevation,
 * ELLIPSE = azimuth x elevation with an elliptical gate. Called from dsp.c for every frame; not thread safe
 * (dsp_task only).
 */
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "daq.h"

#define N_AZ                 DAQ_BLE_MAP_AZ_BINS
#define N_EL                 DAQ_BLE_MAP_EL_BINS
#define HOP_S                ((double)DSP_HOP / DAQ_SAMPLE_RATE_HZ)
#define HIST_HALF_LIFE_S     1.0
#define BIRTH_HIST           1.5
#define GATE_DEG             20.0
#define ELLIPSE_AZ_DEG       15.0
#define ELLIPSE_EL_DEG       45.0
#define ALPHA                0.3
#define CONFIRM_HITS         4
#define TENTATIVE_TIMEOUT_S  0.3
#define CONFIRMED_TIMEOUT_S  1.5
#define MERGE_DEG            10.0
#define MERGE_DEG_AZ         30.0

#define DEG                  (M_PI / 180.0)

typedef struct {
    uint8_t  id;
    double   az, el, dist, conf;
    int      hits;
    uint64_t born, last;
    bool     confirmed;
    bool     updated;          /* updated in the current frame */
} track_t;

static uint8_t s_mode = DAQ_TRACK_AZ;
static double  s_hist[N_EL][N_AZ];    /* AZ mode uses row 0 only */
static track_t s_tracks[DAQ_TRACK_MAX];
static int     s_n;
static uint8_t s_next_id = 1;

static double pymod(double a, double m)   /* Python a % m (result has the sign of m) */
{
    double r = fmod(a, m);
    return (r != 0.0 && (r < 0.0) != (m < 0.0)) ? r + m : r;
}

static double az_diff(double a1, double a2)
{
    return fabs(pymod(a1 - a2 + 180.0, 360.0) - 180.0);
}

static double angular_distance(double az1, double el1, double az2, double el2)
{
    double a1 = az1 * DEG, e1 = el1 * DEG, a2 = az2 * DEG, e2 = el2 * DEG;
    double c = sin(e1) * sin(e2) + cos(e1) * cos(e2) * cos(a1 - a2);
    c = c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c);
    return acos(c) / DEG;
}

static double distance(double az1, double el1, double az2, double el2)
{
    switch (s_mode) {
    case DAQ_TRACK_AZ:
        return az_diff(az1, az2);
    case DAQ_TRACK_AZEL:
        return angular_distance(az1, el1, az2, el2);
    default: {
        double a = az_diff(az1, az2) / ELLIPSE_AZ_DEG, e = fabs(el1 - el2) / ELLIPSE_EL_DEG;
        return GATE_DEG * (a > e ? a : e);
    }
    }
}

static void cell(double az, double el, int *j, int *k)
{
    long kk = (long)floor(az / 10.0 + 0.5) % N_AZ;
    *k = (int)(kk < 0 ? kk + N_AZ : kk);
    long jj = (long)floor((el + 85.0) / 10.0 + 0.5);
    *j = (int)(jj < 0 ? 0 : (jj > N_EL - 1 ? N_EL - 1 : jj));
}

static void hist_add(int j, int k)
{
    if (s_mode == DAQ_TRACK_AZ) {
        s_hist[0][(k + N_AZ - 1) % N_AZ] += 0.5;
        s_hist[0][k] += 1.0;
        s_hist[0][(k + 1) % N_AZ] += 0.5;
        return;
    }
    for (int dj = -1; dj <= 1; dj++) {
        int jj = j + dj;
        if (jj < 0 || jj >= N_EL) {
            continue;
        }
        for (int dk = -1; dk <= 1; dk++) {
            double kern = (dj == 0 && dk == 0) ? 1.0 : ((dj == 0 || dk == 0) ? 0.5 : 0.25);
            s_hist[jj][(k + dk + N_AZ) % N_AZ] += kern;
        }
    }
}

static double hist_at(int j, int k)
{
    return s_mode == DAQ_TRACK_AZ ? s_hist[0][k] : s_hist[j][k];
}

void track_reset(uint8_t mode)
{
    s_mode = (mode >= DAQ_TRACK_AZ && mode <= DAQ_TRACK_ELLIPSE) ? mode : DAQ_TRACK_AZ;
    memset(s_hist, 0, sizeof(s_hist));
    s_n = 0;
    s_next_id = 1;
}

int track_update(uint64_t frame, const loc_result_t *loc, track_out_t out[DAQ_TRACK_MAX])
{
    static const double decay_base = 0.5;
    double decay = pow(decay_base, HOP_S / HIST_HALF_LIFE_S);
    int rows = s_mode == DAQ_TRACK_AZ ? 1 : N_EL;
    for (int j = 0; j < rows; j++) {
        for (int k = 0; k < N_AZ; k++) {
            s_hist[j][k] *= decay;
        }
    }

    /* sources in order of decreasing confidence (stable) */
    int ns = loc ? loc->n_sources : 0;
    const loc_source_t *src[2] = { NULL, NULL };
    for (int s = 0; s < ns && s < 2; s++) {
        src[s] = &loc->src[s];
    }
    if (ns == 2 && src[1]->confidence > src[0]->confidence) {
        const loc_source_t *t = src[0];
        src[0] = src[1];
        src[1] = t;
    }
    for (int s = 0; s < ns; s++) {
        int j, k;
        cell(src[s]->az_deg, src[s]->el_deg, &j, &k);
        hist_add(j, k);
    }

    for (int t = 0; t < s_n; t++) {
        s_tracks[t].updated = false;
    }
    for (int s = 0; s < ns; s++) {
        double az = src[s]->az_deg, el = src[s]->el_deg, dist = src[s]->dist_m, conf = src[s]->confidence;
        track_t *best = NULL;
        double best_d = GATE_DEG;
        for (int t = 0; t < s_n; t++) {
            if (s_tracks[t].updated) {
                continue;
            }
            double d = distance(az, el, s_tracks[t].az, s_tracks[t].el);
            if (d <= best_d) {
                best = &s_tracks[t];
                best_d = d;
            }
        }
        if (best) {
            double ua[3] = { cos(best->el * DEG) * cos(best->az * DEG), cos(best->el * DEG) * sin(best->az * DEG), sin(best->el * DEG) };
            double ub[3] = { cos(el * DEG) * cos(az * DEG), cos(el * DEG) * sin(az * DEG), sin(el * DEG) };
            double v[3];
            for (int i = 0; i < 3; i++) {
                v[i] = (1 - ALPHA) * ua[i] + ALPHA * ub[i];
            }
            double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            for (int i = 0; i < 3; i++) {
                v[i] /= n;
            }
            best->az = pymod(atan2(v[1], v[0]) / DEG, 360.0);
            best->el = atan2(v[2], hypot(v[0], v[1])) / DEG;
            if (dist > 0) {
                best->dist = best->dist == 0 ? dist : (1 - ALPHA) * best->dist + ALPHA * dist;
            }
            best->conf = (1 - ALPHA) * best->conf + ALPHA * conf;
            best->hits++;
            best->last = frame;
            best->confirmed = best->confirmed || best->hits >= CONFIRM_HITS;
            best->updated = true;
        } else {
            int j, k;
            cell(az, el, &j, &k);
            if (hist_at(j, k) >= BIRTH_HIST && s_n < DAQ_TRACK_MAX) {
                s_tracks[s_n++] = (track_t) {
                    .id = s_next_id, .az = az, .el = el, .dist = dist, .conf = conf, .hits = 1,
                    .born = frame, .last = frame, .confirmed = false, .updated = true,
                };
                s_next_id = s_next_id < 255 ? s_next_id + 1 : 1;
            }
        }
    }

    /* timeouts, then merge (creation order: an older track wins) */
    int m = 0;
    for (int t = 0; t < s_n; t++) {
        double timeout = (s_tracks[t].confirmed ? CONFIRMED_TIMEOUT_S : TENTATIVE_TIMEOUT_S) * DAQ_SAMPLE_RATE_HZ;
        if ((double)(frame - s_tracks[t].last) <= timeout) {
            s_tracks[m++] = s_tracks[t];
        }
    }
    s_n = m;
    double merge = s_mode == DAQ_TRACK_AZ ? MERGE_DEG_AZ : MERGE_DEG;
    m = 0;
    for (int t = 0; t < s_n; t++) {
        bool keep = true;
        for (int o = 0; o < m && keep; o++) {
            keep = distance(s_tracks[t].az, s_tracks[t].el, s_tracks[o].az, s_tracks[o].el) > merge;
        }
        if (keep) {
            s_tracks[m++] = s_tracks[t];
        }
    }
    s_n = m;

    /* confirmed tracks by decreasing confidence (stable insertion sort on the double values) */
    int idx[DAQ_TRACK_MAX], n = 0;
    for (int t = 0; t < s_n; t++) {
        if (!s_tracks[t].confirmed) {
            continue;
        }
        int i = n++;
        while (i > 0 && s_tracks[idx[i - 1]].conf < s_tracks[t].conf) {
            idx[i] = idx[i - 1];
            i--;
        }
        idx[i] = t;
    }
    for (int i = 0; i < n; i++) {
        const track_t *t = &s_tracks[idx[i]];
        out[i] = (track_out_t) {
            .id = t->id, .az_deg = (float)t->az, .el_deg = (float)t->el,
            .dist_m = (float)t->dist, .confidence = (float)t->conf,
        };
    }
    return n;
}
