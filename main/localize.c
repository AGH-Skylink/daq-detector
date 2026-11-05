/*
 * Localization (firmware 0.6.0), ports of daq-sim/daq_sim/ls.py and daq-sim/daq_sim/srp.py.
 *
 * Algorithm A (localize_a): least-squares position from the 15 GCC-PHAT time differences
 *   (linear start from M1, Gauss-Newton, rejection of up to 2 pairs above 25 us, ok within 8 m). Decision e: if the
 *   position fit fails, a plane-wave fit gives the direction only (distance unknown).
 *
 * Algorithm B (localize_b): SRP-PHAT on the GCC-PHAT correlations.
 *   1. coarse grid = BLE map grid (az 0..350 step 10, el -85..85 step 10) x 8 distances; power = sum of the
 *      pair correlations at the grid delays (linear interpolation); map = max over distances,
 *   2. up to 2 local maxima (second >= 0.8 x first, >= 40 deg apart, power / 15 >= 0.15),
 *   3. pattern search on the exact correlation (fractional delays) from each maximum,
 *   4. SRP-guided time differences (best correlation value within +-3 samples of the predicted delay) and
 *      the least-squares fit of algorithm A; decision e: position if the fit is ok, otherwise SRP direction only.
 *
 * Every step and constant follows the Python reference; see the docstrings there.
 */
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "config.h"
#include "daq.h"

#define LS_REJECT_US        25.0
#define LS_MAX_REJECT       2
#define LS_GN_ITER          10
#define LS_STEP_STOP_M      1e-5
#define DIR_SLOWNESS_TOL    0.3
#define LS_MAX_RANGE_M      8.0     /* farther = diverged fit, not a result */

#define SRP_N_AZ            DAQ_BLE_MAP_AZ_BINS     /* 36, 10 deg */
#define SRP_N_EL            DAQ_BLE_MAP_EL_BINS     /* 18, -85..85 */
#define SRP_N_DIST          8
#define SRP_MIN_POWER       0.15f
#define SRP_SECOND_RATIO    0.8f
#define SRP_MIN_SEP_DEG     40.0
#define SRP_REFINE_MIN_STEP 1.0
#define SRP_REFINE_ROUNDS   40
#define SRP_GUIDE_WINDOW    3

static const double SRP_DIST_M[SRP_N_DIST] = { 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0 };
static const uint8_t PI_[DSP_NUM_PAIRS] = { 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4 };
static const uint8_t PJ_[DSP_NUM_PAIRS] = { 1, 2, 3, 4, 5, 2, 3, 4, 5, 3, 4, 5, 4, 5, 5 };

static double s_mic[DAQ_NUM_CHANNELS][3];
static double s_c;
static float *s_delay_table;          /* [el][az][dist][pair], samples, PSRAM */
static uint32_t s_table_version = UINT32_MAX;
static float s_power[SRP_N_EL][SRP_N_AZ];

/* ============================== geometry ============================== */

static void load_geometry(void)
{
    for (int k = 0; k < DAQ_NUM_CHANNELS; k++) {
        for (int a = 0; a < 3; a++) {
            s_mic[k][a] = g_settings.mic_xyz_mm[k][a] / 1000.0;
        }
    }
    s_c = g_settings.speed_of_sound_mps;
}

static void from_spherical(double az_deg, double el_deg, double dist, double p[3])
{
    double az = az_deg * M_PI / 180.0, el = el_deg * M_PI / 180.0;
    p[0] = dist * cos(el) * cos(az);
    p[1] = dist * cos(el) * sin(az);
    p[2] = dist * sin(el);
}

static void to_spherical(const double p[3], float *az, float *el, float *dist)
{
    *az = (float)fmod(atan2(p[1], p[0]) * 180.0 / M_PI + 360.0, 360.0);
    *el = (float)(atan2(p[2], hypot(p[0], p[1])) * 180.0 / M_PI);
    *dist = (float)sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

/* Delays (samples) of all pairs for a point */
static void delays(const double p[3], double out[DSP_NUM_PAIRS])
{
    double d[DAQ_NUM_CHANNELS];
    for (int k = 0; k < DAQ_NUM_CHANNELS; k++) {
        double v0 = p[0] - s_mic[k][0], v1 = p[1] - s_mic[k][1], v2 = p[2] - s_mic[k][2];
        d[k] = sqrt(v0 * v0 + v1 * v1 + v2 * v2);
    }
    for (int q = 0; q < DSP_NUM_PAIRS; q++) {
        out[q] = (d[PI_[q]] - d[PJ_[q]]) / s_c * DAQ_SAMPLE_RATE_HZ;
    }
}

/* ============================== linear algebra ============================== */

static bool solve(int n, double a[4][4], double b[4], double x[4])
{
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++) {
            if (fabs(a[r][c]) > fabs(a[p][c])) {
                p = r;
            }
        }
        if (fabs(a[p][c]) < 1e-30) {
            return false;
        }
        if (p != c) {
            for (int k = 0; k < n; k++) {
                double t = a[c][k]; a[c][k] = a[p][k]; a[p][k] = t;
            }
            double t = b[c]; b[c] = b[p]; b[p] = t;
        }
        for (int r = c + 1; r < n; r++) {
            double f = a[r][c] / a[c][c];
            for (int k = c; k < n; k++) {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    for (int r = n - 1; r >= 0; r--) {
        double s = b[r];
        for (int k = r + 1; k < n; k++) {
            s -= a[r][k] * x[k];
        }
        x[r] = s / a[r][r];
    }
    return true;
}

static bool lstsq(int m, int n, double rows[][4], const double *rhs, double x[4])
{
    double ata[4][4] = { 0 }, atb[4] = { 0 };
    for (int r = 0; r < m; r++) {
        for (int i = 0; i < n; i++) {
            atb[i] += rows[r][i] * rhs[r];
            for (int j = 0; j < n; j++) {
                ata[i][j] += rows[r][i] * rows[r][j];
            }
        }
    }
    return solve(n, ata, atb, x);
}

/* ============================== least squares (ls.py) ============================== */

typedef struct {
    double pos[3];
    double rms_us;
    int used[DSP_NUM_PAIRS];
    int n_used;
    bool ok;
} ls_fit_t;

static void distances(const double s[3], double d[DAQ_NUM_CHANNELS], double u[DAQ_NUM_CHANNELS][3])
{
    for (int k = 0; k < DAQ_NUM_CHANNELS; k++) {
        double v[3] = { s[0] - s_mic[k][0], s[1] - s_mic[k][1], s[2] - s_mic[k][2] };
        d[k] = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (d[k] < 1e-6) {
            d[k] = 1e-6;
        }
        for (int a = 0; a < 3; a++) {
            u[k][a] = v[a] / d[k];
        }
    }
}

static void gauss_newton(const double *tau, const int *used, int n_used, double s[3], double *res)
{
    double d[DAQ_NUM_CHANNELS], u[DAQ_NUM_CHANNELS][3], jac[DSP_NUM_PAIRS][4];
    for (int it = 0; it < LS_GN_ITER; it++) {
        distances(s, d, u);
        for (int q = 0; q < n_used; q++) {
            int p = used[q];
            res[q] = tau[p] - (d[PI_[p]] - d[PJ_[p]]) / s_c;
            for (int a = 0; a < 3; a++) {
                jac[q][a] = (u[PI_[p]][a] - u[PJ_[p]][a]) / s_c;
            }
        }
        double step[4];
        if (!lstsq(n_used, 3, jac, res, step)) {
            break;
        }
        for (int a = 0; a < 3; a++) {
            s[a] += step[a];
        }
        if (sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]) < LS_STEP_STOP_M) {
            break;
        }
    }
    distances(s, d, u);
    for (int q = 0; q < n_used; q++) {
        int p = used[q];
        res[q] = tau[p] - (d[PI_[p]] - d[PJ_[p]]) / s_c;
    }
}

static void finish_fit(ls_fit_t *f, const double *res)
{
    double ss = 0.0;
    for (int q = 0; q < f->n_used; q++) {
        ss += res[q] * res[q];
    }
    f->rms_us = sqrt(ss / f->n_used) * 1e6;
}

/* tau in seconds; start == NULL: linear start from M1 */
static ls_fit_t ls_locate(const double *tau, const double *start)
{
    ls_fit_t f = { .n_used = DSP_NUM_PAIRS, .ok = false, .rms_us = INFINITY };
    for (int p = 0; p < DSP_NUM_PAIRS; p++) {
        f.used[p] = p;
    }
    if (start) {
        memcpy(f.pos, start, sizeof(f.pos));
    } else {
        double rows[DAQ_NUM_CHANNELS - 1][4], rhs[DAQ_NUM_CHANNELS - 1], sol[4];
        double m02 = s_mic[0][0] * s_mic[0][0] + s_mic[0][1] * s_mic[0][1] + s_mic[0][2] * s_mic[0][2];
        for (int i = 1; i < DAQ_NUM_CHANNELS; i++) {
            double ri = -s_c * tau[i - 1];   /* pairs (1, i) are p = i - 1 */
            for (int a = 0; a < 3; a++) {
                rows[i - 1][a] = 2.0 * (s_mic[i][a] - s_mic[0][a]);
            }
            rows[i - 1][3] = 2.0 * ri;
            double mi2 = s_mic[i][0] * s_mic[i][0] + s_mic[i][1] * s_mic[i][1] + s_mic[i][2] * s_mic[i][2];
            rhs[i - 1] = mi2 - m02 - ri * ri;
        }
        if (!lstsq(DAQ_NUM_CHANNELS - 1, 4, rows, rhs, sol)) {
            return f;
        }
        memcpy(f.pos, sol, 3 * sizeof(double));
    }
    double res[DSP_NUM_PAIRS];
    for (int round = 0; round <= LS_MAX_REJECT; round++) {
        gauss_newton(tau, f.used, f.n_used, f.pos, res);
        int worst = 0;
        for (int q = 1; q < f.n_used; q++) {
            if (fabs(res[q]) > fabs(res[worst])) {
                worst = q;
            }
        }
        if (fabs(res[worst]) * 1e6 <= LS_REJECT_US || f.n_used <= DSP_NUM_PAIRS - LS_MAX_REJECT) {
            break;
        }
        for (int q = worst; q < f.n_used - 1; q++) {
            f.used[q] = f.used[q + 1];
        }
        f.n_used--;
    }
    finish_fit(&f, res);
    double range = sqrt(f.pos[0] * f.pos[0] + f.pos[1] * f.pos[1] + f.pos[2] * f.pos[2]);
    f.ok = isfinite(range) && f.rms_us <= LS_REJECT_US && range <= LS_MAX_RANGE_M;
    return f;
}

/* Plane-wave fit tau_ij = (m_j - m_i) . v (ls.direction); f.pos = direction vector v */
static ls_fit_t ls_direction(const double *tau)
{
    ls_fit_t f = { .n_used = DSP_NUM_PAIRS, .ok = false };
    double rows[DSP_NUM_PAIRS][4], rhs[DSP_NUM_PAIRS], v[4] = { 0 }, res[DSP_NUM_PAIRS];
    for (int p = 0; p < DSP_NUM_PAIRS; p++) {
        f.used[p] = p;
    }
    for (int round = 0; round <= LS_MAX_REJECT; round++) {
        for (int q = 0; q < f.n_used; q++) {
            int p = f.used[q];
            for (int a = 0; a < 3; a++) {
                rows[q][a] = s_mic[PJ_[p]][a] - s_mic[PI_[p]][a];
            }
            rhs[q] = tau[p];
        }
        if (!lstsq(f.n_used, 3, rows, rhs, v)) {
            return f;
        }
        int worst = 0;
        for (int q = 0; q < f.n_used; q++) {
            res[q] = rhs[q] - (rows[q][0] * v[0] + rows[q][1] * v[1] + rows[q][2] * v[2]);
            if (fabs(res[q]) > fabs(res[worst])) {
                worst = q;
            }
        }
        if (fabs(res[worst]) * 1e6 <= LS_REJECT_US || f.n_used <= DSP_NUM_PAIRS - LS_MAX_REJECT) {
            break;
        }
        for (int q = worst; q < f.n_used - 1; q++) {
            f.used[q] = f.used[q + 1];
        }
        f.n_used--;
    }
    finish_fit(&f, res);
    memcpy(f.pos, v, 3 * sizeof(double));
    double speed_ratio = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) * s_c;
    f.ok = f.rms_us <= LS_REJECT_US && fabs(speed_ratio - 1.0) <= DIR_SLOWNESS_TOL;
    return f;
}

/* Mean peak of the used pairs x (1 - rms / 25 us), 0..1 (ls.confidence) */
static float fit_confidence(const ls_fit_t *f, const float *peak)
{
    if (f->n_used == 0) {
        return 0.0f;
    }
    double sum = 0.0;
    for (int q = 0; q < f->n_used; q++) {
        sum += peak[f->used[q]];
    }
    double c = sum / f->n_used * (1.0 - f->rms_us / LS_REJECT_US);
    return (float)(c < 0.0 ? 0.0 : (c > 1.0 ? 1.0 : c));
}

/* ============================== algorithm A ============================== */

void localize_a(const dsp_result_t *r, loc_result_t *out)
{
    load_geometry();
    memset(out, 0, sizeof(*out));
    out->alg = 'A';
    out->kind = '-';
    double tau[DSP_NUM_PAIRS];
    for (int p = 0; p < DSP_NUM_PAIRS; p++) {
        tau[p] = r->tau_us[p] * 1e-6;
    }
    ls_fit_t f = ls_locate(tau, NULL);
    loc_source_t *s = &out->src[0];
    if (f.ok) {
        to_spherical(f.pos, &s->az_deg, &s->el_deg, &s->dist_m);
        out->kind = 'P';
    } else {
        f = ls_direction(tau);
        if (!f.ok) {
            return;
        }
        float unused;
        to_spherical(f.pos, &s->az_deg, &s->el_deg, &unused);
        s->dist_m = 0.0f;
        out->kind = 'D';
    }
    s->confidence = fit_confidence(&f, r->peak);
    out->n_sources = 1;
    out->has_fit = true;
    out->rms_us = (float)f.rms_us;
    out->n_used = (uint8_t)f.n_used;
}

/* ============================== algorithm B (srp.py) ============================== */

static double grid_el(int j)
{
    return -85.0 + 10.0 * j;
}

static bool build_delay_table(void)
{
    if (!s_delay_table) {
        s_delay_table = heap_caps_malloc(sizeof(float) * SRP_N_EL * SRP_N_AZ * SRP_N_DIST * DSP_NUM_PAIRS, MALLOC_CAP_SPIRAM);
        if (!s_delay_table) {
            return false;
        }
    }
    float *t = s_delay_table;
    for (int j = 0; j < SRP_N_EL; j++) {
        for (int k = 0; k < SRP_N_AZ; k++) {
            for (int d = 0; d < SRP_N_DIST; d++) {
                double p[3], dl[DSP_NUM_PAIRS];
                from_spherical(k * 10.0, grid_el(j), SRP_DIST_M[d], p);
                delays(p, dl);
                for (int q = 0; q < DSP_NUM_PAIRS; q++) {
                    *t++ = (float)dl[q];
                }
            }
        }
    }
    s_table_version = g_daq.settings_version;
    return true;
}

/* Correlation of pair q at a fractional delay, linear interpolation (coarse grid) */
static inline float corr_at(const dsp_result_t *r, int q, float delay)
{
    int lag = r->max_lag[q];
    float x = delay + lag + 1;
    if (x < 0.0f) {
        x = 0.0f;
    } else if (x > 2 * lag + 2) {
        x = (float)(2 * lag + 2);
    }
    int i0 = (int)floorf(x);
    if (i0 > 2 * lag + 1) {
        i0 = 2 * lag + 1;
    }
    float frac = x - i0;
    const float *c = r->corr[q];
    return c[i0] * (1.0f - frac) + c[i0 + 1] * frac;
}

/* Exact power at a point: sum over pairs of Re(sum_k W[k] e^{j 2 pi k tau / N}) / n_band */
static double power_at(const dsp_result_t *r, double az, double el, double dist)
{
    double p[3], dl[DSP_NUM_PAIRS];
    from_spherical(az, el, dist, p);
    delays(p, dl);
    double total = 0.0;
    for (int q = 0; q < DSP_NUM_PAIRS; q++) {
        double phi = 2.0 * M_PI * dl[q] / DSP_FRAME_LEN;
        float cr = (float)cos(phi * r->k_lo), ci = (float)sin(phi * r->k_lo);   /* e^{j phi k}, k = k_lo */
        float sr = (float)cos(phi), si = (float)sin(phi);                       /* rotation by e^{j phi} */
        const float *wr = r->wr[q], *wi = r->wi[q];
        float sum = 0.0f;
        for (int b = 0; b < r->n_band; b++) {
            sum += wr[b] * cr - wi[b] * ci;
            float t = cr * sr - ci * si;
            ci = cr * si + ci * sr;
            cr = t;
        }
        total += (double)sum / r->n_band;
    }
    return total;
}

static void refine(const dsp_result_t *r, double *az, double *el, double *dist, double *best)
{
    *best = -INFINITY;
    for (int d = 0; d < SRP_N_DIST; d++) {   /* start distance: best on the grid (ties: larger distance) */
        double p = power_at(r, *az, *el, SRP_DIST_M[d]);
        if (p >= *best) {
            *best = p;
            *dist = SRP_DIST_M[d];
        }
    }
    double step_a = 5.0, step_e = 5.0, fac = 1.25;
    const double lo = SRP_DIST_M[0], hi = SRP_DIST_M[SRP_N_DIST - 1];
    for (int round = 0; round < SRP_REFINE_ROUNDS; round++) {
        bool moved = false;
        double ca = *az, ce = *el, cd = *dist;
        for (int da = -1; da <= 1; da++) {
            for (int de = -1; de <= 1; de++) {
                for (int dd = -1; dd <= 1; dd++) {
                    if (da == 0 && de == 0 && dd == 0) {
                        continue;
                    }
                    double a = fmod(*az + da * step_a + 360.0, 360.0);
                    double e = fmin(fmax(*el + de * step_e, -90.0), 90.0);
                    double dm = fmin(fmax(*dist * pow(fac, dd), lo), hi);
                    double p = power_at(r, a, e, dm);
                    if (p > *best) {
                        *best = p;
                        ca = a; ce = e; cd = dm;
                        moved = true;
                    }
                }
            }
        }
        if (moved) {
            *az = ca; *el = ce; *dist = cd;
        } else {
            step_a /= 2.0;
            step_e /= 2.0;
            fac = sqrt(fac);
            if (step_a < SRP_REFINE_MIN_STEP) {
                break;
            }
        }
    }
}

/* Best correlation value within +-3 samples of the predicted delays, parabolic refinement */
static void guided_tdoa(const dsp_result_t *r, double az, double el, double dist, double *tau, float *peak)
{
    double p[3], dl[DSP_NUM_PAIRS];
    from_spherical(az, el, dist, p);
    delays(p, dl);
    for (int q = 0; q < DSP_NUM_PAIRS; q++) {
        int lag = r->max_lag[q];
        const float *c = r->corr[q];
        int cen = (int)rint(dl[q]);   /* round half to even, as Python round() */
        int lo = (cen - SRP_GUIDE_WINDOW > -lag ? cen - SRP_GUIDE_WINDOW : -lag) + lag + 1;
        int hi = (cen + SRP_GUIDE_WINDOW < lag ? cen + SRP_GUIDE_WINDOW : lag) + lag + 1;
        if (lo > hi) {
            int cc = cen < -lag ? -lag : (cen > lag ? lag : cen);
            lo = hi = cc + lag + 1;
        }
        int m = lo;
        for (int t = lo + 1; t <= hi; t++) {
            if (c[t] > c[m]) {
                m = t;
            }
        }
        double y0 = c[m], ym = c[m - 1], yp = c[m + 1];
        double den = ym - 2.0 * y0 + yp;
        double delta = den != 0.0 ? 0.5 * (ym - yp) / den : 0.0;
        tau[q] = (m - lag - 1 + delta) / DAQ_SAMPLE_RATE_HZ;
        peak[q] = (float)(y0 - 0.25 * (ym - yp) * delta);
    }
}

static double angular_distance_deg(double az1, double el1, double az2, double el2)
{
    double a1 = az1 * M_PI / 180, e1 = el1 * M_PI / 180, a2 = az2 * M_PI / 180, e2 = el2 * M_PI / 180;
    double c = sin(e1) * sin(e2) + cos(e1) * cos(e2) * cos(a1 - a2);
    return acos(fmin(fmax(c, -1.0), 1.0)) * 180.0 / M_PI;
}

void localize_b(const dsp_result_t *r, loc_result_t *out, uint8_t map[DAQ_BLE_MAP_AZ_BINS * DAQ_BLE_MAP_EL_BINS])
{
    load_geometry();
    memset(out, 0, sizeof(*out));
    out->alg = 'B';
    out->kind = '-';
    if (s_table_version != g_daq.settings_version && !build_delay_table()) {
        return;
    }

    /* 1. coarse grid, map = max over distances */
    const float *t = s_delay_table;
    float top = -INFINITY;
    for (int j = 0; j < SRP_N_EL; j++) {
        for (int k = 0; k < SRP_N_AZ; k++) {
            float best = -INFINITY;
            for (int d = 0; d < SRP_N_DIST; d++, t += DSP_NUM_PAIRS) {
                float pw = 0.0f;
                for (int q = 0; q < DSP_NUM_PAIRS; q++) {
                    pw += corr_at(r, q, t[q]);
                }
                if (pw > best) {
                    best = pw;
                }
            }
            s_power[j][k] = best;
            if (best > top) {
                top = best;
            }
        }
    }
    for (int j = 0; j < SRP_N_EL; j++) {
        for (int k = 0; k < SRP_N_AZ; k++) {
            float v = top > 0.0f ? fmaxf(s_power[j][k], 0.0f) / top * 255.0f : 0.0f;
            map[j * SRP_N_AZ + k] = (uint8_t)floorf(v + 0.5f);
        }
    }

    /* 2. local maxima (8 neighbours, azimuth wraps), strongest first */
    int peak_j[2] = { -1, -1 }, peak_k[2] = { -1, -1 };
    float peak_v[2] = { 0 };
    int n_peaks = 0;
    /* strongest local maximum */
    for (int pass = 0; pass < 2; pass++) {
        float bestv = -INFINITY;
        int bj = -1, bk = -1;
        for (int j = 0; j < SRP_N_EL; j++) {
            for (int k = 0; k < SRP_N_AZ; k++) {
                float v = s_power[j][k];
                bool is_max = true;
                for (int jj = j - 1; jj <= j + 1 && is_max; jj++) {
                    if (jj < 0 || jj >= SRP_N_EL) {
                        continue;
                    }
                    for (int dk = -1; dk <= 1; dk++) {
                        if (jj == j && dk == 0) {
                            continue;
                        }
                        if (!(v > s_power[jj][(k + dk + SRP_N_AZ) % SRP_N_AZ])) {
                            is_max = false;
                            break;
                        }
                    }
                }
                if (!is_max || v <= bestv) {
                    continue;
                }
                if (pass == 0) {
                    bestv = v; bj = j; bk = k;
                } else if (!(j == peak_j[0] && k == peak_k[0]) && v >= SRP_SECOND_RATIO * peak_v[0] &&
                           v / DSP_NUM_PAIRS >= SRP_MIN_POWER &&
                           angular_distance_deg(k * 10.0, grid_el(j), peak_k[0] * 10.0, grid_el(peak_j[0])) >= SRP_MIN_SEP_DEG) {
                    bestv = v; bj = j; bk = k;
                }
            }
        }
        if (bj < 0 || (pass == 0 && bestv / DSP_NUM_PAIRS < SRP_MIN_POWER)) {
            break;
        }
        peak_j[pass] = bj;
        peak_k[pass] = bk;
        peak_v[pass] = bestv;
        n_peaks++;
    }

    /* 3.-4. refinement, guided time differences, least squares (decision e) */
    for (int s = 0; s < n_peaks; s++) {
        double az = peak_k[s] * 10.0, el = grid_el(peak_j[s]), dist = SRP_DIST_M[0], pw;
        refine(r, &az, &el, &dist, &pw);
        double tau[DSP_NUM_PAIRS];
        float peak[DSP_NUM_PAIRS];
        guided_tdoa(r, az, el, dist, tau, peak);
        ls_fit_t f = ls_locate(tau, NULL);
        if (!f.ok) {
            double start[3];
            from_spherical(az, el, dist, start);
            f = ls_locate(tau, start);
        }
        loc_source_t *src = &out->src[s];
        double power = pw / DSP_NUM_PAIRS;
        if (f.ok) {
            to_spherical(f.pos, &src->az_deg, &src->el_deg, &src->dist_m);
            src->confidence = fit_confidence(&f, peak);
            src->kind = 'P';
        } else {
            src->az_deg = (float)az;
            src->el_deg = (float)el;
            src->dist_m = 0.0f;
            src->confidence = (float)(power < 0.0 ? 0.0 : (power > 1.0 ? 1.0 : power));
            src->kind = 'D';
        }
    }
    out->n_sources = (uint8_t)n_peaks;
    if (n_peaks > 0) {
        out->kind = out->src[0].kind;
    }
}
