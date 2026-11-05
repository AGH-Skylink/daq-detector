/*
 * Algorithm C: broadband MUSIC on the small triangle M1-M3 (azimuth only, speaker count from MDL).
 * Port of daq-sim/daq_sim/music.py (the specification is in its docstring). Called by dsp.c for EVERY frame:
 * the covariance decays also without voice; a result is produced only for voice frames.
 *
 * Covariances, eigen-decomposition (complex Jacobi) and the pseudospectrum are computed in double; the steering
 * table [band][az][mic] is float in PSRAM, rebuilt when the geometry or the speed of sound changes.
 */
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "config.h"
#include "daq.h"

#define MU_N_MICS        3
#define MU_N_BANDS       40
#define AVG_FRAMES    16
#define LAMBDA        (1.0 - 1.0 / AVG_FRAMES)
#define MIN_WEIGHT    4.0
#define MDL_PENALTY   1.0
#define MU_N_AZ          360
#define MU_BAND_LO_HZ    300.0
#define MU_BAND_HI_HZ    2800.0
#define DEN_MIN       1e-6

typedef struct {
    double re[MU_N_MICS][MU_N_MICS], im[MU_N_MICS][MU_N_MICS];
} cmat_t;

static cmat_t   s_R[MU_N_BANDS];
static double   s_w;
static int      s_bin[MU_N_BANDS];              /* FFT bin of each band */
static double   s_freq[MU_N_BANDS];
static float   *s_steer;                     /* [band][az][mic][re, im], PSRAM */
static uint32_t s_version = UINT32_MAX;
static double   s_spec[MU_N_AZ];

static bool build_tables(void)
{
    if (!s_steer) {
        s_steer = heap_caps_malloc(sizeof(float) * 2 * MU_N_BANDS * MU_N_AZ * MU_N_MICS, MALLOC_CAP_SPIRAM);
        if (!s_steer) {
            return false;
        }
    }
    int kc_lo = (int)ceil(MU_BAND_LO_HZ * DSP_FRAME_LEN / DAQ_SAMPLE_RATE_HZ);
    int kc_hi = (int)floor(MU_BAND_HI_HZ * DSP_FRAME_LEN / DAQ_SAMPLE_RATE_HZ);
    int n = kc_hi - kc_lo + 1;
    double c = g_settings.speed_of_sound_mps;
    for (int b = 0; b < MU_N_BANDS; b++) {
        s_bin[b] = kc_lo + (b * (n - 1) + (MU_N_BANDS - 1) / 2) / (MU_N_BANDS - 1);
        s_freq[b] = (double)s_bin[b] * DAQ_SAMPLE_RATE_HZ / DSP_FRAME_LEN;
        for (int a = 0; a < MU_N_AZ; a++) {
            double ux = cos(a * M_PI / 180.0), uy = sin(a * M_PI / 180.0);
            for (int m = 0; m < MU_N_MICS; m++) {
                double proj = (ux * g_settings.mic_xyz_mm[m][0] + uy * g_settings.mic_xyz_mm[m][1]) / 1000.0;
                double ph = 2.0 * M_PI * s_freq[b] * proj / c;
                float *p = &s_steer[((b * MU_N_AZ + a) * MU_N_MICS + m) * 2];
                p[0] = (float)cos(ph);
                p[1] = (float)sin(ph);
            }
        }
    }
    s_version = g_daq.settings_version;
    return true;
}

/* Hermitian 3 x 3 eigen-decomposition (cyclic complex Jacobi). Eigenvalues ascending in w, eigenvectors in columns of V. */
static void eig3(const cmat_t *A0, double w[MU_N_MICS], double vre[MU_N_MICS][MU_N_MICS], double vim[MU_N_MICS][MU_N_MICS])
{
    double ar[MU_N_MICS][MU_N_MICS], ai[MU_N_MICS][MU_N_MICS];
    memcpy(ar, A0->re, sizeof(ar));
    memcpy(ai, A0->im, sizeof(ai));
    for (int i = 0; i < MU_N_MICS; i++) {
        for (int j = 0; j < MU_N_MICS; j++) {
            vre[i][j] = i == j ? 1.0 : 0.0;
            vim[i][j] = 0.0;
        }
    }
    for (int sweep = 0; sweep < 20; sweep++) {
        double off = 0.0, diag = 0.0;
        for (int i = 0; i < MU_N_MICS; i++) {
            for (int j = 0; j < MU_N_MICS; j++) {
                double v = ar[i][j] * ar[i][j] + ai[i][j] * ai[i][j];
                if (i == j) {
                    diag += v;
                } else {
                    off += v;
                }
            }
        }
        if (off <= 1e-30 * (diag > 1e-300 ? diag : 1e-300)) {
            break;
        }
        for (int p = 0; p < MU_N_MICS - 1; p++) {
            for (int q = p + 1; q < MU_N_MICS; q++) {
                double r = hypot(ar[p][q], ai[p][q]);
                if (r == 0.0) {
                    continue;
                }
                double er = ar[p][q] / r, ei = ai[p][q] / r;
                double th = 0.5 * atan2(2.0 * r, ar[q][q] - ar[p][p]);
                double c = cos(th), s = sin(th);
                /* J: J[p][p] = c, J[q][q] = c, J[p][q] = s e, J[q][p] = -s conj(e); A = J^H A J, V = V J */
                double jr[MU_N_MICS][MU_N_MICS] = { { 0 } }, ji[MU_N_MICS][MU_N_MICS] = { { 0 } };
                for (int i = 0; i < MU_N_MICS; i++) {
                    jr[i][i] = 1.0;
                }
                jr[p][p] = c;
                jr[q][q] = c;
                jr[p][q] = s * er;
                ji[p][q] = s * ei;
                jr[q][p] = -s * er;
                ji[q][p] = s * ei;
                double tr[MU_N_MICS][MU_N_MICS], ti[MU_N_MICS][MU_N_MICS];
                for (int i = 0; i < MU_N_MICS; i++) {          /* T = A J */
                    for (int j = 0; j < MU_N_MICS; j++) {
                        double sr = 0.0, si = 0.0;
                        for (int k = 0; k < MU_N_MICS; k++) {
                            sr += ar[i][k] * jr[k][j] - ai[i][k] * ji[k][j];
                            si += ar[i][k] * ji[k][j] + ai[i][k] * jr[k][j];
                        }
                        tr[i][j] = sr;
                        ti[i][j] = si;
                    }
                }
                for (int i = 0; i < MU_N_MICS; i++) {          /* A = J^H T */
                    for (int j = 0; j < MU_N_MICS; j++) {
                        double sr = 0.0, si = 0.0;
                        for (int k = 0; k < MU_N_MICS; k++) {
                            sr += jr[k][i] * tr[k][j] + ji[k][i] * ti[k][j];
                            si += jr[k][i] * ti[k][j] - ji[k][i] * tr[k][j];
                        }
                        ar[i][j] = sr;
                        ai[i][j] = si;
                    }
                }
                for (int i = 0; i < MU_N_MICS; i++) {          /* V = V J */
                    double rr[MU_N_MICS], ri[MU_N_MICS];
                    for (int j = 0; j < MU_N_MICS; j++) {
                        double sr = 0.0, si = 0.0;
                        for (int k = 0; k < MU_N_MICS; k++) {
                            sr += vre[i][k] * jr[k][j] - vim[i][k] * ji[k][j];
                            si += vre[i][k] * ji[k][j] + vim[i][k] * jr[k][j];
                        }
                        rr[j] = sr;
                        ri[j] = si;
                    }
                    memcpy(vre[i], rr, sizeof(rr));
                    memcpy(vim[i], ri, sizeof(ri));
                }
            }
        }
    }
    /* sort ascending (eigenvalue order of numpy eigh) */
    int idx[MU_N_MICS] = { 0, 1, 2 };
    for (int i = 1; i < MU_N_MICS; i++) {
        for (int j = i; j > 0 && ar[idx[j]][idx[j]] < ar[idx[j - 1]][idx[j - 1]]; j--) {
            int t = idx[j];
            idx[j] = idx[j - 1];
            idx[j - 1] = t;
        }
    }
    double sr[MU_N_MICS][MU_N_MICS], si[MU_N_MICS][MU_N_MICS];
    for (int j = 0; j < MU_N_MICS; j++) {
        w[j] = ar[idx[j]][idx[j]];
        for (int i = 0; i < MU_N_MICS; i++) {
            sr[i][j] = vre[i][idx[j]];
            si[i][j] = vim[i][idx[j]];
        }
    }
    memcpy(vre, sr, sizeof(sr));
    memcpy(vim, si, sizeof(si));
}

/* Noise-subspace projector per band: P = En En^H (Hermitian) */
static cmat_t s_P[MU_N_BANDS];

/* d_b(a) = a^H P a / M for a steering vector given by its float table entry or computed values */
static double den(int b, const double *are, const double *aim)
{
    const cmat_t *P = &s_P[b];
    double sum = 0.0;
    for (int i = 0; i < MU_N_MICS; i++) {
        for (int j = 0; j < MU_N_MICS; j++) {
            /* conj(a_i) P_ij a_j, real part */
            double pr = P->re[i][j] * are[j] - P->im[i][j] * aim[j];
            double pi = P->re[i][j] * aim[j] + P->im[i][j] * are[j];
            sum += are[i] * pr + aim[i] * pi;
        }
    }
    return sum / MU_N_MICS;
}

static double mean_den_at(double az_deg)
{
    double ux = cos(az_deg * M_PI / 180.0), uy = sin(az_deg * M_PI / 180.0), c = g_settings.speed_of_sound_mps;
    double total = 0.0;
    for (int b = 0; b < MU_N_BANDS; b++) {
        double are[MU_N_MICS], aim[MU_N_MICS];
        for (int m = 0; m < MU_N_MICS; m++) {
            double proj = (ux * g_settings.mic_xyz_mm[m][0] + uy * g_settings.mic_xyz_mm[m][1]) / 1000.0;
            double ph = 2.0 * M_PI * s_freq[b] * proj / c;
            are[m] = cos(ph);
            aim[m] = sin(ph);
        }
        total += den(b, are, aim);
    }
    return total / MU_N_BANDS;
}

void music_reset(void)
{
    memset(s_R, 0, sizeof(s_R));
    s_w = 0.0;
}

bool localize_c(const dsp_result_t *r, loc_result_t *out)
{
    memset(out, 0, sizeof(*out));
    out->alg = 'C';
    out->kind = '-';
    if (s_version != g_daq.settings_version && !build_tables()) {
        return r->voice;
    }
    for (int b = 0; b < MU_N_BANDS; b++) {
        for (int i = 0; i < MU_N_MICS; i++) {
            for (int j = 0; j < MU_N_MICS; j++) {
                s_R[b].re[i][j] *= LAMBDA;
                s_R[b].im[i][j] *= LAMBDA;
            }
        }
    }
    s_w *= LAMBDA;
    if (!r->voice) {
        return false;
    }

    /* covariance update: whitened snapshot of each band's bin */
    for (int b = 0; b < MU_N_BANDS; b++) {
        int k = s_bin[b] - r->k_lo;
        double xr[MU_N_MICS], xi[MU_N_MICS], e = 0.0;
        for (int m = 0; m < MU_N_MICS; m++) {
            xr[m] = r->xr[m][k];
            xi[m] = r->xi[m][k];
            e += xr[m] * xr[m] + xi[m] * xi[m];
        }
        if (e <= 1e-20) {
            continue;
        }
        double s = 1.0 / sqrt(e);
        for (int m = 0; m < MU_N_MICS; m++) {
            xr[m] *= s;
            xi[m] *= s;
        }
        for (int i = 0; i < MU_N_MICS; i++) {
            for (int j = 0; j < MU_N_MICS; j++) {   /* x_i conj(x_j) */
                s_R[b].re[i][j] += xr[i] * xr[j] + xi[i] * xi[j];
                s_R[b].im[i][j] += xi[i] * xr[j] - xr[i] * xi[j];
            }
        }
    }
    s_w += 1.0;
    if (s_w < MIN_WEIGHT) {
        return true;
    }

    /* eigen-decomposition and MDL, k = 1 or 2 */
    static double w[MU_N_BANDS][MU_N_MICS], vre[MU_N_BANDS][MU_N_MICS][MU_N_MICS], vim[MU_N_BANDS][MU_N_MICS][MU_N_MICS];
    double mdl[3] = { INFINITY, 0.0, 0.0 };
    double logn = log(s_w);
    for (int b = 0; b < MU_N_BANDS; b++) {
        eig3(&s_R[b], w[b], vre[b], vim[b]);
        double lam[MU_N_MICS] = { w[b][2], w[b][1], w[b][0] };   /* descending */
        for (int i = 0; i < MU_N_MICS; i++) {
            if (lam[i] < 1e-12 * lam[0]) {
                lam[i] = 1e-12 * lam[0];
            }
        }
        for (int k = 1; k <= 2; k++) {
            double sl = 0.0, sa = 0.0;
            for (int i = k; i < MU_N_MICS; i++) {
                sl += log(lam[i]);
                sa += lam[i];
            }
            int t = MU_N_MICS - k;
            double ratio = exp(sl / t) / (sa / t);
            mdl[k] += -s_w * t * log(ratio) + MDL_PENALTY * 0.5 * k * (2 * MU_N_MICS - k) * logn;
        }
    }
    int count = mdl[2] < mdl[1] ? 2 : 1;

    /* noise projectors: eigenvectors of the MU_N_MICS - count smallest eigenvalues */
    for (int b = 0; b < MU_N_BANDS; b++) {
        memset(&s_P[b], 0, sizeof(cmat_t));
        for (int e = 0; e < MU_N_MICS - count; e++) {
            for (int i = 0; i < MU_N_MICS; i++) {
                for (int j = 0; j < MU_N_MICS; j++) {   /* v_i conj(v_j) */
                    s_P[b].re[i][j] += vre[b][i][e] * vre[b][j][e] + vim[b][i][e] * vim[b][j][e];
                    s_P[b].im[i][j] += vim[b][i][e] * vre[b][j][e] - vre[b][i][e] * vim[b][j][e];
                }
            }
        }
    }

    /* pseudospectrum on the 1 deg azimuth grid */
    for (int a = 0; a < MU_N_AZ; a++) {
        double sum = 0.0;
        for (int b = 0; b < MU_N_BANDS; b++) {
            const float *p = &s_steer[(b * MU_N_AZ + a) * MU_N_MICS * 2];
            double are[MU_N_MICS] = { p[0], p[2], p[4] }, aim[MU_N_MICS] = { p[1], p[3], p[5] };
            double d = den(b, are, aim);
            sum += 1.0 / (d > DEN_MIN ? d : DEN_MIN);
        }
        s_spec[a] = sum / MU_N_BANDS;
    }

    /* the `count` largest local maxima, parabolic refinement */
    int best[2] = { -1, -1 };
    for (int a = 0; a < MU_N_AZ; a++) {
        double y = s_spec[a];
        if (!(y > s_spec[(a + MU_N_AZ - 1) % MU_N_AZ] && y >= s_spec[(a + 1) % MU_N_AZ])) {
            continue;
        }
        if (best[0] < 0 || y > s_spec[best[0]]) {
            best[1] = best[0];
            best[0] = a;
        } else if (best[1] < 0 || y > s_spec[best[1]]) {
            best[1] = a;
        }
    }
    int n = 0;
    for (int s = 0; s < count && s < 2; s++) {
        int a = best[s];
        if (a < 0) {
            break;
        }
        double ym = s_spec[(a + MU_N_AZ - 1) % MU_N_AZ], y0 = s_spec[a], yp = s_spec[(a + 1) % MU_N_AZ];
        double d = ym - 2.0 * y0 + yp;
        double delta = d != 0.0 ? 0.5 * (ym - yp) / d : 0.0;
        double az = fmod(a + delta, 360.0);
        if (az < 0.0) {
            az += 360.0;
        }
        double conf = 1.0 - mean_den_at(az);
        out->src[n++] = (loc_source_t) {
            .az_deg = (float)az, .el_deg = 0.0f, .dist_m = 0.0f,
            .confidence = (float)(conf < 0.0 ? 0.0 : (conf > 1.0 ? 1.0 : conf)), .kind = 'D',
        };
    }
    out->n_sources = (uint8_t)n;
    out->kind = n ? 'D' : '-';
    return true;
}
