/*
 * A/B harness: original quantized-jerk ramp vs. closed-form quintic ramp.
 *
 * Both arms run code extracted VERBATIM from the respective stepper.c, in real
 * C float arithmetic, driven through the real velocity-profile setup. Build:
 *
 *   gcc -O2 -DARM_NEW=1 -o harness_new harness2.c -lm
 *   gcc -O2 -DARM_NEW=0 -o harness_old harness2.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define ENABLE_JERK_ACCELERATION 1
#ifndef JERK_LIMIT_SHORT_MOVES
#define JERK_LIMIT_SHORT_MOVES 0
#endif
#include <stdint.h>
typedef uint_fast8_t uf8;
#define ACCELERATION_TICKS_PER_SECOND 400
#define DT_SEGMENT (1.0f / (60.0f * (float)ACCELERATION_TICKS_PER_SECOND))
#define REQ_MM_INCREMENT_SCALAR 1.25f

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define On 1
#define Off 0

typedef enum { Ramp_Accel, Ramp_Cruise, Ramp_Decel, Ramp_DecelOverride } ramp_type_t;
typedef struct { int velocity_profile, decel_override, parking, hold_partial_block, flags; } prep_flags_t;

typedef struct {
    float millimeters, acceleration, max_acceleration, jerk, programmed_rate, rapid_rate;
    float entry_speed_sqr;
    unsigned int step_event_count;
    struct { int jerk, rapid_motion, system_motion, is_rpm_rate_adjusted, is_laser_ppi_mode, units_per_rev; } condition;
} plan_block_t;

static plan_block_t *pl_block;

/* superset of both implementations' prep fields */
static struct {
    prep_flags_t recalculate;
    ramp_type_t ramp_type;
    bool jerk;
    float last_accel, last_time_var;                                  /* original */
    float ramp_t, ramp_tau, ramp_v0, ramp_v1, ramp_mm, ramp_end, ramp_inv_vsum; /* quintic */
    float dt_remainder;
    unsigned int steps_remaining;
    float steps_per_mm, req_mm_increment;
    float mm_complete, current_speed, maximum_speed, exit_speed;
    float accelerate_until, decelerate_after, target_position, target_feed, inv_feedrate;
} prep;

static struct { struct { int execute_hold, execute_sys_motion, update_spindle_rpm; } step_control; } sys;
static bool exec_fast_hold;
#define STATE_HOMING 1
static int state_get (void) { return 0; }
static float g_exit_speed_sqr, g_nominal_speed;
static float plan_get_exec_block_exit_speed_sqr (void) { return g_exit_speed_sqr; }
static float plan_compute_profile_nominal_speed (plan_block_t *b) { (void)b; return g_nominal_speed; }

#if ARM_NEW
#include "harness_ramp.inc"
#endif

#define MAX_SEG 300000
static struct { double t, v, mm, dt; } trace[MAX_SEG];
static int n_trace;

/* --- planner a_eff --- */
static float accel_eff_quintic (float A, float J, float v)
{
    float a = A * (8.0f / 15.0f);
    float aj = sqrtf(J * v * (1.0f / 5.7735026919f));
    return aj < a ? aj : a;
}
/* original trapezoidal-jerk a_eff, verbatim logic from planner.c 577-585 */
static float accel_eff_trapjerk (float A, float J, float v)
{
    float t_to_max = A / J;
    float v_after = 0.5f * J * t_to_max * t_to_max;
    if(0.5f * v > v_after)
        return v / (2.0f * (t_to_max + (0.5f * v - v_after) / A));
    return v / (2.0f * sqrtf(v / J));
}

typedef struct {
    const char *name;
    float mm, v_in, v_nom, v_out, A, J;
    bool hold, jerk_mode;
} testcase_t;

static double g_total_mm;
static double g_analytic_dv;

static int run_case (testcase_t *tc)
{
    plan_block_t block;
    memset(&block, 0, sizeof(block));
    memset(&prep, 0, sizeof(prep));
    memset(&sys, 0, sizeof(sys));
    n_trace = 0;

    block.millimeters = tc->mm;
    block.max_acceleration = tc->A;
    block.jerk = tc->J;
    block.programmed_rate = tc->v_nom;
    block.entry_speed_sqr = tc->v_in * tc->v_in;
    block.condition.jerk = tc->jerk_mode;
#if ARM_NEW
    block.acceleration = tc->jerk_mode ? accel_eff_quintic(tc->A, tc->J, tc->v_nom) : tc->A;
#else
    block.acceleration = tc->jerk_mode ? accel_eff_trapjerk(tc->A, tc->J, tc->v_nom) : tc->A;
#endif
    block.step_event_count = (unsigned int)(tc->mm * 80.0f);

    pl_block = &block;
    prep.jerk = block.condition.jerk;
    prep.steps_per_mm = 80.0f;
    prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.steps_per_mm;
    prep.current_speed = tc->v_in;
    prep.last_time_var = DT_SEGMENT;
    g_exit_speed_sqr = tc->v_out * tc->v_out;
    g_nominal_speed = tc->v_nom;
    sys.step_control.execute_hold = tc->hold;

#if ARM_NEW
#include "harness_profile.inc"
#else
#include "orig_profile.inc"
#endif

    printf("    a_eff=%-9.1f profile ramp=%d accel_until=%.5f decel_after=%.5f max_v=%.2f exit_v=%.2f mm_complete=%.5f\n",
           block.acceleration, (int)prep.ramp_type, prep.accelerate_until, prep.decelerate_after,
           prep.maximum_speed, prep.exit_speed, prep.mm_complete);

    double t_abs = 0.0, mm_done = 0.0;
    int segments = 0;
    g_total_mm = tc->mm - prep.mm_complete;
    trace[0].t = 0.0; trace[0].v = prep.current_speed; trace[0].mm = 0.0; trace[0].dt = 0.0;
    n_trace = 1;

    while (segments < 100000) {
        float mm_before = block.millimeters;
#if ARM_NEW
#include "harness_loop.inc"
#else
#include "orig_loop.inc"
#endif
        t_abs += dt;
        mm_done += (double)(mm_before - mm_remaining);
        segments++;
        if(n_trace < MAX_SEG) {
            trace[n_trace].t = t_abs;
            trace[n_trace].v = prep.current_speed;
            trace[n_trace].mm = mm_done;
            trace[n_trace].dt = dt;
            n_trace++;
        }
        block.millimeters = mm_remaining;        /* block consumption */
        if(mm_remaining <= prep.mm_complete) break;
    }
    return segments;
}

static double max_dv_uniform;

static void analyse (testcase_t *tc)
{
    /* Segment-average step-rate metrics: what the drives actually see.
       Each segment is executed at a constant step rate, so the velocity
       discontinuity between consecutive segments IS the commanded jerk
       impulse. This is the metric that matters for servo ringing. */
    double max_dv = 0.0, peak_a = 0.0, peak_dj = 0.0, prev_a = 0.0, max_dt = 0.0, min_dt = 1e9;
    max_dv_uniform = 0.0;
    for(int i = 1; i < n_trace; i++) {
        double dt = trace[i].dt;
        if(dt <= 1e-12) continue;
        if(dt > max_dt) max_dt = dt;
        if(dt < min_dt) min_dt = dt;
        double dv = trace[i].v - trace[i-1].v;
        if(fabs(dv) > max_dv) max_dv = fabs(dv);
        double a = dv / dt;
        if(fabs(a) > peak_a) peak_a = fabs(a);
        if(i > 1) {
            double dj = fabs(a - prev_a) / dt;
            if(dj > peak_dj) peak_dj = dj;
        }
        prev_a = a;
        if(fabs(dt - (double)DT_SEGMENT) < 1e-12 && fabs(dv) > max_dv_uniform) max_dv_uniform = fabs(dv);
    }
    {   char fn[256];
        snprintf(fn, sizeof(fn), "trace_%s_%s.csv", ARM_NEW ? "new" : "old", tc->name);
        for(char *p = fn; *p; p++) if(*p == ' ' || *p == ':' || *p == '.') *p = '_';
        strcat(fn, ".csv");
        FILE *f = fopen(fn, "w");
        fprintf(f, "t_min,dt_min,v,mm\n");
        for(int i = 0; i < n_trace; i++)
            fprintf(f, "%.9f,%.9f,%.6f,%.6f\n", trace[i].t, trace[i].dt, trace[i].v, trace[i].mm);
        fclose(f);
    }
    double dist_err = trace[n_trace-1].mm - g_total_mm;
    double v_err = trace[n_trace-1].v - prep.exit_speed;

    printf("    segs=%-5d  peak|a|=%-9.0f (%-5.1f%% of A)   peak|dj| across segs=%-12.0f (%5.2fx J)\n",
           n_trace - 1, peak_a, 100.0 * peak_a / tc->A, peak_dj, peak_dj / tc->J);
    printf("    max inter-segment dv=%-9.3f mm/min   dist err=%+.3e mm   exit v err=%+.3e mm/min   dt range %.4f-%.4f ms\n",
           max_dv, dist_err, v_err, min_dt * 60000.0, max_dt * 60000.0);
    printf("    max dv on uniform 2.5ms segments = %.3f mm/min  (this is the step-rate impulse the drives see)\n", max_dv_uniform);
}

int main (void)
{
    testcase_t cases[] = {
        { "XY F3000 100mm",              100.0f,    0.0f, 3000.0f,    0.0f, 2880000.0f, 6.48e8f, false, true },
        { "XY F3000 3mm triangle",         3.0f,    0.0f, 3000.0f,    0.0f, 2880000.0f, 6.48e8f, false, true },
        { "XY F3000 corner: decel to 2000",20.0f, 3000.0f, 3000.0f, 2000.0f, 2880000.0f, 6.48e8f, false, true },
        { "XY rapid F20000 100mm",       100.0f,    0.0f,20000.0f,    0.0f, 2880000.0f, 6.48e8f, false, true },
        { "XY rapid F20000 1mm hop",       1.0f,    0.0f,20000.0f,    0.0f, 2880000.0f, 6.48e8f, false, true },
        /* arc-chord geometry: short block cruising through, dv ~ 0 -- the case
           the clamp must NOT touch */
        { "arc chord 0.1mm cruise",        0.1f, 2900.0f, 3000.0f, 2900.0f, 2880000.0f, 6.48e8f, false, true },
        { "arc chord 0.05mm cruise",      0.05f, 2900.0f, 3000.0f, 2900.0f, 2880000.0f, 6.48e8f, false, true },
        { "contour 0.5mm cruise",          0.5f, 2500.0f, 3000.0f, 2500.0f, 2880000.0f, 6.48e8f, false, true },
        { "pure cruise 50mm",             50.0f, 3000.0f, 3000.0f, 3000.0f, 2880000.0f, 6.48e8f, false, true },
        { "feed hold from rapid F20000", 400.0f,20000.0f,20000.0f,    0.0f, 2880000.0f, 6.48e8f, true,  true },
        { "Z plunge F500 10mm",           10.0f,    0.0f,  500.0f,    0.0f, 2880000.0f, 6.48e8f, false, true },
        { "jog F3000 (linear path)",     100.0f,    0.0f, 3000.0f,    0.0f, 2880000.0f, 6.48e8f, false, false },
    };

#if ARM_NEW
    printf("########## QUINTIC BEZIER (new) ##########\n\n");
#else
    printf("########## QUANTIZED JERK (original) ##########\n\n");
#endif
    for(size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        printf("[%zu] %s\n", i + 1, cases[i].name);
        run_case(&cases[i]);
        analyse(&cases[i]);
        printf("\n");
    }
    return 0;
}
