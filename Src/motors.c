/**
 * @file motors.c
 * @brief Low‑level motion engine for stepper‑driven SCARA joints.
 *
 * @date 2025-06-12
 * @author Dylan Featherson, Tomas Franco, Charith Sunku
 *
 */
//MOT_TIM3_IRQHandler, and MotorsMove
/* ===========================================================================
 * motors.c – SCARA plotter arm motion control  (rev‑b 2025‑06‑08)
 * ------------------------------------------------------------------------- */

#include "joint_inputs.h"
#include "motors.h"
#include "pins.h"
#include "us_sensor.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>

/* ----------------------------- config ------------------------------------ */
#define STEPS_PER_DEG1       (200.0f * 32.0f / 360.0f)
#define STEPS_PER_DEG2       (200.0f * 32.0f / 360.0f)
#define MAX_Z_MM             5.0f

#define DIR_TO_HOME1         GPIO_PIN_SET
#define DIR_TO_HOME2         GPIO_PIN_SET

#define HOMING_FREQ_HZ       4000U
#define DRAW_FREQ_HZ         2000U
#define HOMING_TIMEOUT_MS    3000U
#define DRAW_TIMEOUT_MS      3000U
#define SWITCH_DEBOUNCE_MS   5U
#define SWITCH_STABLE_CNT    3U
#define PEN_UP				 2000
#define PEN_DOWN		     1000

#define JOINT2_ZERO_DEG   0.00f
#define DIR2_EXTEND   GPIO_PIN_RESET
#define DIR2_RETRACT  GPIO_PIN_SET

/* 0 = driver‑ENABLE active‑LOW (DRV8825/A4988), 1 = active‑HIGH          */
#ifndef EN_ACTIVE_HIGH
#define EN_ACTIVE_HIGH       0
#endif

/* -------------------------- private state -------------------------------- */
extern TIM_HandleTypeDef htim1;   /* TIM1 → servo PWM */
extern TIM_HandleTypeDef htim3;   /* TIM3 → step pulses */

static volatile int32_t steps_remaining1 = 0;
static volatile int32_t steps_remaining2 = 0;
static volatile uint32_t toggle_state1    = 0;
static volatile uint32_t toggle_state2    = 0;
static volatile float   prev_theta1      = 0.0f;
static volatile float   prev_theta2      = 0.0f;
static volatile bool    homing_error     = false;

/* --------------------------- helpers ------------------------------------- */
/* TIM3 clock after prescaler (see MX_TIM3_Init → prescaler = 83 → 1 MHz) */
#define TIM3_CLOCK_HZ  1000000U

/* ───── Configurable knobs ──────────────────────────────────────────── */
#define MAX_PLAN_POINTS    5000     /* hard ceiling on way‑points        */
#define SPLINE_MAX_SUBDIV   48      /* upper cap on subdivisions/segment   */
#define SPLINE_RES_DEG      0.05f    /* max ∆θ (deg) between samples        */
#define SPLINE_TENSION 0.75f        
/**
 * @brief  Returns TRUE if limit switch pressed
 * @param pin Limit Switch Pin
 * @param port Limit Switch Port
 *  
 */

static inline bool rawSwitchPressed(uint16_t pin, GPIO_TypeDef *port)
{ return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET; }
/**
 * @brief  Prevents repeated error of joint bouncing on limit switch 
 * @param pin Limit Switch Pin
 * @param port Limit Switch Port
 *  
 */

static bool debouncedSwitchPressed(uint16_t pin, GPIO_TypeDef *port)
{
    uint8_t ok = 0;
    while (ok < SWITCH_STABLE_CNT) {
        if (rawSwitchPressed(pin, port)) ++ok; else ok = 0;
        HAL_Delay(SWITCH_DEBOUNCE_MS);
    }
    return true;
}
/**
 * @brief  Sets frequency of Timer 3 Channel 1 and 2
 * @param freq_hz Desired frequency
 *  
 */

static void tim3SetFreq(uint32_t freq_hz)
{
    /* full_pulse_ticks = ticks for one complete HIGH→LOW pulse */
    uint32_t full_pulse_ticks = TIM3_CLOCK_HZ / freq_hz;        /* e.g. 1 MHz/4 k = 250 */
    if (full_pulse_ticks < 4) full_pulse_ticks = 4;             /* avoid ARR underflow */

    uint32_t arr = full_pulse_ticks / 2U - 1U;                  /* toggle every full_pulse/2 */
    uint32_t ccr = (arr + 1U) / 2U;                             /* 50 % duty */

    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
}

/* ------------------------- driver enable --------------------------------- */
/**
 * @brief  Enable outputs on both DRV8825 stepper motor drivers
 *  
 */
inline void motorsEnable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}
/**
 * @brief  Disable outputs on both DRV8825 stepper motor drivers
 *  
 */
void motorsDisable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

/* --------------------------- homing -------------------------------------- */
/**
 * @brief  Actuates both stepper motors until robot links engage limit switches
 *  
 */
bool motorsHome(void)
{
    homing_error = false;
    motorsEnable();
    moveServoPulse(PEN_UP);
    HAL_Delay(150);

    /* switches released? */
    uint32_t t0 = HAL_GetTick();
    while (rawSwitchPressed(LIMIT1_Pin, LIMIT1_GPIO_Port) ||
           rawSwitchPressed(LIMIT2_Pin, LIMIT2_GPIO_Port)) {
        if (HAL_GetTick() - t0 > HOMING_TIMEOUT_MS) { homing_error = true; return false; }
        HAL_Delay(10);
    }

    /* set direction */
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, DIR_TO_HOME1);
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, DIR_TO_HOME2);

    /* timer @ homing speed */
    tim3SetFreq(HOMING_FREQ_HZ);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* enable CC IRQs *before* starting */
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1 | TIM_IT_CC2);
    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&htim3);

    bool ch1_done = false, ch2_done = false;
    t0 = HAL_GetTick();
    while (!(ch1_done && ch2_done)) {
        if (!ch1_done && rawSwitchPressed(LIMIT1_Pin, LIMIT1_GPIO_Port) &&
            debouncedSwitchPressed(LIMIT1_Pin, LIMIT1_GPIO_Port)) {
            ch1_done = true;
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
            HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
        }
        if (!ch2_done && rawSwitchPressed(LIMIT2_Pin, LIMIT2_GPIO_Port) &&
            debouncedSwitchPressed(LIMIT2_Pin, LIMIT2_GPIO_Port)) {
            ch2_done = true;
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
            HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_2);
            HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
        }
        if (HAL_GetTick() - t0 > HOMING_TIMEOUT_MS) { homing_error = true; break; }
    }

    HAL_TIM_Base_Stop(&htim3);
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);

    prev_theta1 = 0;
    prev_theta2 = 0;

    return !homing_error;
}

/* ------------------------ Z-servo helper --------------------------------- */
/**
 * @brief  Actuates servo motor to move prismatic end effector 
 * @param us 
 *  
 */
void moveServoPulse(uint32_t us)
{
    if (us < 1000)      us = 1000;
    else if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
}

/* ---------------------------- drawing ------------------------------------ */
/**
 * @brief  Enables Timer 3 channels 1 and 2 to start output compare on STEP pin of motor drivers
 *  
 */
static void tim3StartDraw(void)
{
    tim3SetFreq(DRAW_FREQ_HZ);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1 | TIM_IT_CC2);
    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&htim3);
}
/**
 * @brief  Actuates Joint 1, Joint 2 and end effector to prescribed positions.
 * @param theta1 Required joint angle for Joint 1
 * @param theta2 Required joint angle for Joint 2
 * @param curr_z Required z position of end effector
 *  
 */

void newsendJointAngles(float theta1, float theta2, int curr_z)
{
    /* --- 0) Z-axis --- */
    moveServoPulse(curr_z ? PEN_UP : PEN_DOWN);
    static float err1 = 0.0f, err2 = 0.0f;          

    float d1 = theta1 - prev_theta1 + err1;          
    float steps1_f = d1 * STEPS_PER_DEG1;
    int32_t target1 = (int32_t)llroundf(steps1_f);   
    err1 = (steps1_f - target1) / STEPS_PER_DEG1;    
    const float mech_now  = theta2 + JOINT2_ZERO_DEG;
    const float mech_prev = prev_theta2 + JOINT2_ZERO_DEG;
    float d2 = mech_now - mech_prev + err2;
    float steps2_f = d2 * STEPS_PER_DEG2;
    int32_t target2 = (int32_t)llroundf(steps2_f);
    err2 = (steps2_f - target2) / STEPS_PER_DEG2;

    /* direction pins ------------------------------------------------ */
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (target1 >= 0) ? GPIO_PIN_SET  : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (target2 >= 0) ? DIR2_RETRACT  : DIR2_EXTEND);

    /* absolute step counts ------------------------------------------ */
    steps_remaining1 = (target1 >= 0) ?  target1 : -target1;
    steps_remaining2 = (target2 >= 0) ?  target2 : -target2;
    toggle_state1 = toggle_state2 = 0;
	HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);

	if (steps_remaining1 == 0 && steps_remaining2 ==0) {
		prev_theta1 = theta1;
		prev_theta2 = theta2;
		err1 = err2 = 0.0f;
		return;
	}
	tim3StartDraw();

	uint32_t t0 = HAL_GetTick();
	while (steps_remaining1 || steps_remaining2) {
		if (HAL_GetTick() - t0 > DRAW_TIMEOUT_MS) {
			homing_error = true; /* reuse flag */
			break;
		}
		__WFI(); /* sleep until next IRQ */
	}
	while (obstruction()) {
		__WFI(); /* sleep until ultrasonic not detecting*/
	}
//	HAL_Delay(100);
	prev_theta1 = theta1;
	prev_theta2 = theta2;
}

/* Cubic Hermite basis (Catmull‑Rom) */
/**
 * @brief  Implements Cubic Hermite Spline to smooth out draw path
 * @param p0 Starting Point
 * @param p1 Ending Point
 * @param m0 Starting Tangent
 * @param m1 Ending Tangent
 * @param t Time Step
 *  
 */
static inline float hermite(float p0, float p1, float m0, float m1, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;
    float h00 =  2.0f*t3 - 3.0f*t2 + 1.0f;
    float h10 =        t3 - 2.0f*t2 + t;
    float h01 = -2.0f*t3 + 3.0f*t2;
    float h11 =        t3 -       t2;
    return h00*p0 + h10*m0 + h01*p1 + h11*m1;
}
/**
 * @brief  Outputs hypotenuse length given side lengths
 * @param a Side A
 * @param b Side B
 *  
 */

static inline float hypot2f(float a, float b) { return sqrtf(a * a + b * b); }

/* -------------------------------------------------------------------
 * buildTangentsTension – centripetal Catmull‑Rom with global tension
 * -------------------------------------------------------------------*/
/**
 * @brief  Builds tangents for each segment of draw plan
 * @param p Array of joint anlges for full drawing
 * @param n Size of p
 * @param t1_out Tangents for Joint 1
 * @param t2_out Tangents for Joint 2
 *  
 */
static void buildTangents(const GCodeMove *p, size_t n,
                          float *t1_out, float *t2_out)
{
    const float k = 0.5f * (1.0f - SPLINE_TENSION);   /* 0.5 .. 0 */

    for (size_t i = 0; i < n; ++i) {
        size_t ip = (i == 0)   ? 0    : i - 1;
        size_t in = (i == n-1) ? n-1 : i + 1;

        float d_prev = hypot2f(p[i].theta1_deg - p[ip].theta1_deg,
                               p[i].theta2_deg - p[ip].theta2_deg);
        float d_next = hypot2f(p[in].theta1_deg - p[i].theta1_deg,
                               p[in].theta2_deg - p[i].theta2_deg);
        float denom  = d_prev + d_next;

        if (denom < 1e-6f) {
            t1_out[i] = 0.0f;
            t2_out[i] = 0.0f;
        } else {
            /* centripetal weight scaled by global tension (k) */
            float w = k * d_next / denom;            /* 0 .. 0.5 */
            t1_out[i] = w * (p[in].theta1_deg - p[ip].theta1_deg);
            t2_out[i] = w * (p[in].theta2_deg - p[ip].theta2_deg);
        }
    }
}

/* ---------------------------------------------------------------------
 * motorsRunSplinePlan – high‑level wrapper called from main.c
 * -------------------------------------------------------------------*/
/**
 * @brief  Implements Catmull-Rom spline pathing for given draw plan
 * @param plan Array of joint anlges for full drawing
 * @param n Size of plan
 *  
 */
bool motorsRunSplinePlan(const GCodeMove *plan, size_t n)
{
    if (!plan || n < 2 || n > MAX_PLAN_POINTS) return false;

    /* static buffers – no heap ‑‑----------------------------------- */
    static float m1[MAX_PLAN_POINTS];
    static float m2[MAX_PLAN_POINTS];

    buildTangents(plan, n, m1, m2);

    bool first_pt = true;                  /* avoid duplicate anchors */
    for (size_t i = 0; i < n - 1; ++i) {
        const GCodeMove *P0 = &plan[i];
        const GCodeMove *P1 = &plan[i + 1];

        float dtheta = fmaxf(fabsf(P1->theta1_deg - P0->theta1_deg),
                         fabsf(P1->theta2_deg - P0->theta2_deg));
        uint32_t sub = (uint32_t)ceilf(dtheta / SPLINE_RES_DEG);
        if (sub < 1)                 sub = 1;
        if (sub > SPLINE_MAX_SUBDIV) sub = SPLINE_MAX_SUBDIV;

        uint8_t start_s = first_pt ? 0 : 1;   /* skip dup anchor */
        first_pt = false;

        for (uint8_t s = start_s; s <= sub; ++s) {
            float t = (float)s / sub;         /* 0‑1 inclusive     */
            float theta1 = hermite(P0->theta1_deg, P1->theta1_deg,
                               m1[i], m1[i + 1], t);
            float theta2 = hermite(P0->theta2_deg, P1->theta2_deg,
                               m2[i], m2[i + 1], t);
            int   z  = (t < 0.5f) ? P0->z_mm : P1->z_mm;
            newsendJointAngles(theta1, theta2, z);
        }
    }
    return true;
}
/**
 * @brief  Checks if steps_remaining == 0 for reach stepper motor. If steps_remaining == 0 for either stepper motor, stops timer channel to shut off actuation.
 *  
 */


void MOT_TIM3_IRQHandler(void)
{
    uint32_t sr = htim3.Instance->SR;
    htim3.Instance->SR = ~(TIM_FLAG_CC1 | TIM_FLAG_CC2);   /* clear both */

    /* ── CH-1 ───────────────────────────────────────────── */
    if (sr & TIM_FLAG_CC1) {
        toggle_state1 ^= 1U;

        if (!toggle_state1 && steps_remaining1) {         /* count on LOW edge */
            if (--steps_remaining1 == 0) {
                __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
                HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);   /* stop toggle NOW   */
                HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
            }
        }
    }

    /* ── CH-2 ───────────────────────────────────────────── */
    if (sr & TIM_FLAG_CC2) {
        toggle_state2 ^= 1U;

        if (!toggle_state2 && steps_remaining2) {
            if (--steps_remaining2 == 0) {
                __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
                HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_2);
                HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
            }
        }
    }
}

