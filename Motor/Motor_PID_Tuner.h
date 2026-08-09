#ifndef __MOTOR_PID_TUNER_H
#define __MOTOR_PID_TUNER_H

#include <stdint.h>

/*
 * Adapter for the PC-side llm-pid-tuner generic CSV protocol.
 *
 * First version: tune only the Iq current PI controller.
 * Id-loop gains remain unchanged.
 */

#ifndef MOTOR_PID_TUNER_TX_PERIOD_MS
#define MOTOR_PID_TUNER_TX_PERIOD_MS  10U
#endif

/* Conservative first-test limits around the existing Iq gains. */
#ifndef MOTOR_PID_TUNER_KP_MIN
#define MOTOR_PID_TUNER_KP_MIN        1.037f
#endif
#ifndef MOTOR_PID_TUNER_KP_MAX
#define MOTOR_PID_TUNER_KP_MAX        2.073f
#endif
#ifndef MOTOR_PID_TUNER_KI_MIN
#define MOTOR_PID_TUNER_KI_MIN        0.064f
#endif
#ifndef MOTOR_PID_TUNER_KI_MAX
#define MOTOR_PID_TUNER_KI_MAX        0.186f
#endif

/* D is disabled for the first hardware-tuning version. */
#define MOTOR_PID_TUNER_KD_MIN        0.0f
#define MOTOR_PID_TUNER_KD_MAX        0.0f

/* Maximum absolute Iq reference accepted from SETPOINT. */
#ifndef MOTOR_PID_TUNER_IQ_REF_MAX_A
#define MOTOR_PID_TUNER_IQ_REF_MAX_A  1.0f
#endif

void Motor_PID_Tuner_Init(void);
void Motor_PID_Tuner_Task(void);
void Motor_PID_Tuner_SendTelemetry(void);

#endif
