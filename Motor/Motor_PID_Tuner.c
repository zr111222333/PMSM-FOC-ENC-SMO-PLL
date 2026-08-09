#include "Motor_PID_Tuner.h"
#include "main.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_PID_TUNER_COMMAND_MAX  96U

static uint32_t s_last_tx_ms = 0U;

static uint8_t Motor_PID_Tuner_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static uint8_t Motor_PID_Tuner_IsSafeToTune(void)
{
    return (uint8_t)((Motor_GetState() == motor_state_encoder_run) &&
                     (g_protect.Fault == Fault_NONE));
}

static uint8_t Motor_PID_Tuner_ParseValue(const char *command,
                                          const char *label,
                                          float *value)
{
    const char *start;
    char *end;
    float parsed;

    start = strstr(command, label);
    if (start == NULL)
    {
        return 0U;
    }

    start += strlen(label);
    parsed = strtof(start, &end);
    if ((end == start) || !Motor_PID_Tuner_IsFinite(parsed))
    {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

/*
 * The existing USART driver marks CRLF as complete. The PC tool sends LF.
 * This adapter accepts both forms without changing the original USART files.
 */
static uint8_t Motor_PID_Tuner_TryReadCommand(char *command,
                                               uint16_t command_size)
{
    uint16_t state;
    uint16_t length;
    uint8_t complete;

    if ((command == NULL) || (command_size < 2U))
    {
        return 0U;
    }

    HAL_NVIC_DisableIRQ(USART_UX_IRQn);

    state = g_usart_rx_sta;
    length = (uint16_t)(state & 0x3FFFU);
    complete = (uint8_t)((state & 0x8000U) != 0U);

    if ((!complete) && (length > 0U))
    {
        complete = (uint8_t)(g_usart_rx_buf[length - 1U] == '\n');
    }

    if (!complete)
    {
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);
        return 0U;
    }

    if (length >= command_size)
    {
        length = command_size - 1U;
    }

    memcpy(command, g_usart_rx_buf, length);
    command[length] = '\0';
    g_usart_rx_sta = 0U;

    HAL_NVIC_EnableIRQ(USART_UX_IRQn);

    while ((length > 0U) &&
           ((command[length - 1U] == '\r') ||
            (command[length - 1U] == '\n') ||
            (command[length - 1U] == ' ') ||
            (command[length - 1U] == '\t')))
    {
        command[--length] = '\0';
    }

    return (uint8_t)(length > 0U);
}

static uint8_t Motor_PID_Tuner_ApplyIqPID(float kp, float ki, float kd)
{
    uint32_t primask;

    if (!Motor_PID_Tuner_IsSafeToTune())
    {
        printf("# ERROR: PID update rejected; motor is not in safe encoder-run state\r\n");
        return 0U;
    }

    if ((kp < MOTOR_PID_TUNER_KP_MIN) ||
        (kp > MOTOR_PID_TUNER_KP_MAX) ||
        (ki < MOTOR_PID_TUNER_KI_MIN) ||
        (ki > MOTOR_PID_TUNER_KI_MAX) ||
        (kd < MOTOR_PID_TUNER_KD_MIN) ||
        (kd > MOTOR_PID_TUNER_KD_MAX))
    {
        printf("# ERROR: PID rejected by motor-side safety limits\r\n");
        return 0U;
    }

    /* Apply the three gains and clear history as one short atomic operation. */
    primask = __get_PRIMASK();
    __disable_irq();

    g_current_pid_Iq.Kp = kp;
    g_current_pid_Iq.Ki = ki;
    g_current_pid_Iq.Kd = kd;
    g_current_pid_Iq.Error = 0.0f;
    g_current_pid_Iq.Error_Sum = 0.0f;
    g_current_pid_Iq.Error_Last = 0.0f;
    g_current_pid_Iq.Output_raw = 0.0f;
    g_current_pid_Iq.Output = 0.0f;

    if (primask == 0U)
    {
        __enable_irq();
    }

    printf("# PID Updated: P=%.6f I=%.6f D=%.6f\r\n", kp, ki, kd);
    return 1U;
}

static void Motor_PID_Tuner_SetIqReference(float iq_ref)
{
    uint32_t primask;

    if (!Motor_PID_Tuner_IsSafeToTune())
    {
        printf("# ERROR: SETPOINT rejected; motor is not in safe encoder-run state\r\n");
        return;
    }

    if (!Motor_PID_Tuner_IsFinite(iq_ref) ||
        (iq_ref < -MOTOR_PID_TUNER_IQ_REF_MAX_A) ||
        (iq_ref > MOTOR_PID_TUNER_IQ_REF_MAX_A))
    {
        printf("# ERROR: SETPOINT rejected by Iq safety limit\r\n");
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    foc_input.Iq_ref = iq_ref;
    g_current_pid_Iq.Error_Sum = 0.0f;
    g_current_pid_Iq.Error_Last = 0.0f;
    if (primask == 0U)
    {
        __enable_irq();
    }

    printf("# Setpoint Updated: %.6f\r\n", iq_ref);
}

static void Motor_PID_Tuner_StopMotor(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    foc_input.Id_ref = 0.0f;
    foc_input.Iq_ref = 0.0f;
    if (primask == 0U)
    {
        __enable_irq();
    }

    PMSM_PWM_Duty_Set(0U, 0U, 0U);
    PMSM_PWM_Stop();
    Motor_SetState(motor_state_idle);
    printf("# Motor stopped by PID tuner\r\n");
}

static void Motor_PID_Tuner_PrintStatus(void)
{
    printf("# STATUS: LOOP=IQ P=%.6f I=%.6f D=%.6f "
           "IQ_REF=%.6f IQ=%.6f STATE=%d FAULT=%d\r\n",
           g_current_pid_Iq.Kp,
           g_current_pid_Iq.Ki,
           g_current_pid_Iq.Kd,
           foc_input.Iq_ref,
           I_dq.Iq,
           (int)Motor_GetState(),
           (int)g_protect.Fault);
}

static void Motor_PID_Tuner_ProcessCommand(const char *command)
{
    float kp;
    float ki;
    float kd;
    float setpoint;

    if (strcmp(command, "STATUS") == 0)
    {
        Motor_PID_Tuner_PrintStatus();
        return;
    }

    if ((strcmp(command, "STOP") == 0) ||
        (strcmp(command, "RESET") == 0))
    {
        Motor_PID_Tuner_StopMotor();
        return;
    }

    if (strncmp(command, "SETPOINT:", 9U) == 0)
    {
        if (Motor_PID_Tuner_ParseValue(command, "SETPOINT:", &setpoint))
        {
            Motor_PID_Tuner_SetIqReference(setpoint);
        }
        else
        {
            printf("# ERROR: invalid SETPOINT command\r\n");
        }
        return;
    }

    if (strncmp(command, "SET ", 4U) == 0)
    {
        if (Motor_PID_Tuner_ParseValue(command, "P:", &kp) &&
            Motor_PID_Tuner_ParseValue(command, "I:", &ki) &&
            Motor_PID_Tuner_ParseValue(command, "D:", &kd))
        {
            (void)Motor_PID_Tuner_ApplyIqPID(kp, ki, kd);
        }
        else
        {
            printf("# ERROR: expected SET P:value I:value D:value\r\n");
        }
        return;
    }

    printf("# ERROR: unknown PID tuner command\r\n");
}

void Motor_PID_Tuner_Init(void)
{
    s_last_tx_ms = HAL_GetTick();
    printf("# Motor PID Tuner adapter ready\r\n");
    printf("# Loop: Iq; CSV: timestamp,setpoint,input,output,error,p,i,d\r\n");
}

void Motor_PID_Tuner_SendTelemetry(void)
{
    float setpoint;
    float input;
    float output;
    float error;

    if (!Motor_PID_Tuner_IsSafeToTune())
    {
        return;
    }

    setpoint = foc_input.Iq_ref;
    input = I_dq.Iq;
    output = V_dq.Vq;
    error = setpoint - input;

    printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
           (unsigned long)HAL_GetTick(),
           setpoint,
           input,
           output,
           error,
           g_current_pid_Iq.Kp,
           g_current_pid_Iq.Ki,
           g_current_pid_Iq.Kd);
}

void Motor_PID_Tuner_Task(void)
{
    char command[MOTOR_PID_TUNER_COMMAND_MAX];
    uint32_t now;

    if (Motor_PID_Tuner_TryReadCommand(command, sizeof(command)))
    {
        Motor_PID_Tuner_ProcessCommand(command);
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_tx_ms) >= MOTOR_PID_TUNER_TX_PERIOD_MS)
    {
        s_last_tx_ms = now;
        Motor_PID_Tuner_SendTelemetry();
    }
}
