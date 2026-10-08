#include "ControlTask.h"
#include "can.h"
#include "Gm6020.h"
#include "Pid.h"
#include "iwdg.h"
#include <cmath>

constexpr float PI = 3.14159265358979323846f;

constexpr float SINE_AMPLITUDE = 50.0f;  // 幅值 50 rpm
constexpr float SINE_PERIOD = 10.0f;     // 周期 10 s

volatile float debug_time = 0.0f;

volatile float startup_comp = 700.0f;

volatile float debug_filtered_speed = 0.0f;

static Pid speed_pid(
    15.0f,     // Kp
    0.0f,      // Ki
    0.0f,      // Kd
    1000.0f,   // 输出上限
    -1000.0f   // 输出下限
);

volatile float debug_target_speed = 0.0f;
volatile float debug_speed_output = 0.0f;

static Gm6020 motor(1);

volatile uint32_t debug_control_tick = 0;

volatile uint32_t debug_can_tx_count = 0;
volatile uint32_t debug_can_tx_error = 0;

volatile float debug_angle = 0.0f;
volatile float debug_vel = 0.0f;
volatile int16_t debug_rpm = 0;
volatile float debug_current = 0.0f;
volatile float debug_temp = 0.0f;

extern "C" {

volatile uint32_t debug_can_filter = 0xFFFFFFFF;
volatile uint32_t debug_can_start = 0xFFFFFFFF;
volatile uint32_t debug_can_notify = 0xFFFFFFFF;

volatile uint32_t debug_can_rx_count = 0;
volatile uint32_t debug_can_rx_id = 0;
volatile uint8_t debug_can_rx_data[8] = {0};
volatile uint32_t debug_can_rx_error = 0;

void MotorSendCurrent(int16_t current);

static volatile uint8_t control_pending = 0;

void ControlTaskTick(void)
{
    static uint8_t tick_count = 0;

    tick_count++;

    if (tick_count >= 10)
    {
        tick_count = 0;
        control_pending = 1;
        debug_control_tick++;
    }
}

void ControlTaskRun(void)
{

    if (!control_pending)
    {
        return;
    }

    control_pending = 0;

    // 100 Hz 控制周期
    constexpr float dt = 0.01f;

    static float time = 0.0f;
    time += dt;

    // 正弦目标速度
    float target_speed =
        SINE_AMPLITUDE *
        sinf(2.0f * PI * time / SINE_PERIOD);

    // 电机实际速度
    float actual_speed = static_cast<float>(motor.velRpm());

    // 一阶低通滤波
    constexpr float alpha = 0.8f;
    static float filtered_speed = 0.0f;

    filtered_speed =
        alpha * actual_speed +
        (1.0f - alpha) * filtered_speed;

    // PID 计算
    float output = speed_pid.calc(
        target_speed,
        actual_speed
    );
    // 连续摩擦补偿
    constexpr float friction_comp = 350.0f;
    constexpr float friction_scale = 3.0f;

    output += friction_comp *
            tanhf(target_speed / friction_scale);

    // 最终电流限幅
    if (output > 1000.0f) output = 1000.0f;
    if (output < -1000.0f) output = -1000.0f;

    // Ozone 观察变量
    debug_time = time;
    debug_target_speed = target_speed;
    debug_speed_output = output;

    // 发送 PID 电流输出
    MotorSendCurrent(static_cast<int16_t>(output));

    debug_filtered_speed = filtered_speed;

    // 喂狗
    HAL_IWDG_Refresh(&hiwdg);
}

void MotorSendCurrent(int16_t current)
{
    motor.setCurrent(current);

    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.StdId = motor.txId();
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8] = {0};
    motor.encode(tx_data);

    uint32_t tx_mailbox;

    if (HAL_CAN_AddTxMessage(
            &hcan1,
            &tx_header,
            tx_data,
            &tx_mailbox) == HAL_OK)
    {
        debug_can_tx_count++;
    }
    else
    {
        debug_can_tx_error++;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
    {
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(
            hcan,
            CAN_RX_FIFO0,
            &rx_header,
            rx_data) == HAL_OK)
    {
        debug_can_rx_count++;
        debug_can_rx_id = rx_header.StdId;

        if (rx_header.IDE == CAN_ID_STD &&
            rx_header.StdId == motor.rxId() &&
            rx_header.DLC == 8)
        {
            motor.decode(rx_data);

            debug_angle = motor.angle();
            debug_vel = motor.vel();
            debug_rpm = motor.velRpm();
            debug_current = motor.current();
            debug_temp = motor.temp();
        }

        for (int i = 0; i < 8; i++)
        {
            debug_can_rx_data[i] = rx_data[i];
        }
    }
    else
    {
        debug_can_rx_error++;
    }
}

void ControlTaskInit(void)
{
    CAN_FilterTypeDef filter = {};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    debug_can_filter = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (debug_can_filter != HAL_OK) return;

    debug_can_start = HAL_CAN_Start(&hcan1);
    if (debug_can_start != HAL_OK) return;

    debug_can_notify = HAL_CAN_ActivateNotification(
        &hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING
    );
}

}