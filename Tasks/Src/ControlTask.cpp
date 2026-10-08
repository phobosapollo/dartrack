#include "ControlTask.h"
#include "can.h"
#include "Gm6020.h"

static Gm6020 motor(1);

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