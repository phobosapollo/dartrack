
#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 CAN
void ControlTaskInit(void);

// TIM6 1kHz 中断调用
void ControlTaskTick(void);

// main while(1) 中调用
void ControlTaskRun(void);

// 保留原有接口声明
void MainTask(void);
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H