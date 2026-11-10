/**
 * @file config.cpp
 * @author cjw
 * @brief 宏
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "config.h"

/* Private macros ------------------------------------------------------------*/
// J0-J3 / ARM_CAN 已移至 config.h，供驱动和任务层共同使用。
// J3_CAN_ID / J3_MASTER_ID 统一使用 config.h 中的配置。
// J4_CAN_ID / J4_MASTER_ID 统一使用 config.h 中的配置。
#define GRIP_CAN_ID 0x17
#define GRIP_MASTER_ID 0x27
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
