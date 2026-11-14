/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "ins_task.h"
#include "control_task.h"
#include "comm_task.h"
#include "user_freertos.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* User tasks are created here, following the reference project's layout.
 * CubeMX owns only defaultTask in main.c. */
osThreadId_t sensorTaskHandle;
osThreadId_t controlTaskHandle;
osThreadId_t commTaskHandle;

static const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask", .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
static const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask", .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
static const osThreadAttr_t commTask_attributes = {
  .name = "commTask", .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static void SENSORTASK(void *argument)
{
  (void)argument;
  INS_task();
}
static void CONTROLTASK(void *argument)
{
  (void)argument;
  Control_task();
}
static void COMMTASK(void *argument)
{
  (void)argument;
  Comm_task();
}

void MX_USER_FREERTOS_Init(void)
{
  sensorTaskHandle = osThreadNew(SENSORTASK, NULL, &sensorTask_attributes);
  controlTaskHandle = osThreadNew(CONTROLTASK, NULL, &controlTask_attributes);
  commTaskHandle = osThreadNew(COMMTASK, NULL, &commTask_attributes);
  if (sensorTaskHandle == NULL || controlTaskHandle == NULL || commTaskHandle == NULL) {
    Error_Handler();
  }
}

/* USER CODE END Application */

