/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
#include "dev_usart.h"
#include "service_message.h"
#include "state.h"
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
osThreadId defaultTaskHandle;
osThreadId stateTaskHandle;
osThreadId transportTaskHandle;
osThreadId recvTaskHandle;
osThreadId controlTaskHandle;
osSemaphoreId msgReadyBinarySemHandle;
osSemaphoreId controlBinarySemHandle;
osSemaphoreId transportBinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartStateTask(void const * argument);
void StartTransportTask(void const * argument);
void recvStartTask(void const * argument);
void StartControlTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of msgReadyBinarySem */
  osSemaphoreDef(msgReadyBinarySem);
  msgReadyBinarySemHandle = osSemaphoreCreate(osSemaphore(msgReadyBinarySem), 1);

  /* definition and creation of controlBinarySem */
  osSemaphoreDef(controlBinarySem);
  controlBinarySemHandle = osSemaphoreCreate(osSemaphore(controlBinarySem), 1);

  /* definition and creation of transportBinarySem */
  osSemaphoreDef(transportBinarySem);
  transportBinarySemHandle = osSemaphoreCreate(osSemaphore(transportBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of stateTask */
  osThreadDef(stateTask, StartStateTask, osPriorityNormal, 0, 128);
  stateTaskHandle = osThreadCreate(osThread(stateTask), NULL);

  /* definition and creation of transportTask */
  osThreadDef(transportTask, StartTransportTask, osPriorityNormal, 0, 512);
  transportTaskHandle = osThreadCreate(osThread(transportTask), NULL);

  /* definition and creation of recvTask */
  osThreadDef(recvTask, recvStartTask, osPriorityHigh, 0, 512);
  recvTaskHandle = osThreadCreate(osThread(recvTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityHigh, 0, 512);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  controller_init();
  /* Infinite loop */
  for (;;) {
    controller_step();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartStateTask */
/**
 * @brief Function implementing the stateTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStateTask */
void StartStateTask(void const * argument)
{
  /* USER CODE BEGIN StartStateTask */
  /* Infinite loop */
  for (;;) {
    state_step();
    osDelay(100);
  }
  /* USER CODE END StartStateTask */
}

/* USER CODE BEGIN Header_StartTransportTask */
/**
 * @brief Function implementing the transportTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTransportTask */
void StartTransportTask(void const * argument)
{
  /* USER CODE BEGIN StartTransportTask */
  /* Infinite loop */
  for (;;) {
    if (osOK == osSemaphoreWait(transportBinarySemHandle, portMAX_DELAY)) {
      dev_usart_periodic();
    }
  }
  /* USER CODE END StartTransportTask */
}

/* USER CODE BEGIN Header_recvStartTask */
/**
 * @brief Function implementing the recvTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_recvStartTask */
void recvStartTask(void const * argument)
{
  /* USER CODE BEGIN recvStartTask */
  /* Infinite loop */
  for (;;) {
    if (osOK == osSemaphoreWait(msgReadyBinarySemHandle, portMAX_DELAY)) {
      uint8_t buf[128];
      uint16_t size;
      dev_usart_get_rx_buffer(buf, &size);
      service_message_process(buf, size);
    }
  }
  /* USER CODE END recvStartTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the controlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for (;;) {
    if (osOK == osSemaphoreWait(controlBinarySemHandle, portMAX_DELAY)) {
      controller_core_step();
    }
  }
  /* USER CODE END StartControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
