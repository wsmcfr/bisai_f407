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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "conveyor_motor_service.h"
#include "ldc1614_service.h"
#include "system_heartbeat_service.h"
#include "weight_service.h"

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
/* 传送带电机任务句柄保留在用户区，避免后续 CubeMX 重新生成时被覆盖。 */
static osThreadId_t conveyorMotorTaskHandle = NULL;

/* 传送带电机任务需要周期性闭环调速，优先级保持在普通业务级。 */
static const osThreadAttr_t conveyorMotorTask_attributes = {
  .name = "conveyorMotorTask",             /* 任务名称用于 RTOS 调试视图识别传送带电机服务线程。 */
  .stack_size = 256 * 4,                   /* 任务栈大小按字节配置，用于容纳电机状态机、串口发送和局部变量开销。 */
  .priority = (osPriority_t) osPriorityNormal, /* 普通优先级保证电机控制能及时运行，同时不压制更高实时性采样任务。 */
};

/* 心跳灯任务句柄保留在用户区，避免后续 CubeMX 重新生成时被覆盖。 */
static osThreadId_t heartbeatTaskHandle = NULL;

/* 心跳灯任务使用低优先级，便于观察系统调度是否被高优先级任务长期占用。 */
static const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",                 /* 任务名称用于调试器中定位系统心跳线程。 */
  .stack_size = 128 * 4,                   /* 心跳任务只翻转状态和延时等待，较小栈即可满足当前调用链。 */
  .priority = (osPriority_t) osPriorityLow, /* 低优先级避免心跳显示影响称重、LDC 检测和电机控制等业务任务。 */
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",                   /* 默认任务名称，当前用于承载 WeightService_Task 的称重业务入口。 */
  .stack_size = 128 * 4,                   /* 默认任务栈大小按字节配置，需覆盖称重服务入口的基础调用链。 */
  .priority = (osPriority_t) osPriorityNormal, /* 普通优先级让称重业务与传送带控制处于同一调度层级。 */
};
/* Definitions for ldc1614Task */
osThreadId_t ldc1614TaskHandle;
const osThreadAttr_t ldc1614Task_attributes = {
  .name = "ldc1614Task",                   /* LDC1614 检测任务名称，用于调试器区分电感检测线程。 */
  .stack_size = 256 * 4,                   /* LDC 检测包含采样、滤波和状态机处理，预留比默认任务更大的栈。 */
  .priority = (osPriority_t) osPriorityBelowNormal, /* 低于普通业务优先级，避免连续检测逻辑影响称重和电机控制响应。 */
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLdc1614Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  初始化 FreeRTOS 对象并创建本工程启动阶段需要运行的任务。
  * @param  None
  * @retval None
  *
  * 主要流程：
  * 1. 保留 CubeMX 生成的互斥量、信号量、定时器和队列扩展区；
  * 2. 创建默认任务和 LDC1614 任务；
  * 3. 在用户区创建传送带电机任务和心跳灯任务；
  * 4. 对关键任务创建失败路径调用 Error_Handler，避免系统以缺失任务的状态继续启动。
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ldc1614Task */
  ldc1614TaskHandle = osThreadNew(StartLdc1614Task, NULL, &ldc1614Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 传送带电机任务独立创建在用户区，负责 Emm42 的串口控制与状态机推进。 */
  conveyorMotorTaskHandle = osThreadNew(ConveyorMotorService_Task, NULL, &conveyorMotorTask_attributes);
  if (conveyorMotorTaskHandle == NULL)
  {
    /*
     * 传送带任务属于关键业务任务。
     * 若这里创建失败，后续串口命令虽然还能收发，但电机功能不会真正运行，
     * 因此直接进入统一错误处理，避免系统带着“半残状态”继续启动。
     */
    Error_Handler();
  }

  /* 心跳灯任务独立创建在用户区，后续重新生成 freertos.c 时不会被 CubeMX 覆盖。 */
  heartbeatTaskHandle = osThreadNew(SystemHeartbeatService_Task, NULL, &heartbeatTask_attributes);
  if (heartbeatTaskHandle == NULL)
  {
    /*
     * 心跳灯任务本身不参与业务控制，
     * 但它承担“调度器是否活着”的现场可视化观察职责。
     * 如果这里都创建失败，说明当前系统资源已经异常，仍然按致命错误处理。
     */
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务入口，当前把调度器线程转接到称重服务主循环。
  * @param  argument CMSIS-RTOS 创建任务时传入的用户参数，当前原样传给 WeightService_Task。
  * @retval None 任务入口不返回；称重服务内部负责持续运行或自行处理异常。
  *
  * 主要流程：
  * 1. 接收 RTOS 传入的任务参数；
  * 2. 立即调用 WeightService_Task，把实际业务保持在 User/App 模块中；
  * 3. 本函数不直接访问硬件资源，避免生成文件承载业务状态。
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 将默认任务接到用户应用层任务，避免把业务逻辑继续堆在生成文件里。 */
  WeightService_Task(argument);
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLdc1614Task */
/**
  * @brief  LDC1614 检测任务入口，负责把 RTOS 线程转接到电感检测服务。
  * @param  argument CMSIS-RTOS 创建任务时传入的用户参数，当前原样传给 Ldc1614Service_Task。
  * @retval None 任务入口不返回；LDC 服务内部负责周期采样、状态机和错误处理。
  *
  * 主要流程：
  * 1. 接收 RTOS 传入的任务参数；
  * 2. 调用 Ldc1614Service_Task 进入独立检测循环；
  * 3. 通过独立任务隔离称重、电机和电感检测的调度职责。
  */
/* USER CODE END Header_StartLdc1614Task */
void StartLdc1614Task(void *argument)
{
  /* USER CODE BEGIN StartLdc1614Task */
  /* 电感检测任务独立运行，避免和称重主循环耦合在同一个线程里。 */
  Ldc1614Service_Task(argument);
  /* USER CODE END StartLdc1614Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

