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
#include "camera_motor_service.h"
#include "conveyor_motor_service.h"
#include "ldc1614_service.h"
#include "robot_arm_service.h"
#include "system_heartbeat_service.h"
#include "weight_service.h"
#include "usart.h"

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
  .stack_size = 320 * 4,                   /* 电机任务会经过 Emm42 状态机、串口发送和日志路径，扩到 320 word 减少边界场景栈压缩。 */
  .priority = (osPriority_t) osPriorityNormal, /* 普通优先级保证电机控制能及时运行，同时不压制更高实时性采样任务。 */
};

/* 摄像头运动电机任务句柄保留在用户区，负责 USART6 上两个 Emm42 电机的串行控制。 */
static osThreadId_t cameraMotorTaskHandle = NULL;

/* 摄像头运动电机任务只响应点动/停止命令，优先级保持在普通业务级，避免阻塞称重和 LDC 采样。 */
static const osThreadAttr_t cameraMotorTask_attributes = {
  .name = "cameraMotorTask",               /* 任务名称用于 RTOS 调试视图识别摄像头运动电机服务线程。 */
  .stack_size = 320 * 4,                   /* 摄像头电机任务同样会经过双电机控制和日志路径，扩到 320 word 给后续调试留余量。 */
  .priority = (osPriority_t) osPriorityNormal, /* 普通优先级保证点动命令能及时执行，同时不压制更高实时性采样任务。 */
};

/* 心跳灯任务句柄保留在用户区，避免后续 CubeMX 重新生成时被覆盖。 */
static osThreadId_t heartbeatTaskHandle = NULL;

/* 心跳灯任务使用低优先级，便于观察系统调度是否被高优先级任务长期占用。 */
static const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",                 /* 任务名称用于调试器中定位系统心跳线程。 */
  .stack_size = 128 * 4,                   /* 心跳任务只翻转状态和延时等待，较小栈即可满足当前调用链。 */
  .priority = (osPriority_t) osPriorityLow, /* 低优先级避免心跳显示影响称重、LDC 检测和电机控制等业务任务。 */
};

/* 机械臂转发任务句柄保留在用户区，负责把 USART2 主链路触发的机械臂动作异步转发到 USART3。 */
static osThreadId_t robotArmTaskHandle = NULL;

/* 机械臂转发任务只处理短帧队列和 USART3 阻塞发送，USART3 当前按 115200 8N1 对接 ESP32 PA5/PA4 串口。 */
/* 栈和优先级保持在普通业务任务级别，避免机械臂短帧转发影响称重、电感检测等更高实时性路径。 */
static const osThreadAttr_t robotArmTask_attributes = {
  .name = "robotArmTask",                  /* 任务名称用于 RTOS 调试视图中识别机械臂 USART2->USART3 转发线程。 */
  .stack_size = 512 * 4,                   /* 机械臂任务会叠加 ACK/DONE 帧缓存、称重结果打包和日志调用链，扩到 512 word 规避栈踩踏。 */
  .priority = (osPriority_t) osPriorityNormal, /* 普通优先级保证机械臂命令及时转发，同时不压制更高实时性采样或中断回调。 */
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ldc1614Task */
osThreadId_t ldc1614TaskHandle;
const osThreadAttr_t ldc1614Task_attributes = {
  .name = "ldc1614Task",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLdc1614Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* 摄像头运动电机任务独立创建，负责 USART6 PC6/PC7 上地址 0x02 和 0x03 两个 Emm42 电机。 */
  cameraMotorTaskHandle = osThreadNew(CameraMotorService_Task, NULL, &cameraMotorTask_attributes);
  if (cameraMotorTaskHandle == NULL)
  {
    /*
     * 摄像头电机任务创建失败时，后续 `CAMFWD/CAMZ/CAMSTOP` 命令无法执行，
     * 自动视觉微调也无法工作，因此按关键业务任务失败处理。
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

  /* 机械臂服务先创建队列，再启动转发任务，确保 USART2 主链路触发机械臂动作后有可投递的目标。 */
  if (RobotArmService_Init() == 0U)
  {
    /*
     * 机械臂队列创建失败说明 FreeRTOS 堆空间不足。
     * 此时即使 USART2 还能收到命令，也无法安全异步转发到 USART3，因此直接进入统一错误处理。
     */
    Error_Handler();
  }

  robotArmTaskHandle = osThreadNew(RobotArmService_Task, NULL, &robotArmTask_attributes);
  if (robotArmTaskHandle == NULL)
  {
    /*
     * 机械臂转发任务是 STM32 与 ESP32 机械臂控制链路的执行端。
     * 若任务创建失败，机械臂命令会停留在队列中无法发送，所以按关键业务任务失败处理。
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

/**
  * @brief FreeRTOS 检测到任务栈越界时的致命故障钩子。
  * @param xTask 发生栈越界的任务句柄；故障现场不再解引用，避免访问已损坏对象。
  * @param pcTaskName 发生栈越界的任务名称；故障现场不格式化输出，避免继续消耗栈。
  * @retval None 本函数发送固定诊断字节后关闭中断并停机，不返回调度器。
  *
  * 设计原因：栈已经损坏时不能安全调用 printf、互斥锁或复杂协议发送，
  * 因此仅使用非 MP157 主链路的 USART2 轮询发送固定英文标记，
  * 现场看到 `F4 STACK OVERFLOW` 即可确认资源根因，同时避免污染 USART1 二进制主链路。
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  static const uint8_t message[] = "\r\n[FATAL] F4 STACK OVERFLOW\r\n";

  (void)xTask;
  (void)pcTaskName;
  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)message,
                          (uint16_t)(sizeof(message) - 1U),
                          100U);
  __disable_irq();
  for (;;)
  {
    /* 保持停机，避免栈损坏后继续执行电机控制产生不可预测动作。 */
  }
}

/**
  * @brief FreeRTOS 动态内存申请失败时的致命故障钩子。
  * @retval None 本函数发送固定诊断字节后关闭中断并停机，不返回调度器。
  *
  * 队列、信号量或任务创建时若 heap_4 空间不足会进入这里，
  * USART2 固定标记用于区分“队列投递逻辑问题”和“真正的 FreeRTOS 堆耗尽”，
  * USART1 继续保持 MP157 二进制主链路，不直接输出致命文本。
  */
void vApplicationMallocFailedHook(void)
{
  static const uint8_t message[] = "\r\n[FATAL] F4 MALLOC FAILED\r\n";

  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)message,
                          (uint16_t)(sizeof(message) - 1U),
                          100U);
  __disable_irq();
  for (;;)
  {
    /* 保持停机，避免关键任务或队列创建失败后系统继续处于半工作状态。 */
  }
}

/* USER CODE END Application */

