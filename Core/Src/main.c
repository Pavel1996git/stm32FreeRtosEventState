/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for TaskBlinkLong */
osThreadId_t TaskBlinkLongHandle;
uint32_t TaskBlinkLongBuffer[ 128 ];
osStaticThreadDef_t TaskBlinkLongControlBlock;
const osThreadAttr_t TaskBlinkLong_attributes = {
  .name = "TaskBlinkLong",
  .cb_mem = &TaskBlinkLongControlBlock,
  .cb_size = sizeof(TaskBlinkLongControlBlock),
  .stack_mem = &TaskBlinkLongBuffer[0],
  .stack_size = sizeof(TaskBlinkLongBuffer),
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for TaskBlinkShort */
osThreadId_t TaskBlinkShortHandle;
uint32_t TaskBlinkShortBuffer[ 128 ];
osStaticThreadDef_t TaskBlinkShortControlBlock;
const osThreadAttr_t TaskBlinkShort_attributes = {
  .name = "TaskBlinkShort",
  .cb_mem = &TaskBlinkShortControlBlock,
  .cb_size = sizeof(TaskBlinkShortControlBlock),
  .stack_mem = &TaskBlinkShortBuffer[0],
  .stack_size = sizeof(TaskBlinkShortBuffer),
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for TaskBlinkReal */
osThreadId_t TaskBlinkRealHandle;
uint32_t TaskBlinkRealBuffer[ 128 ];
osStaticThreadDef_t TaskBlinkRealControlBlock;
const osThreadAttr_t TaskBlinkReal_attributes = {
  .name = "TaskBlinkReal",
  .cb_mem = &TaskBlinkRealControlBlock,
  .cb_size = sizeof(TaskBlinkRealControlBlock),
  .stack_mem = &TaskBlinkRealBuffer[0],
  .stack_size = sizeof(TaskBlinkRealBuffer),
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for TaskStateHandle */
osThreadId_t TaskStateHandleHandle;
uint32_t TaskStateHandleBuffer[ 128 ];
osStaticThreadDef_t TaskStateHandleControlBlock;
const osThreadAttr_t TaskStateHandle_attributes = {
  .name = "TaskStateHandle",
  .cb_mem = &TaskStateHandleControlBlock,
  .cb_size = sizeof(TaskStateHandleControlBlock),
  .stack_mem = &TaskStateHandleBuffer[0],
  .stack_size = sizeof(TaskStateHandleBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TaskExtEventHan */
osThreadId_t TaskExtEventHanHandle;
uint32_t TaskEventHandleBuffer[ 128 ];
osStaticThreadDef_t TaskEventHandleControlBlock;
const osThreadAttr_t TaskExtEventHan_attributes = {
  .name = "TaskExtEventHan",
  .cb_mem = &TaskEventHandleControlBlock,
  .cb_size = sizeof(TaskEventHandleControlBlock),
  .stack_mem = &TaskEventHandleBuffer[0],
  .stack_size = sizeof(TaskEventHandleBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for TaskStart */
osThreadId_t TaskStartHandle;
uint32_t TaskStartBuffer[ 128 ];
osStaticThreadDef_t TaskStartControlBlock;
const osThreadAttr_t TaskStart_attributes = {
  .name = "TaskStart",
  .cb_mem = &TaskStartControlBlock,
  .cb_size = sizeof(TaskStartControlBlock),
  .stack_mem = &TaskStartBuffer[0],
  .stack_size = sizeof(TaskStartBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskTimerEvent */
osThreadId_t TaskTimerEventHandle;
uint32_t TaskTimerEventBuffer[ 128 ];
osStaticThreadDef_t TaskTimerEventControlBlock;
const osThreadAttr_t TaskTimerEvent_attributes = {
  .name = "TaskTimerEvent",
  .cb_mem = &TaskTimerEventControlBlock,
  .cb_size = sizeof(TaskTimerEventControlBlock),
  .stack_mem = &TaskTimerEventBuffer[0],
  .stack_size = sizeof(TaskTimerEventBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for TaskStartFSM1 */
osThreadId_t TaskStartFSM1Handle;
uint32_t TaskStartFSM1Buffer[ 128 ];
osStaticThreadDef_t TaskStartFSM1ControlBlock;
const osThreadAttr_t TaskStartFSM1_attributes = {
  .name = "TaskStartFSM1",
  .cb_mem = &TaskStartFSM1ControlBlock,
  .cb_size = sizeof(TaskStartFSM1ControlBlock),
  .stack_mem = &TaskStartFSM1Buffer[0],
  .stack_size = sizeof(TaskStartFSM1Buffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for TaskStartFSM2 */
osThreadId_t TaskStartFSM2Handle;
uint32_t TaskStartFSM2Buffer[ 128 ];
osStaticThreadDef_t TaskStartFSM2ControlBlock;
const osThreadAttr_t TaskStartFSM2_attributes = {
  .name = "TaskStartFSM2",
  .cb_mem = &TaskStartFSM2ControlBlock,
  .cb_size = sizeof(TaskStartFSM2ControlBlock),
  .stack_mem = &TaskStartFSM2Buffer[0],
  .stack_size = sizeof(TaskStartFSM2Buffer),
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for QueueState */
osMessageQueueId_t QueueStateHandle;
uint8_t QueueEventBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t QueueEventControlBlock;
const osMessageQueueAttr_t QueueState_attributes = {
  .name = "QueueState",
  .cb_mem = &QueueEventControlBlock,
  .cb_size = sizeof(QueueEventControlBlock),
  .mq_mem = &QueueEventBuffer,
  .mq_size = sizeof(QueueEventBuffer)
};
/* Definitions for QueueMyEvent */
osMessageQueueId_t QueueMyEventHandle;
uint8_t QueueMyEventBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t QueueMyEventControlBlock;
const osMessageQueueAttr_t QueueMyEvent_attributes = {
  .name = "QueueMyEvent",
  .cb_mem = &QueueMyEventControlBlock,
  .cb_size = sizeof(QueueMyEventControlBlock),
  .mq_mem = &QueueMyEventBuffer,
  .mq_size = sizeof(QueueMyEventBuffer)
};
/* Definitions for myQueue0 */
osMessageQueueId_t myQueue0Handle;
uint8_t myQueue0Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue0ControlBlock;
const osMessageQueueAttr_t myQueue0_attributes = {
  .name = "myQueue0",
  .cb_mem = &myQueue0ControlBlock,
  .cb_size = sizeof(myQueue0ControlBlock),
  .mq_mem = &myQueue0Buffer,
  .mq_size = sizeof(myQueue0Buffer)
};
/* Definitions for myQueue1 */
osMessageQueueId_t myQueue1Handle;
uint8_t myQueue1Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue1ControlBlock;
const osMessageQueueAttr_t myQueue1_attributes = {
  .name = "myQueue1",
  .cb_mem = &myQueue1ControlBlock,
  .cb_size = sizeof(myQueue1ControlBlock),
  .mq_mem = &myQueue1Buffer,
  .mq_size = sizeof(myQueue1Buffer)
};
/* Definitions for myQueue2 */
osMessageQueueId_t myQueue2Handle;
uint8_t myQueue2Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue2ControlBlock;
const osMessageQueueAttr_t myQueue2_attributes = {
  .name = "myQueue2",
  .cb_mem = &myQueue2ControlBlock,
  .cb_size = sizeof(myQueue2ControlBlock),
  .mq_mem = &myQueue2Buffer,
  .mq_size = sizeof(myQueue2Buffer)
};
/* Definitions for myQueue3 */
osMessageQueueId_t myQueue3Handle;
uint8_t myQueue3Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue3ControlBlock;
const osMessageQueueAttr_t myQueue3_attributes = {
  .name = "myQueue3",
  .cb_mem = &myQueue3ControlBlock,
  .cb_size = sizeof(myQueue3ControlBlock),
  .mq_mem = &myQueue3Buffer,
  .mq_size = sizeof(myQueue3Buffer)
};
/* Definitions for myQueue4 */
osMessageQueueId_t myQueue4Handle;
uint8_t myQueue4Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue4ControlBlock;
const osMessageQueueAttr_t myQueue4_attributes = {
  .name = "myQueue4",
  .cb_mem = &myQueue4ControlBlock,
  .cb_size = sizeof(myQueue4ControlBlock),
  .mq_mem = &myQueue4Buffer,
  .mq_size = sizeof(myQueue4Buffer)
};
/* Definitions for myQueue5 */
osMessageQueueId_t myQueue5Handle;
uint8_t myQueue5Buffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue5ControlBlock;
const osMessageQueueAttr_t myQueue5_attributes = {
  .name = "myQueue5",
  .cb_mem = &myQueue5ControlBlock,
  .cb_size = sizeof(myQueue5ControlBlock),
  .mq_mem = &myQueue5Buffer,
  .mq_size = sizeof(myQueue5Buffer)
};
/* Definitions for TimerBlinkDelay */
osTimerId_t TimerBlinkDelayHandle;
osStaticTimerDef_t TimerBlinkDelayControlBlock;
const osTimerAttr_t TimerBlinkDelay_attributes = {
  .name = "TimerBlinkDelay",
  .cb_mem = &TimerBlinkDelayControlBlock,
  .cb_size = sizeof(TimerBlinkDelayControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void fTaskBlinkLong(void *argument);
void fTaskBlinkShort(void *argument);
void fTaskBlinkReal(void *argument);
void fTaskStateHandler(void *argument);
void fTaskEventHandler(void *argument);
void fTaskStart(void *argument);
void fTaskTimerEvent(void *argument);
void fTaskStartFSM1(void *argument);
void fTaskStartFSM2(void *argument);
void CallbackTimerBlinkDelay(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DELAY_QUEUE 15000
// Определение состояний и событий

typedef enum {
	TaskBlinkLong,
	TaskBlinkShort,
	TaskBlinkReal,
	TaskStateHandle,
	TTaskEventHandle,
	TaskStart,
	TaskTimerEvent,
	TaskStartFSM1,
	TaskStartFSM2,
    NUM_STATES
} State_t;

typedef enum {
	EVENT_END_START,
	EVENT_END_BLIND_SHORT,
	EVENT_END_BLIND_LONG,
	EVENT_END_BLIND_REAL,
	EVENT_START_FSM_1,
	EVENT_START_FSM_2,
	EVENT_TIMER_UPDATE,
	fgrf45f,
    NUM_EVENTS
} Event_t;
/*
typedef enum {
	QueueState,
	QueueMyEvent,
	myQueue0,
	myQueue1,
	myQueue2,
	myQueue3,
    NUM_EVENTS
} Event_t;
*/


// Тип для таблицы переходов
int8_t transitionTable[NUM_STATES][NUM_EVENTS];
/*
// Структура для связи состояния и очереди
typedef struct {
    State_t state;
    osMessageQueueId_t queueHandle;
} StateQueueMapping_t;
*/

// Создание массива структур для ассоциации состояний и очередей
osMessageQueueId_t stateQueueMappings[NUM_STATES];


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerBlinkDelay */
  TimerBlinkDelayHandle = osTimerNew(CallbackTimerBlinkDelay, osTimerPeriodic, NULL, &TimerBlinkDelay_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueState */
  QueueStateHandle = osMessageQueueNew (16, sizeof(uint16_t), &QueueState_attributes);

  /* creation of QueueMyEvent */
  QueueMyEventHandle = osMessageQueueNew (16, sizeof(uint16_t), &QueueMyEvent_attributes);

  /* creation of myQueue0 */
  myQueue0Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue0_attributes);

  /* creation of myQueue1 */
  myQueue1Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue1_attributes);

  /* creation of myQueue2 */
  myQueue2Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue2_attributes);

  /* creation of myQueue3 */
  myQueue3Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue3_attributes);

  /* creation of myQueue4 */
  myQueue4Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue4_attributes);

  /* creation of myQueue5 */
  myQueue5Handle = osMessageQueueNew (4, sizeof(uint16_t), &myQueue5_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskBlinkLong */
  TaskBlinkLongHandle = osThreadNew(fTaskBlinkLong, NULL, &TaskBlinkLong_attributes);

  /* creation of TaskBlinkShort */
  TaskBlinkShortHandle = osThreadNew(fTaskBlinkShort, NULL, &TaskBlinkShort_attributes);

  /* creation of TaskBlinkReal */
  TaskBlinkRealHandle = osThreadNew(fTaskBlinkReal, NULL, &TaskBlinkReal_attributes);

  /* creation of TaskStateHandle */
  TaskStateHandleHandle = osThreadNew(fTaskStateHandler, NULL, &TaskStateHandle_attributes);

  /* creation of TaskExtEventHan */
  TaskExtEventHanHandle = osThreadNew(fTaskEventHandler, NULL, &TaskExtEventHan_attributes);

  /* creation of TaskStart */
  TaskStartHandle = osThreadNew(fTaskStart, NULL, &TaskStart_attributes);

  /* creation of TaskTimerEvent */
  TaskTimerEventHandle = osThreadNew(fTaskTimerEvent, NULL, &TaskTimerEvent_attributes);

  /* creation of TaskStartFSM1 */
  TaskStartFSM1Handle = osThreadNew(fTaskStartFSM1, NULL, &TaskStartFSM1_attributes);

  /* creation of TaskStartFSM2 */
  TaskStartFSM2Handle = osThreadNew(fTaskStartFSM2, NULL, &TaskStartFSM2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  int8_t transitionTable[NUM_STATES][NUM_EVENTS];
  //initializeTransitionTable((int8_t **)transitionTable, NUM_STATES, NUM_EVENTS);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void initializeTransitionTable(int8_t **table, uint8_t num_states, uint8_t num_events) {
    // Заполнение таблицы состояний значением -1
    for (uint8_t i = 0; i < num_states; ++i) {
        for (uint8_t j = 0; j < num_events; ++j) {
            table[i][j] = -1;
        }
    }
}

void addToTransitionTable(State_t initial_state, State_t new_state, Event_t event) {
    // Проверяем, что переданные состояния и событие валидны
    if (initial_state >= NUM_STATES || new_state >= NUM_STATES || event >= NUM_EVENTS) {
        // Выводим сообщение об ошибке или принимаем другие меры
        return;
    }

    // Заполняем таблицу переходов
    transitionTable[initial_state][event] = new_state;
}

State_t handleTransition(Event_t event, State_t currentState) {
    // Проверяем, есть ли новое состояние для данного события
    if (transitionTable[currentState][event] != -1) {
        // Выполняем переход к новому состоянию
        //*currentState = transitionTable[*currentState][event];
    	State_t newState = transitionTable[currentState][event];
    	//xQueueSend(QueueEvent, &newState, portMAX_DELAY);
    	return newState;
        // Здесь можно выполнить дополнительные действия, связанные с переходом
        // Например, вызвать функцию, связанную с новым состоянием
        // processState(*currentState);
    } else {
        // Если не найдено нового состояния для события, то игнорируем его
        // и передаем его более низкоприоритетной задаче
        //sendEventToLowPriorityTask(event);
    	return -1;
    }
}

// Функция для создания ассоциативного значения (связи) между состоянием и очередью
void createStateQueueMapping(State_t state, osMessageQueueId_t queueHandle) {
    //StateQueueMapping_t mapping = {state, queueHandle};
    stateQueueMappings[state] = queueHandle;
}

// Функция для получения очереди по состоянию
osMessageQueueId_t getQueueForState(State_t state) {
    return stateQueueMappings[state];
}

void MessageQueueState(State_t state) {
	xQueueSend(getQueueForState(state), &state, pdMS_TO_TICKS(DELAY_QUEUE));
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_fTaskBlinkLong */
/**
  * @brief  Function implementing the TaskBlinkLong thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_fTaskBlinkLong */
void fTaskBlinkLong(void *argument)
{
  /* USER CODE BEGIN 5 */
	State_t newState = transitionTable[NUM_STATES][NUM_EVENTS];
	xQueueSend(QueueStateHandle, &newState, portMAX_DELAY);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fTaskBlinkShort */
/**
* @brief Function implementing the TaskBlinkShort thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskBlinkShort */
void fTaskBlinkShort(void *argument)
{
  /* USER CODE BEGIN fTaskBlinkShort */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskBlinkShort */
}

/* USER CODE BEGIN Header_fTaskBlinkReal */
/**
* @brief Function implementing the TaskBlinkReal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskBlinkReal */
void fTaskBlinkReal(void *argument)
{
  /* USER CODE BEGIN fTaskBlinkReal */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskBlinkReal */
}

/* USER CODE BEGIN Header_fTaskStateHandler */
/**
* @brief Function implementing the TaskStateHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskStateHandler */
void fTaskStateHandler(void *argument)
{
  /* USER CODE BEGIN fTaskStateHandler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskStateHandler */
}

/* USER CODE BEGIN Header_fTaskEventHandler */
/**
* @brief Function implementing the TaskEventHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskEventHandler */
void fTaskEventHandler(void *argument)
{
  /* USER CODE BEGIN fTaskEventHandler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskEventHandler */
}

/* USER CODE BEGIN Header_fTaskStart */
/**
* @brief Function implementing the TaskStart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskStart */
void fTaskStart(void *argument)
{
  /* USER CODE BEGIN fTaskStart */
	initializeTransitionTable((int8_t **)transitionTable, NUM_STATES, NUM_EVENTS);
	addToTransitionTable(TaskStart, TaskBlinkShort, END_START);
	addToTransitionTable(TaskBlinkShort, TaskBlinkLong, END_BLIND_SHORT);
	addToTransitionTable(TaskBlinkLong, TaskBlinkShort, END_BLIND_LONG);

	createStateQueueMapping(TaskStart, myQueue0Handle);

	State_t newState = transitionTable[NUM_STATES][NUM_EVENTS];
	handleTransition
	xQueueSend(QueueStateHandle, &newState, portMAX_DELAY);
  /* Infinite loop */
  for(;;)
  {


	State_t currentState;
	xQueueReceive(QueueStateHandle, &currentState, portMAX_DELAY);
    osDelay(1);
  }
  /* USER CODE END fTaskStart */
}

/* USER CODE BEGIN Header_fTaskTimerEvent */
/**
* @brief Function implementing the TaskTimerEvent thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskTimerEvent */
void fTaskTimerEvent(void *argument)
{
  /* USER CODE BEGIN fTaskTimerEvent */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskTimerEvent */
}

/* USER CODE BEGIN Header_fTaskStartFSM1 */
/**
* @brief Function implementing the TaskStartFSM1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskStartFSM1 */
void fTaskStartFSM1(void *argument)
{
  /* USER CODE BEGIN fTaskStartFSM1 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskStartFSM1 */
}

/* USER CODE BEGIN Header_fTaskStartFSM2 */
/**
* @brief Function implementing the TaskStartFSM2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskStartFSM2 */
void fTaskStartFSM2(void *argument)
{
  /* USER CODE BEGIN fTaskStartFSM2 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTaskStartFSM2 */
}

/* CallbackTimerBlinkDelay function */
void CallbackTimerBlinkDelay(void *argument)
{
  /* USER CODE BEGIN CallbackTimerBlinkDelay */

  /* USER CODE END CallbackTimerBlinkDelay */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
