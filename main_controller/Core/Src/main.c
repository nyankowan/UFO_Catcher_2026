/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "can.h"
#include "servo.h"
#include "robomas.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  UFO_MANUAL,   // 上下左右ボタンで移動
  UFO_LOWER,    // アーム下降
  UFO_GRAB,     // 爪(サーボ)を閉じる
  UFO_RAISE,    // アーム上昇
  UFO_HOME,     // 景品口(LEFT/UP端)へ移動
  UFO_RELEASE,  // 爪(サーボ)を開く
} ufo_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 爪サーボのパルス幅[us]。実機で要調整
#define SERVO_OPEN_US   1000
#define SERVO_CLOSE_US  2000
// Robomas(0x200)のモータ割当: 1=X, 2=Y, 3=Z(アーム昇降)。爪はサーボ(PC5)
#define MOTOR_X     0
#define MOTOR_Y     1
#define MOTOR_ARM   2

// --- 制御パラメータ。すべて実機で要調整 ---
// 電流指令の上限(C610: ±10000 = ±10A)
#define CUR_MAX         3000

// XY: 回転数[rpm]の速度制御
#define XY_MAX_RPM      3000.0f
#define XY_HOME_RPM     2000.0f
#define XY_ACCEL_RPM    6.0f     // 1ループ(1ms)あたりの目標回転数の変化量(加減速)

// Z: 位置制御(位置P -> 回転数PI)。単位はロータ軸のエンコーダ値(8192=1回転)
// 電源投入時のアーム位置(上端)を0とする。下降側は負方向を想定
#define Z_DOWN_POS      (-100000)
#define Z_UP_POS        0
#define Z_MAX_RPM       2000.0f
#define Z_KP_POS        0.05f    // [rpm/エンコーダ値]
#define Z_TOL           400      // 到達判定の許容誤差

// 回転数PI(XY, Zで共通)
#define VEL_KP          6.0f
#define VEL_KI          30.0f
#define VEL_I_MAX       2000.0f

#define FB_TIMEOUT_MS   100      // ロボマスのフィードバック途絶で電流0

// シーケンスの各時間[ms]。実機で要調整
#define T_LOWER_MAX_MS  5000     // 下降の到達待ち上限
#define T_GRAB_MS       1000
#define T_RAISE_MAX_MS  5000     // 上昇の到達待ち上限
#define T_HOME_MAX_MS   10000
#define T_RELEASE_MS    1000

#define LOOP_PERIOD_MS   1     // ロボマスの指令/FBは1kHz
#define CON_TIMEOUT_MS   100   // コントローラ無通信でモータ停止

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */
static controller_data_t con;
static uint32_t con_last_rx_ms;
static ufo_state_t state = UFO_MANUAL;
static uint32_t state_start_ms;

static float xy_tgt[2];   // XY目標回転数
static float xy_cmd[2];   // 加減速をかけた回転数指令
static int32_t z_target = Z_UP_POS;
static pid_ctrl_t vel_pid[3] = {
  [0 ... 2] = {.kp = VEL_KP, .ki = VEL_KI, .i_max = VEL_I_MAX, .out_max = CUR_MAX},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
static void can_rx_init(void);
static void can_rx_poll(void);
static void set_state(ufo_state_t next);
static void control_step(void);
static void actuate(int16_t cur[4]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// リミットスイッチはプルアップ入力なので押下でLOW
#define LIM_PRESSED(name) (HAL_GPIO_ReadPin(name##_GPIO_Port, name##_Pin) == GPIO_PIN_RESET)

static void can_rx_init(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0;
  f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0;
  f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();
  robomas_rx_init(&hcan2);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
}

static void can_rx_poll(void)
{
  CAN_RxHeaderTypeDef hdr;
  uint8_t d[CAN_DLC_MAX];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
    if (hdr.IDE == CAN_ID_STD && hdr.StdId == CAN_ID_CONTROLLER && hdr.DLC >= 1) {
      con.raw = d[0];
      con_last_rx_ms = HAL_GetTick();
    }
  }
}

static void set_state(ufo_state_t next)
{
  state = next;
  state_start_ms = HAL_GetTick();
  if (next == UFO_GRAB) servo_set_pulse_us(SERVO_CLOSE_US);
  if (next == UFO_RELEASE) servo_set_pulse_us(SERVO_OPEN_US);
}

static bool z_reached(void)
{
  int32_t err = z_target - robomas_fb[MOTOR_ARM].total;
  return err > -Z_TOL && err < Z_TOL;
}

// 目標(xy_tgt, z_target)を状態に応じて決める
static void control_step(void)
{
  uint32_t elapsed = HAL_GetTick() - state_start_ms;
  xy_tgt[MOTOR_X] = 0;
  xy_tgt[MOTOR_Y] = 0;

  switch (state) {
  case UFO_MANUAL:
    z_target = Z_UP_POS;
    // 同時押しは打ち消す
    if (con.left  && !con.right) xy_tgt[MOTOR_X] = -XY_MAX_RPM;
    if (con.right && !con.left)  xy_tgt[MOTOR_X] =  XY_MAX_RPM;
    if (con.up    && !con.down)  xy_tgt[MOTOR_Y] =  XY_MAX_RPM;
    if (con.down  && !con.up)    xy_tgt[MOTOR_Y] = -XY_MAX_RPM;
    if (con.cat) set_state(UFO_LOWER);
    break;

  case UFO_LOWER:
    z_target = Z_DOWN_POS;
    if (z_reached() || elapsed >= T_LOWER_MAX_MS) set_state(UFO_GRAB);
    break;

  case UFO_GRAB:
    if (elapsed >= T_GRAB_MS) set_state(UFO_RAISE);
    break;

  case UFO_RAISE:
    z_target = Z_UP_POS;
    if (z_reached() || elapsed >= T_RAISE_MAX_MS) set_state(UFO_HOME);
    break;

  case UFO_HOME: {
    bool x_done = LIM_PRESSED(LEFT_LIM);
    bool y_done = LIM_PRESSED(UP_LIM);
    if (!x_done) xy_tgt[MOTOR_X] = -XY_HOME_RPM;
    if (!y_done) xy_tgt[MOTOR_Y] =  XY_HOME_RPM;
    if ((x_done && y_done) || elapsed >= T_HOME_MAX_MS) set_state(UFO_RELEASE);
    break;
  }

  case UFO_RELEASE:
    if (elapsed >= T_RELEASE_MS) set_state(UFO_MANUAL);
    break;
  }
}

// 目標からモータ電流を計算する
static void actuate(int16_t cur[4])
{
  const float dt = LOOP_PERIOD_MS / 1000.0f;

  for (int i = 0; i < 3; i++) {
    if (!robomas_alive(i, FB_TIMEOUT_MS)) {
      // フィードバックがないモータは電流0。再開時に急発進しないよう状態も戻す
      for (int j = 0; j < 3; j++) pid_reset(&vel_pid[j]);
      xy_cmd[MOTOR_X] = xy_cmd[MOTOR_Y] = 0;
      return;
    }
  }

  // XY: 加減速付きの速度制御。端のリミットに向かう指令は即0
  for (int i = 0; i < 2; i++) {
    float diff = xy_tgt[i] - xy_cmd[i];
    if (diff >  XY_ACCEL_RPM) diff =  XY_ACCEL_RPM;
    if (diff < -XY_ACCEL_RPM) diff = -XY_ACCEL_RPM;
    xy_cmd[i] += diff;
  }
  if (LIM_PRESSED(LEFT_LIM)  && xy_cmd[MOTOR_X] < 0) xy_cmd[MOTOR_X] = 0;
  if (LIM_PRESSED(RIGHT_LIM) && xy_cmd[MOTOR_X] > 0) xy_cmd[MOTOR_X] = 0;
  if (LIM_PRESSED(UP_LIM)    && xy_cmd[MOTOR_Y] > 0) xy_cmd[MOTOR_Y] = 0;
  if (LIM_PRESSED(DOWN_LIM)  && xy_cmd[MOTOR_Y] < 0) xy_cmd[MOTOR_Y] = 0;

  for (int i = 0; i < 2; i++) {
    cur[i] = (int16_t)pid_step(&vel_pid[i], xy_cmd[i] - robomas_fb[i].rpm, dt);
  }

  // Z: 位置P -> 回転数PI
  float z_rpm = Z_KP_POS * (float)(z_target - robomas_fb[MOTOR_ARM].total);
  if (z_rpm >  Z_MAX_RPM) z_rpm =  Z_MAX_RPM;
  if (z_rpm < -Z_MAX_RPM) z_rpm = -Z_MAX_RPM;
  cur[MOTOR_ARM] = (int16_t)pid_step(&vel_pid[MOTOR_ARM], z_rpm - robomas_fb[MOTOR_ARM].rpm, dt);
}

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  can_rx_init();
  servo_init(SERVO_OPEN_US);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    can_rx_poll();

    int16_t cur[4] = {0};
    if (HAL_GetTick() - con_last_rx_ms > CON_TIMEOUT_MS) {
      con.raw = 0;  // 通信断中はMANUALで全停止。シーケンス実行中はそのまま完了させる
    }
    control_step();
    actuate(cur);

    can_command_data_t com = {.id = CAN_ID_ROBOMAS_SEND};
    can_protocol_pack_robomas(&com.data, cur);
    stm_can_send(&hcan2, &com);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, state == UFO_MANUAL ? GPIO_PIN_RESET : GPIO_PIN_SET);
    // 次のtick(1ms)まで待つ。ロボマスのFBは割り込みで受信済み
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() == t) {}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 30;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_LIM_Pin UP_LIM_Pin RIGHT_LIM_Pin DOWN_LIM_Pin */
  GPIO_InitStruct.Pin = LEFT_LIM_Pin|UP_LIM_Pin|RIGHT_LIM_Pin|DOWN_LIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
