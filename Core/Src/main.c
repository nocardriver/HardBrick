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
#include "dcache.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "usart.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "tx_api.h"
#include "LC760Z.h"
#include "gps.h"
#include "ili9341.h"
#include "display.h"
#include "backlight.h"
#include "backlight_app.h"
#include "sdcard.h"
#include "storage.h"
#include "usb_msc.h"
#include "e22.h"
#include "e22_port.h"
#include "qmc5883p.h"
#include "compass.h"
#include "mx01p.h"
#include "mx01p_port.h"
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

/* USER CODE BEGIN PV */
TX_THREAD my_thread;
/* 线程栈：静态分配放 .bss，由链接器核算总量（溢出在链接期报错）；Cortex-M 栈需 8 字节对齐 */
static uint8_t my_thread_stack[1024] __attribute__((aligned(8)));

/* LoRa（E22-400T22S）：接收环形缓冲必须是 2 的幂，满时驱动丢弃新数据 */
static uint8_t g_lora_rx_ring[1024];
static e22_t   g_lora;

/* 蓝牙（MX-01P）：环形缓冲容量必须是 2 的幂，满时驱动丢弃新数据。
   g_bt_probe 故意做成全局（非 static）：J-Link 可以按符号直接读自检结果，
   不依赖 RTT 是否在连。 */
static uint8_t g_bt_rx_ring[1024];
static mx01p_t g_bt;
mx01p_probe_t  g_bt_probe;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief first threadx test thread
 */
void my_thread_entry(ULONG thread_input){
  /* Enter into a forever loop. */
  while(1)
  {
    /* Increment thread counter. */
    // SEGGER_RTT_WriteString(0, "threadx working\r\n");
    /* Sleep for 1 second. */
    tx_thread_sleep(100);
  }
}

/* ---- LoRa(E22-400T22S) 上电自检：打印模块型号/固件/参数，便于上板确认 ---- */

static const char *lora_baud_str(e22_uart_baud_t baud)
{
  switch (baud) {
  case E22_UART_BAUD_1200:   return "1200";
  case E22_UART_BAUD_2400:   return "2400";
  case E22_UART_BAUD_4800:   return "4800";
  case E22_UART_BAUD_9600:   return "9600";
  case E22_UART_BAUD_19200:  return "19200";
  case E22_UART_BAUD_38400:  return "38400";
  case E22_UART_BAUD_57600:  return "57600";
  case E22_UART_BAUD_115200: return "115200";
  default:                   return "?";
  }
}

static const char *lora_parity_str(e22_uart_parity_t parity)
{
  switch (parity) {
  case E22_UART_PARITY_8N1: return "8N1";
  case E22_UART_PARITY_8O1: return "8O1";
  case E22_UART_PARITY_8E1: return "8E1";
  default:                  return "?";
  }
}

static uint32_t lora_air_bps(e22_air_rate_t rate)
{
  switch (rate) {
  case E22_AIR_RATE_4K8:  return 4800u;
  case E22_AIR_RATE_9K6:  return 9600u;
  case E22_AIR_RATE_19K2: return 19200u;
  case E22_AIR_RATE_38K4: return 38400u;
  case E22_AIR_RATE_62K5: return 62500u;
  case E22_AIR_RATE_2K4:
  default:                return 2400u;
  }
}

static uint8_t lora_power_dbm(e22_tx_power_t power)
{
  static const uint8_t table[4] = {22u, 17u, 14u, 10u};

  return table[(unsigned)power & 0x03u];
}

static void lora_selftest(void)
{
  static const uint16_t packet_bytes[4] = {240u, 128u, 64u, 32u};
  e22_cfg_t cfg = {
    .rx_ring = g_lora_rx_ring,
    .rx_ring_size = (uint16_t)sizeof(g_lora_rx_ring),
    .aux_cb = NULL,
    .user = NULL,
  };
  e22_params_t params;
  uint8_t pid[E22_PID_LEN];
  char type[24];
  char fw[24];
  e22_status_t st;

  st = e22_init(&g_lora, &cfg);
  if (st != E22_OK) {
    SEGGER_RTT_printf(0, "[LORA] init failed: st=%d aux_busy=%u\r\n",
                      (int)st, (unsigned)e22_is_busy(&g_lora));
    return;
  }
  SEGGER_RTT_WriteString(0, "[LORA] init ok\r\n");

  if (e22_at_query(&g_lora, "AT+DEVTYPE=?", "DEVTYPE=", type, sizeof(type), 300u) != E22_OK) {
    type[0] = '?';
    type[1] = '\0';
  }
  if (e22_at_query(&g_lora, "AT+FWCODE=?", "FWCODE=", fw, sizeof(fw), 300u) != E22_OK) {
    fw[0] = '?';
    fw[1] = '\0';
  }
  SEGGER_RTT_printf(0, "[LORA] type=%s fw=%s\r\n", type, fw);

  if (e22_read_pid(&g_lora, pid) == E22_OK) {
    SEGGER_RTT_printf(0, "[LORA] pid=%02X%02X%02X%02X%02X%02X%02X\r\n",
                      pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6]);
  }

  if (e22_read_params(&g_lora, &params) == E22_OK) {
    SEGGER_RTT_printf(0, "[LORA] addr=0x%04X netid=%u ch=%u (%u.125MHz) air=%u bps\r\n",
                      (unsigned)params.address, (unsigned)params.netid,
                      (unsigned)params.channel, (unsigned)(410u + params.channel),
                      (unsigned)lora_air_bps(params.air_rate));
    SEGGER_RTT_printf(0, "[LORA] uart=%s %s pkt=%u B pwr=%u dBm fixed=%u lbt=%u relay=%u\r\n",
                      lora_baud_str(params.uart_baud),
                      lora_parity_str(params.uart_parity),
                      (unsigned)packet_bytes[(unsigned)params.packet_len & 0x03u],
                      (unsigned)lora_power_dbm(params.tx_power),
                      (unsigned)params.fixed_tx, (unsigned)params.lbt,
                      (unsigned)params.relay);
    SEGGER_RTT_printf(0, "[LORA] wor_role=%u period=%u ms rssi_env=%u rssi_byte=%u soft=%u\r\n",
                      (unsigned)params.wor_role,
                      (unsigned)(((unsigned)params.wor_period + 1u) * 500u),
                      (unsigned)params.rssi_env, (unsigned)params.rssi_byte,
                      (unsigned)params.soft_mode);
  } else {
    SEGGER_RTT_WriteString(0, "[LORA] read params failed\r\n");
  }
}

/* ---- 蓝牙（MX-01P）上电自检：诊断量全部落进 g_bt_probe，RTT 只是其中一份拷贝 ---- */
static void bt_selftest(void)
{
  /* 手册里 AT+UART=NUM 的码值顺序，下标即码值 */
  static const char *const baud_code_name[9] = {
    "9600",  "14400", "19200",  "38400",  "57600",
    "115200", "230400", "460800", "921600",
  };
  mx01p_cfg_t cfg = {
    .rx_ring = g_bt_rx_ring,
    .rx_ring_size = (uint16_t)sizeof(g_bt_rx_ring),
    .evt_cb = NULL,
    .user = NULL,
    .ready_timeout_ms = 0u, /* 0 = 用驱动默认的 3000 ms */
  };
  const mx01p_probe_t *p = &g_bt_probe;
  const char *baud_name;
  bool ok;

  ok = mx01p_probe(&g_bt, &cfg, &g_bt_probe);

  baud_name = ((unsigned)p->baud_code < 9u) ? baud_code_name[(unsigned)p->baud_code]
                                            : "?";

  SEGGER_RTT_printf(0, "[BT] probe=%u stage=%u init=%d ready=%u ms retry=%u\r\n",
                    (unsigned)ok, (unsigned)p->stage, (int)p->init_st,
                    (unsigned)p->ready_ms, (unsigned)p->reset_retry);
  if (p->init_st != 0) {
    /* 超时基本只有三种原因：模块没供电、RES 没释放、串口波特率不匹配 */
    SEGGER_RTT_WriteString(0, "[BT] init failed: check VCC/RES/USART1 115200\r\n");
    SEGGER_RTT_printf(0, "[BT] rx_dropped=%u resp_overrun=%u\r\n",
                      (unsigned)p->rx_dropped, (unsigned)p->resp_overrun);
    return;
  }

  SEGGER_RTT_printf(0, "[BT] mac=%s(%d) name=%s(%d)\r\n", p->mac,
                    (int)p->mac_st, p->name, (int)p->name_st);
  SEGGER_RTT_printf(0, "[BT] ver=%s(%d)\r\n", p->version, (int)p->version_st);
  SEGGER_RTT_printf(0, "[BT] uart=%s code=%d(%d) adv=%d(%d) aintvl=%d ms(%d)\r\n",
                    baud_name, (int)p->baud_code, (int)p->baud_st, (int)p->adv_on,
                    (int)p->adv_st, (int)p->interval_ms, (int)p->interval_st);
  SEGGER_RTT_printf(0, "[BT] txpower_code=%d(%d)\r\n", (int)p->power_code,
                    (int)p->power_st);
  SEGGER_RTT_printf(0, "[BT] uuid s=%s n=%s w=%s (%d)\r\n", p->uuid_service,
                    p->uuid_notify, p->uuid_write, (int)p->uuid_st);
  SEGGER_RTT_printf(0, "[BT] sleep st=%d blocked=%d wake=%d\r\n", (int)p->sleep_st,
                    (int)p->sleep_blocked, (int)p->wake_st);
  SEGGER_RTT_printf(0, "[BT] loop_st=%d len=%u text=%s\r\n", (int)p->loop_st,
                    (unsigned)p->loop_len, p->loopback);
  SEGGER_RTT_printf(0, "[BT] reboot_st=%d\r\n", (int)p->reboot_st);
  SEGGER_RTT_printf(0, "[BT] link_st=%d connected=%u dev_mac=%s\r\n",
                    (int)p->link_st, (unsigned)mx01p_is_connected(&g_bt),
                    mx01p_is_connected(&g_bt) ? "see J-Link" : "-");
  SEGGER_RTT_printf(0, "[BT] at=%u fail=%u drop=%u overrun=%u\r\n",
                    (unsigned)p->at_total, (unsigned)p->at_fail,
                    (unsigned)p->rx_dropped, (unsigned)p->resp_overrun);
}

void tx_application_define(void *first_unused_memory){
  /* 任务栈为静态数组（见 PV 区），此处只传入指针与大小 */
  (void)first_unused_memory;

  /* 屏幕 SPI 发送切 DMA：在这里创建完成信号量（ili9341_init() 已在内核启动前跑完，
     那一段仍是轮询）。未配置 SPI1_TX DMA 时返回 false，发送保持轮询 */
  if (ili9341_rtos_attach()) {
    SEGGER_RTT_WriteString(0, "[LCD] SPI DMA enabled\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "[LCD] SPI DMA unavailable, polling mode\r\n");
  }

  tx_thread_create(&my_thread, "test thread",
                    my_thread_entry, 0x1234, my_thread_stack, sizeof(my_thread_stack),
                    3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* GPS 应用层：创建解析线程与事件组（ISR 侧入口为 gps_rx_push） */
  gps_init();

  /* 显示应用层：快照读取在 gps_init 之后，线程优先级低于 GPS */
  display_init();

  /* 屏幕背光：KEY_LIGHT 上升沿点亮 3 s */
  backlight_app_init();

  /* USB 设备（MSC）：必须早于 storage_init——storage 要用这里的媒体互斥量串行化
     对 TF 卡的访问，而 PC 枚举后 usb_msc 会置位"主机占用"标志把 storage 挡在外面 */
  usb_msc_init();

  /* 存储应用层：挂载 FAT 卷并做一次读写自检。
     必须在内核启动后——FatFs 的卷互斥量是 ThreadX 对象 */
  storage_init();

  /* 地磁罗盘：25 Hz 采集线程（优先级最低）。传感器已在 compass_hw_init() 里配好 */
  compass_init();
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
  MX_GPDMA1_Init();
  MX_USART2_UART_Init();
  MX_ICACHE_Init();
  MX_DCACHE1_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_SDMMC1_SD_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 模拟 USB 拔出：OTG 软断开（DCTL.SDIS=1）并把 D+ (PA12) 拉成 GPIO 输出低，
     主机立刻判定设备已拔出。等 USBX 设备栈就绪后由 usb_msc 线程把 D+ 交还给
     OTG 并打开内部上拉，主机看到一次干净的插入并重新枚举——这样每次软件复位
     （含调试器复位）都不需要手动拔插 USB 线。
     必须放在 MX_USB_OTG_FS_PCD_Init() 之后：HAL_PCD_Stop() 需要有效的 PCD 句柄 */
  usb_msc_boot_detach();

  /* 背光 PWM 先以 0% 启动：电源打开后引脚就是确定的低电平，不会闪一下 */
  bool backlight_ready = backlight_init();

  // 启动电源
  HAL_Delay(100);
  HAL_GPIO_WritePin(PW_EN_GPIO_Port, PW_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  // RTT初始化输出
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "System Boot OK\r\n");

  /* TF 卡硬件自检（只读）：十几毫秒级，结果同时留在 g_sd_probe 里给 J-Link 看 */
  sdcard_probe();
  {
    char label[12];

    for (uint32_t i = 0u; i < 11u; i++) {
      label[i] = (char)g_sd_probe.label[i];
    }
    label[11] = '\0';

    SEGGER_RTT_printf(0, "[SD] stage=%u cd=%u type=%u ver=%u rca=%u err=0x%08x\r\n",
                      (unsigned)g_sd_probe.stage, (unsigned)g_sd_probe.cd_pin,
                      (unsigned)g_sd_probe.card_type, (unsigned)g_sd_probe.card_version,
                      (unsigned)g_sd_probe.rca, (unsigned)g_sd_probe.hal_error);
    if (g_sd_probe.stage >= (uint32_t)SDCARD_STAGE_MBR_OK) {
      SEGGER_RTT_printf(0, "[SD] %u x %u B, mbr=%u part=0x%02x lba=%u fat32=%u\r\n",
                        (unsigned)g_sd_probe.block_nbr, (unsigned)g_sd_probe.block_size,
                        (unsigned)g_sd_probe.mbr_sig_ok, (unsigned)g_sd_probe.part_type,
                        (unsigned)g_sd_probe.part_lba, (unsigned)g_sd_probe.fat32_ok);
    }
    if (g_sd_probe.stage >= (uint32_t)SDCARD_STAGE_BS_OK) {
      SEGGER_RTT_printf(0, "[SD] bps=%u spc=%u rsv=%u fats=%u fatsz=%u label=%s\r\n",
                        (unsigned)g_sd_probe.bytes_per_sector,
                        (unsigned)g_sd_probe.sectors_per_cluster,
                        (unsigned)g_sd_probe.reserved_sectors,
                        (unsigned)g_sd_probe.num_fats, (unsigned)g_sd_probe.fat_size,
                        label);
    }
  }
  if (!backlight_ready) {
    SEGGER_RTT_WriteString(0, "[BL] PWM init failed\r\n");
  }
  // 屏幕初始化（约 1.3 s：复位 + 软复位 + 退出睡眠）；此时 GPS 仍按住复位，不丢数据
  if (!ili9341_init()) {
    SEGGER_RTT_WriteString(0, "[LCD] init failed\r\n");
  }
  // LC760Z初始化
  LC760Z_init();
  // LoRa(E22-400T22S)初始化 + 上电自检（型号/固件/参数打印到 RTT）
  lora_selftest();

  /* 蓝牙 MX-01P 上电自检：硬复位等 +READY → 一轮 AT 查询 → 睡眠脚验证。
     结果同时留在 g_bt_probe 里，J-Link 可以直接按符号读，不依赖 RTT 是否在看 */
  bt_selftest();

  /* 地磁 QMC5883P 上电自检：地址 → CHIPID → 16 个样本 → Set/Reset 内部自检。
     结果同时留在 g_qmc_probe 里，J-Link 可以直接读，不依赖 RTT 是否在看 */
  qmc5883p_probe();
  SEGGER_RTT_printf(0, "[MAG] stage=%u id=0x%02x n=%u err=%u stat=0x%02x\r\n",
                    (unsigned)g_qmc_probe.stage, (unsigned)g_qmc_probe.chip_id,
                    (unsigned)g_qmc_probe.samples, (unsigned)g_qmc_probe.errors,
                    (unsigned)g_qmc_probe.status);
  SEGGER_RTT_printf(0, "[MAG] last x=%d y=%d z=%d  min/max x=%d..%d y=%d..%d z=%d..%d\r\n",
                    (int)g_qmc_probe.x_last, (int)g_qmc_probe.y_last,
                    (int)g_qmc_probe.z_last, (int)g_qmc_probe.x_min,
                    (int)g_qmc_probe.x_max, (int)g_qmc_probe.y_min,
                    (int)g_qmc_probe.y_max, (int)g_qmc_probe.z_min,
                    (int)g_qmc_probe.z_max);
  SEGGER_RTT_printf(0, "[MAG] ctrl=0x%02x/0x%02x self=%d/%d/%d mg=%d\r\n",
                    (unsigned)g_qmc_probe.reg_ctrl1,
                    (unsigned)g_qmc_probe.reg_ctrl2,
                    (int)g_qmc_probe.st_dx, (int)g_qmc_probe.st_dy,
                    (int)g_qmc_probe.st_dz, (int)g_qmc_probe.last_mg_l1);

  /* 罗盘应用层：按 25 Hz 采集的要求重配传感器（ODR 100 Hz / ±8 G）。
     自检那一次用的是 200 Hz 临时配置，这里以应用配置为准 */
  compass_hw_init();

  // start threadx kernel
  tx_kernel_enter();
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == KEY_LIGHT_Pin){
    /* ISR 只发事件：点亮与 3 s 计时都在 backlight 线程里做 */
    backlight_app_button_isr();
  }
  else if(GPIO_Pin == LORA_AUX_Pin){
    /* AUX 上升沿 = 模块回到空闲 */
    e22_port_aux_isr();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == LORA_AUX_Pin){
    /* AUX 下降沿 = 模块开始忙（收包前 / 发包 / 切模式） */
    e22_port_aux_isr();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart->Instance == USART2){
    /* 半传输事件：既不是完整帧，也不能 rearm，直接忽略，等 IDLE / TC */
    if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT){
      return;
    }
    /* ISR 只搬运：Size 是本次 burst 的有效字节数（DMA_NORMAL 下 HAL 已 abort DMA
       并置 RxState=READY，所以紧接着 rearm 是合法的）。不能用整个缓冲当字符串，
       否则会把上一帧更长的残留一起读进来；解析与打印都在 gps 线程里做 */
    gps_rx_push(LC760Z_rx_data, Size);
    LC760Z_uart_start_receive();
  }
  else if(huart->Instance == LPUART1){
    /* ISR 只把本次 burst 拷进 LoRa 环形缓冲并 rearm */
    if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT){
      return;
    }
    e22_port_rx_event(Size);
  }
  else if(huart->Instance == USART1){
    /* 蓝牙：ISR 里只做分流（控制行 → 应答邮箱/事件，数据 → 环形缓冲）并 rearm */
    if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT){
      return;
    }
    mx01p_port_rx_event(Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART2){
    /* TODO: 如果重启接收失败将永远关闭接收，待后续完善 */
    LC760Z_uart_start_receive();
  }
  else if(huart->Instance == LPUART1){
    /* ORE/帧错误：复位接收并 rearm，最多毁掉当前一包 */
    e22_port_uart_error();
  }
  else if(huart->Instance == USART1){
    /* 蓝牙同理：复位接收并 rearm，不 panic、不打印 */
    mx01p_port_uart_error();
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
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
