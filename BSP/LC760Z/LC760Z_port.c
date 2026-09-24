/**
 * @file LC760Z_port.c
 * @author YangKY
 * @brief LC760Z接口函数实现
 */

#include "LC760Z.h"
#include "stm32u5xx_hal.h"
#include "usart.h"
#include "main.h"

/**
 * @brief 开启LC760Z的串口数据接收
 * @return 开启成功(LC760Z_OK)或失败(LC760Z_ERROR)
 */
LC760Z_ret_t LC760Z_uart_start_receive(void){
  LC760Z_ret_t ret = LC760Z_OK;
  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart2, LC760Z_rx_data, LC760Z_RX_DATA_SIZE) == HAL_OK){
    ret = LC760Z_OK;
  }
  else{
    ret = LC760Z_ERROR;
  }
  return ret;
}

void LC760Z_device_on(void){
  HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, GPIO_PIN_SET);
}
