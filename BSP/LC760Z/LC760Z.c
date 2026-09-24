/**
 * @file LC760.c
 * @author YangKY
 * @brief LC760Z驱动文件实现
 */

#include "LC760Z.h"

uint8_t LC760Z_rx_data[LC760Z_RX_DATA_SIZE] = {0};

LC760Z_ret_t LC760Z_init(void){
  LC760Z_ret_t ret = LC760Z_OK;
  ret = LC760Z_uart_start_receive();
  LC760Z_device_on();
  return ret;
}