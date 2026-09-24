/**
 * @file LC760Z.h
 * @author YangKY
 * @brief LC760Z头文件
 */

#ifndef _LC760Z_H_
#define _LC760Z_H_

#include <stdint.h>

#define LC760Z_RX_DATA_SIZE 1024

typedef enum{
  LC760Z_OK,
  LC760Z_ERROR
}LC760Z_ret_t;

extern uint8_t LC760Z_rx_data[LC760Z_RX_DATA_SIZE];

LC760Z_ret_t LC760Z_init(void);

/*以下是由用户实现的接口函数*/

LC760Z_ret_t LC760Z_uart_start_receive(void);
void LC760Z_device_on(void);

#endif /*_LC760Z_H_*/