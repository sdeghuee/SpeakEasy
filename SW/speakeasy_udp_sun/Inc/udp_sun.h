/*
 * udp_user.h
 *
 *  Created on: Oct 9, 2018
 *      Author: David Tilly
 */

#ifndef UDP_USER_H_
#define UDP_USER_H_

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */


/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN Private function prototypes */
void udp_scratch_connect();
void udp_receive_init();
void udp_scratch_send(uint16_t * txData, uint16_t count, uint16_t index);
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port);
/* USER CODE END Private function prototypes */

#endif /* UDP_USER_H_ */
