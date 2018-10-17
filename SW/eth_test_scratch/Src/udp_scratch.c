/*
 * udp_scratch.c
 *
 *  Created on: Oct 9, 2018
 *      Author: 477grp2
 */

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "udp_scratch.h"
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Private variables */
uint8_t txData[100];
struct udp_pcb *upcb;
/* USER CODE END Private variables */


/* Private function definitions ----------------------------------------------*/

/* USER CODE BEGIN Private function definitions */
void udp_scratch_connect() {
	ip_addr_t destIPaddr;
	err_t err;

	upcb = udp_new();

	if (upcb != NULL) {
		IP4_ADDR(&destIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
		err = udp_connect(upcb, &destIPaddr, UDP_SERVER_PORT);
		if (err == ERR_OK) {
			// set receive callback for udp object
			udp_recv(upcb, udp_receive_callback, NULL);
		}
	}
}

void udp_receive_init(void)
{
   struct udp_pcb *upcb;
   err_t err;

   /* Create a new UDP control block  */
   upcb = udp_new();

   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_receive_callback, NULL);
      }
   }
}

void udp_scratch_send(char * txData) {
	struct pbuf *p;
	p = pbuf_alloc(PBUF_TRANSPORT, strlen((char*) txData), PBUF_POOL);
	err_t err;
	if (p != NULL) {
		// copy txData into pbuf
		pbuf_take(p, (char*) txData, strlen((char*) txData));
		err = udp_send(upcb, p);
		pbuf_free(p);
		if (err != ERR_OK) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
		}
	}
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port) {
    char * ptr = p->payload;
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	i2c_play();
	pbuf_free(p);
}
/* USER CODE END Private function definitions */
