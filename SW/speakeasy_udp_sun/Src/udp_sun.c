/*
 * udp_user.c
 *
 *  Created on: Oct 9, 2018
 *      Author: David Tilly
 */

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "udp_sun.h"
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Private variables */
struct udp_pcb *upcb;
extern uint8_t bufferReceived[10];
extern uint16_t txBuffer[10 * (BUFF_SIZE + 1)];
extern uint16_t playbackControlRx;
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
//   struct udp_pcb *upcb;
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

void udp_scratch_send(uint16_t * txData, uint16_t count) {
	struct pbuf *p;
	uint16_t length = (sizeof(txData[0]) * count);
	p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_POOL);
	err_t err;

	if (p != NULL) {
		// copy txData into pbuf
		pbuf_take(p, (uint16_t *) txData, length);
		err = udp_send(upcb, p);
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, err != ERR_OK);
		pbuf_free(p);
	}
}

void udp_scratch_send_audio(uint16_t * txData, uint16_t count, uint16_t index) {
	struct pbuf *p;
//	uint16_t length = (sizeof(txData[0]) * count) + 1;
	uint16_t length = (sizeof(txData[0]) * (count + 1));
	p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_POOL);
	err_t err;

	if (p != NULL) {
		// copy txData into pbuf
		uint16_t indexedData[BUFF_SIZE + 1];
		indexedData[0] = index;
		memcpy(&indexedData[1], txData, BUFF_SIZE * sizeof(txData[0]));
		pbuf_take(p, (uint16_t *) indexedData, length);
		err = udp_send(upcb, p);
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, err != ERR_OK);
		pbuf_free(p);
	}
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port) {
    uint16_t bufferCheck = ((uint16_t *) p->payload)[0];
    playbackControlRx = ((uint16_t *) p->payload)[1];
    bufferReceived[bufferCheck] = 1;
    if (bufferCheck < 1) {
    	bufferCheck += 10;
    }
    if (!bufferReceived[bufferCheck - 1]) {
		udp_scratch_send_audio(&txBuffer[(bufferCheck - 1) * BUFF_SIZE], BUFF_SIZE, bufferCheck - 1);
    }
	pbuf_free(p);
}
/* USER CODE END Private function definitions */

