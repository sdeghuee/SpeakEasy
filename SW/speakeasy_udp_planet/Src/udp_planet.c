/*
 * udp_user.c
 *
 *  Created on: Oct 9, 2018
 *      Author: David Tilly
 */

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "udp_planet.h"
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Private variables */
struct udp_pcb *upcb;
extern uint8_t rxDone;
extern uint16_t * rxData;
extern uint16_t playbackControl;
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
   ip_addr_t destIPaddr;

   /* Create a new UDP control block  */
   upcb = udp_new();

   if (upcb)
   {
	   IP4_ADDR(&destIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP4_ADDR_ANY, UDP_SERVER_PORT);
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_receive_callback, NULL);
      }
   }
}

void udp_scratch_send(uint16_t * txData, uint16_t count) {
	struct pbuf *p;
	uint16_t length = sizeof(txData[0]) * count;
	p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_POOL);
	err_t err;

	if (p != NULL) {
		// copy txData into pbuf
		pbuf_take(p, (uint16_t *) txData, length);
		err = udp_send(upcb, p);
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, err != ERR_OK);
		pbuf_free(p);
	}
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port) {
	rxData = p->payload;
    rxDone = 1;

    if (rxData[1] != NOP) {
		uint16_t sendback[2];
		err_t err = udp_connect(upcb, addr, port);
		memcpy(sendback, &rxData[0], 2);
		sendback[1] = playbackControl;	// playback control
		playbackControl = NOP;
		udp_scratch_send(&sendback[0], 2);
		udp_disconnect(upcb);
    }

//    HAL_GPIO_TogglePin(LED_Amber_GPIO_Port, LED_Amber_Pin);
	pbuf_free(p);
}
/* USER CODE END Private function definitions */

