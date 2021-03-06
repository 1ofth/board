/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/app_ethernet.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "lwip/opt.h"
#include "main.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "ethernetif.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USE_DHCP
#define MAX_DHCP_TRIES  4
__IO uint8_t DHCP_state;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Notify the User about the nework interface config status
  * @param  netif: the network interface
  * @retval None
  */
void User_notification(struct netif *netif)
{
  if (netif_is_up(netif))
 {
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
#endif /* USE_DHCP */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
    /* Turn On LED 3 to indicate ETH and LwIP init success*/
//    BSP_LED_On(LED3);
}
 else
  {
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
#endif  /* USE_DHCP */

    /* Turn On LED 4 to indicate ETH and LwIP init error */
//    BSP_LED_On(LED4);
      HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_notify_conn_changed(struct netif *netif)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;

  if(netif_is_link_up(netif))
  {
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
#else
    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif /* USE_DHCP */
    printf("netif_is_link_up is true!\r\n");
    netif_set_addr(netif, &ipaddr , &netmask, &gw);
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(netif);

  //   BSP_LED_Off(LED4);
      HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
//    BSP_LED_On(LED3);
      HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
  }
  else
  {
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
#endif /* USE_DHCP */
    printf("netif_is_link_up is false!\r\n");

    /*  When the netif link is down this function must be called.*/
    netif_set_down(netif);

//    BSP_LED_On(LED4);
      HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
//     BSP_LED_Off(LED3);
      HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
  }
}

#ifdef USE_DHCP
/**
  * @brief  DHCP Process worker called by tcpip thread
  * @param  argument: network interface
  * @retval None
  */
void dhcp_do(struct netif *netif)
{
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
    uint32_t IPaddress;

    switch (DHCP_state)
    {
    case DHCP_START:
      {
        netif->ip_addr.addr = 0;
        netif->netmask.addr = 0;
        netif->gw.addr = 0;
        IPaddress = 0;
        dhcp_start(netif);
        DHCP_state = DHCP_WAIT_ADDRESS;
      }
      break;

    case DHCP_WAIT_ADDRESS:
      {
        /* Read the new IP address */
        IPaddress = netif->ip_addr.addr;

        if (IPaddress!=0)
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;

          /* Stop DHCP */
          dhcp_stop(netif);

//    BSP_LED_On(LED3);
            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
        }
        else
        {
          /* DHCP timeout */
          if (netif->dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;

            /* Stop DHCP */
            dhcp_stop(netif);

            /* Static address used */
            IP4_ADDR(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
            IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
            IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
            netif_set_addr(netif, &ipaddr , &netmask, &gw);

              //    BSP_LED_On(LED4);
              HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
          }
        }
      }
      break;

    default: break;
    }
}

/**
  * @brief  DHCP Process
* @param  argument: network interface
  * @retval None
  */
void DHCP_thread(void const * argument)
{
  for (;;)
  {
      tcpip_callback((tcpip_callback_fn) dhcp_do, (void *) argument);
      /* wait 250 ms */
      osDelay(250);
  }
}
#endif  /* USE_DHCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
