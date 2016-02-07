#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "string.h"
#include "battery-socket.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief serve tcp connection
  * @param conn: connection socket
  * @retval None
  */
void broadcastVoltage(int conn)
{
  uint32_t lastTime = 0;
//  memcpy(voltagePacket, getVoltagePacket(), sizeof(uint32_t) * 1000);


  /* Read in the request */
//  read(conn, recv_buffer, buflen);

  int goodWrite = 1;
  while(goodWrite > 0)
  {
	  if(voltageStruct.bufferFirstHalf[voltageStruct.bufferLength - 2] > lastTime){
		  goodWrite = send(conn, (uint32_t *)voltageStruct.bufferFirstHalf,
				  (size_t)(sizeof(uint32_t) * voltageStruct.bufferLength), 1);
		  lastTime = voltageStruct.bufferFirstHalf[voltageStruct.bufferLength - 2];
	  }
	  if(voltageStruct.bufferLastHalf[voltageStruct.bufferLength - 2] > lastTime){
		  goodWrite = send(conn, (uint32_t *)voltageStruct.bufferLastHalf,
				  (size_t)(sizeof(uint32_t) * voltageStruct.bufferLength), 1);
		  lastTime = voltageStruct.bufferLastHalf[voltageStruct.bufferLength - 2];
	  }
  }

  /* Close connection socket */
  close(conn);
}


//void broadcastVoltage2(int conn)
//{
//  uint32_t * voltagePacket = getVoltagePacket();
////  memcpy(voltagePacket, getVoltagePacket(), sizeof(uint32_t) * 1000);
//
//
//  /* Read in the request */
////  read(conn, recv_buffer, buflen);
//
//  int goodWrite = 1;
//  while(goodWrite > 0)
//  {
//	  goodWrite = send(conn, (uint32_t *)voltagePacket, (size_t)(sizeof(uint32_t) * ETHERNET_BUFFER_LENGTH), 1);
//	  voltagePacket = getVoltagePacket();
//	  osDelay(100);
//  }
//
//  /* Close connection socket */
//  close(conn);
//}

/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
void voltage_server_socket()
{
  int sock, newconn, size;
  struct sockaddr_in address, remotehost;

 /* create a TCP socket */
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
  {
    return;
  }
  
  /* bind to port 80 at any interface */
  address.sin_family = AF_INET;
  address.sin_port = htons(80);
  address.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
  {
    return;
  }
  
  /* listen for incoming connections (TCP listen backlog = 5) */
  listen(sock, 5);
  
  size = sizeof(remotehost);
  
  while (1) 
  {
    newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
    broadcastVoltage(newconn);
  }
}
