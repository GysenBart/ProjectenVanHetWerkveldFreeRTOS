/*
 * mqtt_global.c
 *
 *  Created on: 20 mei 2023
 *      Author: bart-
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "mqtt.h"
#include "mqtt_global.h"

#define BROKER_IP "192.168.0.164"

static ip_addr_t mqtt_ip;
mqtt_client_t* mqtt_client;
extern UART_HandleTypeDef huart3;

char RxBuffer[100];

static const struct mqtt_connect_client_info_t mqtt_client_info =
{
  "test", /* ClientId */
  "mqtt", /* user */
  "mqtt", /* pass */
  0,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

void mqtt_init(mqtt_client_t* mqtt_client)
{
	  char uartmsg[50];
	  err_t err;
	  uint8_t IP_ADDRESS[4];
	  /* This can be modified to the arguments */
	  /* IP addresses initialization */
	  IP_ADDRESS[0] = 192;
	  IP_ADDRESS[1] = 168;
	  IP_ADDRESS[2] = 0;
	  IP_ADDRESS[3] = 164;

	  IP4_ADDR(&mqtt_ip, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);

	  err = mqtt_client_connect(mqtt_client, &mqtt_ip, MQTT_PORT, mqtt_connection_cb, 0, &mqtt_client_info);

	  /* For now just print the result code if something goes wrong */
	  if(err != ERR_OK)
	  {
		const char uartmsg[] = "mqtt_connect failed\n";
	    printf("mqtt_connect failed return: %d\n", err);
	    HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);
	    HAL_UART_Transmit(&huart3, (uint8_t)err, sizeof(err),100);
	  }
	  else
	  {
		char uartmsg[] = "mqtt_connect succeed\n";
		printf("mqtt_connect succeed");
		HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);

		  const char *pub_payload= "INIT SUCCES";
		  const char *topic = "DEBUG";
		  u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
		  u8_t retain = 0; /* No don't retain such crappy payload... */
		  err = mqtt_publish(mqtt_client, topic, pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, NULL);
		  if(err != ERR_OK)
		  {
		  printf("Publish err: %d\n", err);
		  HAL_UART_Transmit(&huart3, "Publish error", sizeof("publish error"), 250);
		  }
	  }
}

void mqtt_connect_broker()
{

//  struct mqtt_connect_client_info_t ci;
  err_t err;
  uint8_t IP_ADDRESS[4];

  /* This can be modified to the arguments */
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 0;
  IP_ADDRESS[3] = 164;

  IP4_ADDR(&mqtt_ip, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */

  err = mqtt_client_connect(mqtt_client, &mqtt_ip, MQTT_PORT, mqtt_connection_cb, 0, &mqtt_client_info);

  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK)
  {
    printf("mqtt_connect failed return: %d\n", err);
  }
  else
  {
	printf("mqtt_connect succeed");
  }

}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{

  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED)
  {
	  char uartmsg[] = "mqtt_connection_cb: Successfully connected\n";
    printf("mqtt_connection_cb: Successfully connected\n");
    HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 0, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, "A", 0, mqtt_sub_request_cb, arg);
    err = mqtt_subscribe(client, "B", 0, mqtt_sub_request_cb, arg);
    err = mqtt_subscribe(client, "C", 0, mqtt_sub_request_cb, arg);
    err = mqtt_subscribe(client, "D", 0, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
    	char uartmsg[] = "mqtt_subscribe failed\n";
      printf("mqtt_subscribe return: %d\n", err);
      HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);

    }
  }
  else
  {
	  char uartmsg[] = "mqtt_connection_cb: Disconnected\n";
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
    HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);

    /* Its more nice to be connected, so try to reconnect */
    mqtt_connect_broker();

  }
}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
}

/* The idea is to demultiplex topic and create some reference to be used in data callbacks
   Example here uses a global variable, better would be to use a member in arg
   If RAM and CPU budget allows it, the easiest implementation might be to just take a copy of
   the topic string and use it in mqtt_incoming_data_cb
*/
static int inpub_id;
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* Relais 1*/
    inpub_id = 1;
  } else if(topic[0] == 'B'){
    /* Relais 2 */
    inpub_id = 2;
  } else if(topic[0] == 'C'){
	/* Relais 3 */
	inpub_id = 3;
  } else if(topic[0] == 'D'){
	/* Relais 4 */
	inpub_id = 4;
  }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	uint8_t i = 0;
  printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);

  if(flags & MQTT_DATA_FLAG_LAST)
  {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0)
    {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0)
      {
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      }
    }
    else if(inpub_id == 1)
    {
      /* Call an 'A' function... */
    	if (data[0] == '1')
    	{
    		HAL_GPIO_WritePin(Relais_1_GPIO_Port, Relais_1_Pin,GPIO_PIN_RESET);
    	}
    	else
    	{
    		HAL_GPIO_WritePin(Relais_1_GPIO_Port, Relais_1_Pin,GPIO_PIN_SET);
    	}

    }

    else if(inpub_id == 2)
    {
      /* Call an 'B' function... */
    	if (data[0] == '1')
    	{
    		HAL_GPIO_WritePin(Relais_2_GPIO_Port, Relais_2_Pin,GPIO_PIN_RESET);
    	}
    	else
    	{
    		HAL_GPIO_WritePin(Relais_2_GPIO_Port, Relais_2_Pin,GPIO_PIN_SET);
    	}

    }
    else if(inpub_id == 3)
    {
      /* Call an 'C' function... */
    	if (data[0] == '1')
    	{
    		HAL_GPIO_WritePin(Relais_3_GPIO_Port, Relais_3_Pin,GPIO_PIN_RESET);
    	}
    	else
    	{
    		HAL_GPIO_WritePin(Relais_3_GPIO_Port, Relais_3_Pin,GPIO_PIN_SET);
    	}

    }
    else if(inpub_id == 4)
    {
      /* Call an 'D' function... */
    	if (data[0] == '1')
    	{
    		HAL_GPIO_WritePin(Relais_4_GPIO_Port, Relais_4_Pin,GPIO_PIN_RESET);
    	}
    	else
    	{
    		HAL_GPIO_WritePin(Relais_4_GPIO_Port, Relais_4_Pin,GPIO_PIN_SET);
    	}

    }
    else
    {
      //printf("mqtt_incoming_data_cb: Ignoring payload...\n");
    }
  }
  else
  {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
  while (data[i] != 0)
  {
	  RxBuffer[i] = data[i];
	  i++;
  }

}

uint8_t publish_message(mqtt_client_t *client ,const void * pub_payload , const char *topic, void *arg)
{
  err_t err;
  u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, topic, pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, pub_payload);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
    HAL_UART_Transmit(&huart3, "Publish error", sizeof("publish error"), 250);

  }
  return err;
}

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{

  if(result != ERR_OK)
  {
	  char uartmsg[] = "Publisch failed\n";
    printf("Publish result: %d\n", result);
    HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);
    while (1)
    {

    }
  }
  else
  {
	  char uartmsg[] = "Message published\n";
	printf("Message Published \n");
	HAL_UART_Transmit(&huart3, uartmsg, sizeof(uartmsg), 100);
  }
}

uint8_t Check_client_connected(void)
{
	uint8_t status = 0;
	status = mqtt_client_is_connected(mqtt_client);
	return status;
}
