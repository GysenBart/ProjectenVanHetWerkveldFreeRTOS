/*
 * mqtt_global.h
 *
 *  Created on: 20 mei 2023
 *      Author: bart-
 */

#ifndef INC_MQTT_GLOBAL_H_
#define INC_MQTT_GLOBAL_H_

void mqtt_init(mqtt_client_t* mqtt_client);
void mqtt_connect_broker();
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_sub_request_cb(void *arg, err_t result);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
uint8_t publish_message(mqtt_client_t *client ,const void * pub_payload , const char *topic, void *arg);
static void mqtt_pub_request_cb(void *arg, err_t result);
uint8_t Check_client_connected(void);

#endif /* INC_MQTT_GLOBAL_H_ */
