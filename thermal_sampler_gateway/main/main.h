#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_sleep.h"
#include "sys/time.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "sys/socket.h"
#include "arpa/inet.h"
#include "netinet/in.h"
#include "sys/types.h"

/*
16 -> CSN
17 -> CE
14 -> SCK
13 -> MOSI
12 -> MISO
22 -> IRQ
*/
#define ZERO_ARRAY {0,}

#define LED_PIN GPIO_NUM_2
#define NRF_CSN_PIN 16
#define NRF_CE_PIN 17
#define NRF_SCK_PIN 14
#define NRF_MOSI_PIN 13
#define NRF_MISO_PIN 12
#define NRF_IRQ_PIN 22

#define WIFI_SSID "MyRoom"
#define WIFI_PASS "9876543210"
#define SERVER_ADDR "192.168.0.3"
#define SERVER_PORT 8000

spi_device_interface_config_t spi_device_conf;
spi_device_handle_t spi_device_handle;



typedef struct
{
	uint64_t device_address;
	int8_t temperature;
	uint8_t humidity;
	uint16_t battery_voltage;
} DataPacket;

typedef struct {
	uint64_t device_address;
	int8_t temperature;
	uint8_t humidity;
	uint16_t battery_voltage;
	uint32_t time;
}ReceivedDataPacket;

#endif