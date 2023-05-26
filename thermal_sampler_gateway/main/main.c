#include "main.h"

#include "nrf_driver.h"

static QueueHandle_t nrf_irq_interrupt_queue = NULL;
static QueueHandle_t data_packet_queue = NULL;

spi_device_interface_config_t spi_device_conf = ZERO_ARRAY; // SPI device interface configuration
spi_device_handle_t spi_device_handle = ZERO_ARRAY;			// SPI device handle

static void IRAM_ATTR gpio_interrupt_handler(void *arg)
{
	uint32_t gpio_pin = (uint32_t)arg;

	if (gpio_pin == NRF_IRQ_PIN)
	{
		uint8_t data = (uint8_t)gpio_pin;
		xQueueSendFromISR(nrf_irq_interrupt_queue, &data, NULL);
	}
}

void init_queues()
{
	nrf_irq_interrupt_queue = xQueueCreate(5, sizeof(uint8_t));
	data_packet_queue = xQueueCreate(50, sizeof(ReceivedDataPacket));
	ESP_LOGI(__func__, "QUEUE INIT DONE!");
}

static void nrf_data_receiver_task(void *arg)
{
	uint8_t data_received;
	ESP_LOGI(__func__, "Receiver task started!");
	while (1)
	{
		if (xQueueReceive(nrf_irq_interrupt_queue, &data_received, portMAX_DELAY) != pdPASS)
		{
		}
		else
		{
			// ESP_LOGI(__func__, "GOT interrupt!");
			if (NRF_ReadRegister(NRF_REG_STATUS) & (1 << 6))
			{
				NRF_CE_LOW;
				NRF_WriteRegister(NRF_REG_STATUS, (1 << 6));		   // clearing IRQ
				uint8_t data_len = NRF_ReadRegister(NRF_REG_RX_PW_P0); // getting data length
				uint8_t data_read[100] = ZERO_ARRAY;
				NRF_ReadRxBuffer(data_read, data_len);
				DataPacket dp = ZERO_ARRAY;
				memcpy(&dp, data_read, sizeof(DataPacket));
				ReceivedDataPacket dp2 = ZERO_ARRAY;
				dp2.battery_voltage = dp.battery_voltage;
				dp2.device_address = dp.device_address;
				dp2.humidity = dp.humidity;
				dp2.temperature = dp.temperature;
				time(&dp2.time);
				// dp2.time += (1800 * 11);
				ESP_LOGI(__func__, "Read data %d %d T:%d H:%d", (data_len), dp2.time, dp2.temperature, dp2.humidity);
				xQueueSend(data_packet_queue, &dp2, 0);
				NRF_FlushRxBuffer();
				NRF_Execute_Rx();
			}

			gpio_set_level(LED_PIN, 1);
			vTaskDelay(100 / portTICK_RATE_MS);
			gpio_set_level(LED_PIN, 0);
		}
	}
}

int send_data_to_server(uint8_t *data, ssize_t data_length)
{
	int s = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in info = ZERO_ARRAY;
	info.sin_addr.s_addr = inet_addr(SERVER_ADDR);
	info.sin_family = AF_INET;
	info.sin_port = htons(SERVER_PORT);

	int con = connect(s, (struct sockaddr *)&info, sizeof(info));

	int data_sent = 0;

	data_sent = send(s, data, data_length, 0);
	shutdown(s, SHUT_RDWR);
	close(s);

	return data_sent;
}

static void sender_task(void *arg)
{
	ReceivedDataPacket packet_array[60] = ZERO_ARRAY;
	uint8_t packet_ctr = 0;
	while (1)
	{
		packet_ctr = 0;
		bzero(packet_array, sizeof(packet_array));
		while (xQueueReceive(data_packet_queue, &packet_array[packet_ctr], 0) == pdPASS)
		{
			packet_ctr++;
		}

		ESP_LOGI(__func__, "Found %d packets in queue", packet_ctr);

		// if anything was actually received
		if (packet_ctr > 0)
		{
			int stat = send_data_to_server(packet_array, sizeof(ReceivedDataPacket) * packet_ctr);
			ESP_LOGI(__func__, "Send Stat %d", stat);
		}
		// will run every 5 minutes
		vTaskDelay((60 * 1000 * 5) / portTICK_RATE_MS);
	}
}

void initialize_spi()
{
	esp_err_t ret;

	// SPI bus
	spi_bus_config_t buscfg = {
		0,
	};

	buscfg.miso_io_num = NRF_MISO_PIN;
	buscfg.mosi_io_num = NRF_MOSI_PIN;
	buscfg.sclk_io_num = NRF_SCK_PIN;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 0;
	buscfg.flags = SPICOMMON_BUSFLAG_MASTER;

	// Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

	ESP_LOGI(__func__, "Starting SPI INIT");

	spi_device_conf.command_bits = 0;
	spi_device_conf.address_bits = 0;
	spi_device_conf.dummy_bits = 0;
	spi_device_conf.queue_size = 1;
	spi_device_conf.clock_speed_hz = SPI_MASTER_FREQ_10M / 10;
	spi_device_conf.mode = 0;
	spi_device_conf.spics_io_num = -1;
	spi_device_conf.flags = SPI_DEVICE_HALFDUPLEX | SPI_TRANS_MODE_OCT;

	ret = spi_bus_add_device(HSPI_HOST, &spi_device_conf, &spi_device_handle);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
	ESP_LOGI(__func__, "SPI INIT DONE!");
}

void initialize_gpio()
{
	gpio_config_t gp = ZERO_ARRAY;
	ESP_LOGI(__func__, "Starting GPIO Init");

	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_install_isr_service(0));
	ESP_LOGI(__func__, "ISR Started");

	// LED
	gp.intr_type = GPIO_INTR_DISABLE;
	gp.mode = GPIO_MODE_OUTPUT;
	gp.pin_bit_mask = BIT(LED_PIN);
	gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gp.pull_up_en = GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gp));
	gpio_set_level(LED_PIN, 0);

	// NRF CSN
	gp.intr_type = GPIO_INTR_DISABLE;
	gp.mode = GPIO_MODE_OUTPUT;
	gp.pin_bit_mask = BIT(NRF_CSN_PIN);
	gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gp.pull_up_en = GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gp));
	gpio_set_level(NRF_CSN_PIN, 1);

	// NRF CE
	gp.intr_type = GPIO_INTR_DISABLE;
	gp.mode = GPIO_MODE_OUTPUT;
	gp.pin_bit_mask = BIT(NRF_CE_PIN);
	gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gp.pull_up_en = GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gp));
	gpio_set_level(NRF_CE_PIN, 0);
}

void start_nrf_irq(void)
{
	gpio_config_t gp = ZERO_ARRAY;

	// NRF IRQ
	gp.intr_type = GPIO_INTR_NEGEDGE; // GPIO_INTR_LOW_LEVEL;
	gp.mode = GPIO_MODE_INPUT;
	gp.pin_bit_mask = BIT(NRF_IRQ_PIN);
	gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gp.pull_up_en = GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gp));
	ESP_LOGI(__func__, "Adding handler");
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(NRF_IRQ_PIN, gpio_interrupt_handler, (void *)NRF_IRQ_PIN));
	ESP_LOGI(__func__, "Handler added");

	ESP_LOGI(__func__, "GPIO INIT DONE!");
}

void time_sync_notification_cb(struct timeval *tv)
{
	time_t now;
	time(&now);
	now += (1800 * 11);
	ESP_LOGI(__func__, "%s", ctime(&now));
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		esp_wifi_connect();
		ESP_LOGI(__func__, "retry to connect to the AP");
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(__func__, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_setservername(0, "pool.ntp.org");
		sntp_set_time_sync_notification_cb(time_sync_notification_cb);
		sntp_init();
	}
}

void wifi_init_sta(void)
{

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&event_handler,
														NULL,
														&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
														IP_EVENT_STA_GOT_IP,
														&event_handler,
														NULL,
														&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = WIFI_SSID,
			.password = WIFI_PASS,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(__func__, "wifi_init_sta finished.");
}

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	init_queues();
	wifi_init_sta();
	initialize_gpio();
	initialize_spi();
	NRF_WriteRegister(NRF_REG_STATUS, 0xff);
	NRF_Rx_Init();

	uint8_t data[0x1c] = ZERO_ARRAY;
	NRF_SetPayloadLength(sizeof(DataPacket), 0);
	NRF_WriteRegister(NRF_REG_STATUS, 0xff); // clearing everything
	start_nrf_irq();
	xTaskCreate(nrf_data_receiver_task, "nrf_data_receiver_task", 8192, NULL, 0, NULL);
	xTaskCreate(sender_task, "sender_task", 8192, NULL, 0, NULL);
	NRF_Execute_Rx();

	while (1)
	{
		vTaskDelay(5000 / portTICK_RATE_MS);
	}
}
