#include "comm_layer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "string.h"

char nodeID[10] = {0};
char mon_topic[100] = {0};
char setting_topic[100] = {0};
char sub_topic[100] = {0};

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_21)
#define RXD_PIN (GPIO_NUM_20)

esp_mqtt_client_handle_t mqtt_client;
extern esp_mqtt_client_handle_t s_client;

void rx_task(void *arg) ;
static esp_err_t parse_stm32_packet(char *str, uint16_t len);
static void printJsonObject(cJSON *obj);

esp_err_t uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    if(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) {
        return ESP_FAIL;
    } else
    {
        ESP_LOGI("SMARTTRAK_MSG", "Driver Install Successfully...");
    }


    if(uart_param_config(UART_NUM_1, &uart_config) != ESP_OK) {
        return ESP_FAIL;
    }else
    {
        ESP_LOGI("SMARTTRAK_MSG", "Config Done Successfully...");
    }


    if(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return ESP_FAIL;
    }else
    {
        ESP_LOGI("SMARTTRAK_MSG", "UART Pin Set Successfully...");
    }


    return ESP_OK;
}

void rx_task(void *arg) {
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t data[1000] = {0};
    uint8_t length = 0;
    while (1)
    {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length));
        int length = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000/portTICK_PERIOD_MS);
        if (length > 0)
        {
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", length, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, length, ESP_LOG_INFO);

            uint8_t flag = parse_stm32_packet((char *)data, length);
            printf("Recieved Packet:\n%s\n", data);

            if (*nodeID != 0) {
                if (flag == 1)
                {
                    esp_mqtt_client_publish(s_client, mon_topic, (char *)data, length, 1, 0);
                }
                if (flag == 2)
                {
                    esp_mqtt_client_publish(s_client, setting_topic, (char *)data, length, 1, 0);
                }
			}

            data[length] = 0;
        }
    }
    free(data);
}

int sendData(const char *logName, const char *data) {
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void tx_task(void *arg) {
    static const char *TX_TASK_TAG = "TX_TAG";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sendData(TX_TASK_TAG, "Hello SmartTRAK\n");
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }

}

static void printJsonObject(cJSON *obj) {
    cJSON *child = obj->child;
    while (child) {
        printf("%s: ", child->string);

        // Check the type of the value
        if (cJSON_IsNumber(child)) {
            printf("%f\n", child->valuedouble);
        } else if (cJSON_IsString(child)) {
            printf("%s\n", child->valuestring);
        }

        child = child->next;
    }
}

static int parse_stm32_packet(char *str, uint16_t len) {
    uint8_t ret = 0;
    cJSON *json = cJSON_Parse(str);
    // Check if parsing was successful
    if (json == NULL) {
        fprintf(stderr, "Error parsing JSON: %s\n", cJSON_GetErrorPtr());
        return ESP_FAIL;
    }

    // Access specific values in the JSON structure
    cJSON *buildDate = cJSON_GetObjectItem(json, "BuildDate");
    cJSON *res = cJSON_GetObjectItem(json, "Res");
    cJSON *jsonnodeID = cJSON_GetObjectItem(json, "NodeID");
    if (*nodeID == 0) {
    	snprintf(nodeID, 10, "%s", jsonnodeID->valuestring);
    	memset(mon_topic, 0, sizeof(mon_topic));
    	snprintf(mon_topic, 100, "smarttrak/abijit/monitor/%s", nodeID);
    	memset(setting_topic, 0, sizeof(setting_topic));
		snprintf(setting_topic, 100, "smarttrak/abijit/setting/%s", nodeID);
		memset(sub_topic, 0, sizeof(setting_topic));
		snprintf(sub_topic, 100, "smarttrak/abijit/setting/%s", nodeID);
    }

    if (buildDate != NULL)
    {
        ret = 1;
    } else if (res != NULL)
    {
        ret = 2;
    }

    // Clean up cJSON structure
    cJSON_Delete(json);

    return ret;
}

esp_err_t construct_mqtt_packet(char *str, uint8_t len) {
    cJSON *root = cJSON_Parse(str);
    cJSON *timeSettingArray = cJSON_GetObjectItem(root, "time_setting");
    if (root == NULL) {
        fprintf(stderr, "Error parsing JSON.\n");
        return ESP_FAIL;
    }

    printf("Checking cJSON Parser....\r\n");
    for (int i = 0; i < 3; i++)
    {
        printf("Time Setting[%d]: %d\r\n", i, cJSON_GetArrayItem(timeSettingArray, i)->valueint);
    }

    return ESP_OK;
}

int transmit_stm32(uint8_t *data, uint16_t size) {
    return uart_write_bytes(UART_NUM_1, data, size);
}

