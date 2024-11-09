#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <driver/gpio.h>
#include "driver/uart.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// Wifi menuconfigs
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  4

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define WIFI_CONNECTED_BIT BIT0     // connected to AP & has an IP
#define WIFI_FAIL_BIT      BIT1     // failed to connect
static EventGroupHandle_t s_wifi_event_group;
static int retry_count = 0;


// Logging
static const char* STA = "STA";
static const char* NTP = "NTP";
static const char* GPIO = "GPIO";
static const char* UART = "UART";

// Globals
struct tm* localTime;
const uart_port_t uart_num = UART_NUM_0;
#define RX_BUFFER_SIZE 1024

typedef struct waterTaskParams_t {
    uint8_t pin;
    uint8_t durationSec;
    struct tm scheduledTime;
} waterTaskParams_t;

struct waterTaskParams_t pin3Parameters = {.pin = 3, .durationSec = 5,
                                            .scheduledTime.tm_hour = 22, .scheduledTime.tm_min = 30, .scheduledTime.tm_sec = 10};

enum AskState {
    DEFAULT,

    // for 's' menu option
    HOUR,
    MIN,
    CONFIRM_TIME,

    // for 'd' menu option
    DURATION,
    CONFIRM_DURATION
};

static void wifi_connection_events_handler
            (void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    // if wifi has been started in station mode
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    // if wifi fails to connect or is called to disconnect
        if (retry_count < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(STA, "Retrying to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(STA,"Connect attempt to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    // if wifi successfully got IPv4 from DHCP (ready to begin tasks)
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(STA, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void init_wifi_STA(void) {
    s_wifi_event_group = xEventGroupCreate();

    // use NETIF lib for application layer abstraction & thread safety
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // init wifi
    wifi_init_config_t wifi_configs = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_configs));

    // handler events
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    // handler check if got wifi ID data
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_connection_events_handler,
                                                        NULL,
                                                        &instance_any_id));
    // handler check if got IP data
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_connection_events_handler,
                                                        NULL,
                                                        &instance_got_ip));


    // set sta mode, configs, & start wifi
    wifi_config_t sta_configs = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_configs));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Block until one of these events occurs
    EventBits_t wifiEventBits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // react to the event that occurred
    if (wifiEventBits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(STA, "Successfully connected to AP SSID: %s", EXAMPLE_ESP_WIFI_SSID);
    } else if (wifiEventBits & WIFI_FAIL_BIT) {
        ESP_LOGI(STA, "Failed to connect to SSID: %s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(STA, "Unexpected error occured when attempting to connect to SSDI: %s", EXAMPLE_ESP_WIFI_SSID);
    }
}


void init_SNTP() {
    // set PST timezone
    char timezone[] = "PST8PDT";
    setenv("TZ", timezone, 1);     // change timezone env variable
    tzset();    // set runtime timezone to TZ env variable (auto checks daylight savings)
    ESP_LOGI(NTP, "Timezone set to %s", timezone);

    // choose server, init SNTP, & start service
    char ntp_server_address[] = "pool.ntp.org";
    esp_sntp_config_t sntp_configs = ESP_NETIF_SNTP_DEFAULT_CONFIG(ntp_server_address);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_configs));    // also auto-starts trying to achieve SNTP service
    ESP_ERROR_CHECK(esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000))); // give 15 secs to attempt to connect to service
    ESP_LOGI(NTP, "Synced to SNTP service %s successfully!", ntp_server_address);
}


// Init GPIO pin to provide a pump signal.
void initPump(uint32_t pinNum) {
    gpio_reset_pin(pinNum);   // enables pullup
    gpio_set_direction(pinNum, GPIO_MODE_OUTPUT);
    gpio_set_level(pinNum, 0);
    ESP_LOGI(GPIO, "Finish init GPIO pin %"PRIu32" to provide pump signal", pinNum);
}

// Shows if scheduled time & local time match, to min accuracy.
bool isTimeMatch(struct tm scheduledTime) {
    if ((scheduledTime.tm_hour - localTime->tm_hour == 0) &&
        (scheduledTime.tm_min - localTime->tm_min == 0)) {
        return true;
    }
    else {
        return false;
    }
}


void initUART() {
    // set configs
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // choose which UART pin & helper pins to use
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // install driver
    // with Rx buffer, no Tx buffer (Tx is blocking)
    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUFFER_SIZE*2, 0,
                                        0, NULL, 0));
    ESP_LOGI(UART, "UART initialized");
}


void vRxTask() {
    uint8_t RxdMsgLen = 0;
    char RxdData[RX_BUFFER_SIZE];
    enum AskState state = DEFAULT;
    // Tell menu options
    ESP_LOGI(UART, "Enter 's' to change a scheduled time. \n \
                    Enter 'd' to change a water duration.");

    while(1) {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num,    // store length of data in RxdMsgLen
                                                   (size_t*)&RxdMsgLen));
        if(RxdMsgLen) {
            uart_read_bytes(uart_num, RxdData, 20, pdMS_TO_TICKS(1000));
            RxdData[RxdMsgLen] = '\0'; // add null character at end of data            
            // ESP_LOGI(UART, "you entered: %s", RxdData);

            // Handle user's menu option input
            if(*RxdData == 's') {
                ESP_LOGI(UART, "GIMME A PIN: ");
                state = HOUR;
                continue;   // skip to next while loop iteration
            }
            else if(*RxdData == 'd') {
                ESP_LOGI(UART, "GIMME A PIN: ");
                state = DURATION;
                continue;   // skip to next while loop iteration
            }

            // Take user inputs
            switch(state) {
                ////// 's'
                case HOUR:
                    // set pin
                    pin3Parameters.pin = (uint8_t)atoi(RxdData);
                    // prompt hr
                    ESP_LOGI(UART, "GIMME A HR: ");
                    state = MIN;
                    break;
                case MIN:
                    // set hr
                    pin3Parameters.scheduledTime.tm_hour = atoi(RxdData);
                    // prompt min
                    ESP_LOGI(UART, "GIMME A MIN: ");
                    state = CONFIRM_TIME;
                    break;
                case CONFIRM_TIME:
                    // set min
                    pin3Parameters.scheduledTime.tm_min = atoi(RxdData);
                    // Convert tm info into H:M time format & print confirmation
                    char timeStr[6];
                    strftime(timeStr, sizeof(timeStr), "%H:%M", &pin3Parameters.scheduledTime);
                    ESP_LOGI(UART, "You just changed the scheduled time for pin %hhu to %s",
                                    pin3Parameters.pin, timeStr);
                    state = 0;
                    break;
                //////
                ////// 'd'
                case DURATION:
                    // set pin
                    pin3Parameters.pin = (uint8_t)atoi(RxdData);
                    // prompt duration
                    ESP_LOGI(UART, "GIMME A DURATION (s): ");
                    state = CONFIRM_DURATION;
                    break;
                case CONFIRM_DURATION:
                    // set duration
                    pin3Parameters.durationSec = (uint8_t)atoi(RxdData);
                    // confirm choices
                    ESP_LOGI(UART, "You just changed the duration for pin %hhu to %d seconds.",
                                pin3Parameters.pin, pin3Parameters.durationSec);
                    state = DEFAULT;
                    break;
                //////
                //////
                case DEFAULT:
                default:
                    state = DEFAULT;
                    break;
            }

            // reset RxdMsgLen, clear RxdData, flush ESP's UART Rx buffer
            RxdMsgLen = 0;
            RxdData[0] = '\0';
            uart_flush(uart_num);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// Task to turn on pin for its duration.
void vWaterTask(void* params) {
    waterTaskParams_t* p = (waterTaskParams_t*) params;
    TickType_t xPrevWakeTime = xTaskGetTickCount();
    BaseType_t xDelayUntilSuccess;

    while(1) {

        if (isTimeMatch(p->scheduledTime)) {  // if no delta
            gpio_set_level(p->pin, 1);
            ESP_LOGI(GPIO, "IT'S WATER TIME.");
            ESP_LOGI(GPIO, "Current time: %s", asctime(localTime));
            ESP_LOGI(GPIO, "Scheduled time: %s", asctime(&p->scheduledTime));

            vTaskDelay(pdMS_TO_TICKS(p->durationSec * 1000));   // pump stays on for this long
            gpio_set_level(p->pin, 0);
            ESP_LOGI(GPIO, "Water time done.");
        }

        xDelayUntilSuccess = xTaskDelayUntil(&xPrevWakeTime,  pdMS_TO_TICKS(10 * 1000));    // every 10 sec
        if(xDelayUntilSuccess == pdFALSE) {
                ESP_LOGE(NTP, "Error:  vWaterTask()'s xTaskDelayUntil was unsuccessful bc ???");
        }
    }
}


// Task to periodically print time.
void vPrintTimeTask(void* periodSec) {
    TickType_t* xPeriodMS = (TickType_t*) periodSec;
    *xPeriodMS *= 1000;     // convert from sec to ms

    time_t secondsSinceEpoch;
    // struct tm* localTime;
    TickType_t xPrevWakeTime = xTaskGetTickCount();
    BaseType_t xDelayUntilSuccess;

    while(1) {
        secondsSinceEpoch = time(NULL);   // get current system time
        localTime = localtime(&secondsSinceEpoch);
        ESP_LOGI(NTP, "Current time is:  %s", asctime(localTime));

        // block task for desired period
        xDelayUntilSuccess = xTaskDelayUntil(&xPrevWakeTime,  pdMS_TO_TICKS(*xPeriodMS));
        if(xDelayUntilSuccess == pdFALSE) {
            ESP_LOGE(NTP, "Error:  vPrintTimeTask()'s xTaskDelayUntil was unsuccessful bc ???");
        }
    }
}




void app_main(void) 
{
    // init NVS partition
    esp_err_t nvs_return_handle = nvs_flash_init();
    if (nvs_return_handle == ESP_ERR_NVS_NO_FREE_PAGES || nvs_return_handle == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_return_handle = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_return_handle);


    init_wifi_STA();
    init_SNTP();
    
    // esp_err_t status_handle = esp_wifi_get_mode(WIFI_MODE_STA);
    // char* status_string[] = esp_err_to_name(status_handle);

    // ESP_LOGI(STA, "Wifi should be on -> Status is: %s", status_string);
    // ESP_ERROR_CHECK(esp_wifi_stop());

    initPump(5);
    initUART();

    // Print Time Task
    TickType_t* periodSecParam = malloc(sizeof(TickType_t));
    *periodSecParam = 10;
    BaseType_t taskCreateStatus = xTaskCreate(vPrintTimeTask, "Print time task", 1024 * 4,
                                              (void*) periodSecParam, 1, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vPrintTimeTask() task creation successful");
    }

    // Water GPIO Task
    taskCreateStatus = xTaskCreate(vWaterTask, "GPIO3 task", 1024 * 4,
                                   (void*) &pin3Parameters, 1, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vWaterTask() task creation for GPIO3 successful");
    }

    // UART Rx User Input Task
    taskCreateStatus = xTaskCreate(vRxTask, "UART RX task", 1024 * 4,
                                   NULL, 2, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vRxTask() task creation successful");
    }

    // esp idf's task.h API automatically runs vTaskStartScheduler() at end of app_main()
}

