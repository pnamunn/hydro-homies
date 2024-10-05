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

// Global
typedef struct waterTaskParams_t {
    uint8_t pin;
    uint32_t durationSec;
    struct tm scheduledTime;
} waterTaskParams_t;

struct waterTaskParams_t pin5WaterParams = {.pin = 5, .durationSec = 5};


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


void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    // use NETIF lib for application layer abstraction & thread safety
    ESP_ERROR_CHECK(esp_netif_init());

    // choose server & start service
    char ntp_server_address[] = "pool.ntp.org";
    esp_sntp_config_t sntp_configs = ESP_NETIF_SNTP_DEFAULT_CONFIG(ntp_server_address);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_configs));
    ESP_LOGI(NTP, "Service started from %s", ntp_server_address);

    // set PST timezone
    char timezone[] = "PST8PDT";
    setenv("TZ", timezone, 1);     // change timezone env variable
    tzset();    // set runtime timezone to TZ env variable (auto checks daylight savings)
    ESP_LOGI(NTP, "Timezone set to %s", timezone);


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

    wifi_config_t sta_configs = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    // set sta mode, configs, & start wifi
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

    ESP_LOGI(STA, "wifi_init_sta() finished.");
}

// Task to turn on pump for water duration 5 sec, every 10 seconds.
void vWaterTask(void* params) {
    waterTaskParams_t* p = (waterTaskParams_t*) params;

    while(1) {
        gpio_set_level(p->pin, 1);
        ESP_LOGI(GPIO, "pump %"PRIu8" ON", p->pin);

        vTaskDelay(pdMS_TO_TICKS(p->durationSec * 1000));   // pump stays on for this long

        gpio_set_level(p->pin, 0);
        ESP_LOGI(GPIO, "pump %"PRIu8" OFF", p->pin);

    }
}


// Task to periodically print time.
void vPrintTimeTask(void* periodSec) {
    TickType_t* xPeriodMS = (TickType_t*) periodSec;
    *xPeriodMS *= 1000;     // convert from sec to ms

    time_t secondsSinceEpoch;
    struct tm* localTime;
    TickType_t xPrevWakeTime = xTaskGetTickCount();
    // int count = 0;
    BaseType_t xDelayUntilSuccess;

    while(1) {
        secondsSinceEpoch = time(NULL);   // get current system time
        localTime = localtime(&secondsSinceEpoch);
        ESP_LOGI(NTP, "Current time is:  %s", asctime(localTime));
        // ++count;
        // ESP_LOGI(NTP, "%d", count);

        // block task for desired period
        xDelayUntilSuccess = xTaskDelayUntil(&xPrevWakeTime,  pdMS_TO_TICKS(*xPeriodMS));
        if(xDelayUntilSuccess == pdFALSE) {
            ESP_LOGE(NTP, "Error:  vPrintTimeTask()'s xTaskDelayUntil was unsuccessful bc ???");
        }

    }
}


// Init GPIO pin to provide a pump signal.
void initPump(uint32_t pinNum) {
    gpio_reset_pin(pinNum);   // enables pullup
    gpio_set_direction(pinNum, GPIO_MODE_OUTPUT);
    gpio_set_level(pinNum, 0);
    ESP_LOGI(GPIO, "Finish init GPIO pin %"PRIu32" to provide pump signal", pinNum);
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

    ESP_LOGI(STA, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    
    // esp_err_t status_handle = esp_wifi_get_mode(WIFI_MODE_STA);
    // char* status_string[] = esp_err_to_name(status_handle);

    // ESP_LOGI(STA, "Wifi should be on -> Status is: %s", status_string);
    // ESP_ERROR_CHECK(esp_wifi_stop());

    initPump(5);


    // Print Time Task
    TickType_t* periodSecParam = malloc(sizeof(TickType_t));
    *periodSecParam = 10;
    BaseType_t taskCreateStatus = xTaskCreate(vPrintTimeTask, "Print time", 1024 * 4,
                                              (void*) periodSecParam, 1, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vPrintTimeTask() task creation successful");
    }


    // Water GPIO Task
    taskCreateStatus = xTaskCreate(vWaterTask, "GPIO 5", 1024 * 4,
                                   (void*) &pin5WaterParams, 1, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vWaterTask() task creation for GPIO 5 successful");
    }


    // esp idf's task.h API automatically runs vTaskStartScheduler() at end of app_main()
}

