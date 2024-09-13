/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
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
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <driver/gpio.h>

#include "lwip/err.h"
#include "lwip/sys.h"

// Wifi menuconfigs
#define EXAMPLE_ESP_WIFI_SSID      "DALLEVIGNE-SKY"
#define EXAMPLE_ESP_WIFI_PASS      "welcome2dallevigne"
#define EXAMPLE_ESP_MAXIMUM_RETRY  4

// GPIO menuconfigs
#define LED_GPIO 5


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

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char* STA = "STA";
static const char* NTP = "NTP";
static const char* GPIO = "GPIO";
static int retry_count = 0;


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
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    // set sta mode, configs, & start wifi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_configs));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned,
    hence we can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(STA, "Successfully connected to AP SSID: %s", EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(STA, "Failed to connect to SSID: %s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(STA, "Unexpected error occured when attempting to connect to SSDI: %s", EXAMPLE_ESP_WIFI_SSID);
    }

    ESP_LOGI(STA, "wifi_init_sta() finished.");
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
    time_t seconds_since_epoch;
    struct tm* current_time;

    gpio_reset_pin(LED_GPIO);   // enables pullup
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    ESP_LOGI(GPIO, "Init GPIO pin 5 to output");


    while(1) {
        // sleep(10);
        uint16_t delay_seconds = 10;
        vTaskDelay((delay_seconds * 1000) / portTICK_PERIOD_MS);

        seconds_since_epoch = time(NULL);
        current_time = localtime(&seconds_since_epoch);
        printf("Current time is:  %s", asctime(current_time));


        // if the second's tens place is odd, turn led on; if even, turn led off
        uint32_t led_level = (current_time->tm_sec / 10) % 2;
        gpio_set_level(LED_GPIO, led_level);
        ESP_LOGI(GPIO, "Turning led to %"PRIu32"", led_level);


    }
}

