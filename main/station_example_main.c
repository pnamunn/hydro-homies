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
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

/////////////// Menuconfigs (ESP-IDF Kconfigs) ////////////////
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  4
#define PARTITION_TABLE_TYPE CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE
// TODO add rest of Kconfigs for BT & erase Kconfigs for Wifi that we don't need
#define LOG_DEFAULT_LEVEL CONFIG_LOG_DEFAULT_LEVEL_INFO

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

/////////////// end Menuconfigs (ESP-IDF Kconfigs) ////////////////


////////////////// GLOBALS ////////////////////

// Function Declarations //
// TODO add rest of function declarations
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* params);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_interface, esp_ble_gatts_cb_param_t* params);
static void gatts_profile_0_event_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_interface, esp_ble_gatts_cb_param_t* params);
static void gatts_profile_1_event_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_interface, esp_ble_gatts_cb_param_t* params);

// Logging //
static const char* STA = "STA";
static const char* NTP = "NTP";
static const char* GPIO = "GPIO";
static const char* UART = "UART";
static const char* BLE = "BLE";
// static const char* BLE_WARN = "BLE";

// Wifi and NTP //
static EventGroupHandle_t s_wifi_event_group;
static int retry_count = 0;
struct tm* localTime;

// UART //
const uart_port_t uart_num = UART_NUM_0;
#define RX_BUFFER_SIZE 1024

// GPIO //
typedef struct waterTaskParams_t {
    uint8_t pin;
    uint8_t durationSec;
    struct tm scheduledTime;
} waterTaskParams_t;

waterTaskParams_t pin3Parameters = {.pin = 3, .durationSec = 5,
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


// BLE GAP //

// Advertisement service UUIDs (as big endian)
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 
    0xEE, 0x00,
     0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 
    0xFF, 0x00, 0x00, 0x00,
};

// Advertising Data Payload configs
static esp_ble_adv_data_t advertising_data_payload_configs = {
    .set_scan_rsp = false,  // is this not a Scan Response Payload
    .include_name = true,   // advertise this ESP's name
    .include_txpower = false,
    .min_interval = 0x0006, // advertise the preferred connection min interval = val * 1.25 ms = 7.5 ms
    .max_interval = 0x0010, // advertise the preferred connection max interval = val * 1.25 ms = 20 ms
    .appearance = ESP_BLE_APPEARANCE_GENERIC_HEART_RATE,    // icon apperance on the central device
    .p_manufacturer_data =  NULL,   // don't advertise manufacturer data
    .manufacturer_len = 0,
    .p_service_data = NULL, // don't advertise any service data
    .service_data_len = 0,
    .p_service_uuid = adv_service_uuid128,  // advertise this array of service uuids
    .service_uuid_len = sizeof(adv_service_uuid128),
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)    // bit field to set discovery settings
};

// Set advertising parameters
static uint8_t advertising_done_flag = 0;
static esp_ble_adv_params_t advertising_params = {
    .adv_int_min = 0x0020,    // min interval = val * 0.625 ms = 20 ms
    .adv_int_max = 0x0800,    // max interval = val * 0.625 ms = 1.28 sec
    .adv_type = ADV_TYPE_IND,   // indication
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,    // use all channels
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // allow scan response & connection response from any central peripheral (client)
};


// BLE GATT //

// Struct to hold all info of a GATT Server Application Profile
typedef struct gatts_profile_t {
    esp_gatts_cb_t gatts_callback;
    uint16_t gatts_interface;   // holds server-client interface type
    uint16_t application_id;
    uint16_t connection_id;
    esp_gatt_srvc_id_t service_id;  // differentiates multiple services with the same UUID
    esp_bt_uuid_t characteristic_uuid;
    esp_gatt_char_prop_t characteristic_properties; // bit field to set flags
    esp_bt_uuid_t characteristic_descriptor_uuid;
    esp_gatt_perm_t attribute_permissions;
    uint16_t service_handle;
    uint16_t characteristic_handle;
    uint16_t characteristic_descriptor_handle;   
} gatts_profile_t;

// The number of handles that are going to be needed to per service
// Passed as a parameter to esp_ble_gatts_create_service()
// Going to be 4 handles in a service:
//      service handle, characteristic handle,
//      characteristic data value handle, characteristic descriptor handle
static const uint8_t NUM_HANDLES_PER_SERVICE = 4;

// Create array of structs that holds the 2 GATT Server Profiles
static gatts_profile_t gatts_profiles[2] = {
    [0] = {
        .gatts_callback = gatts_profile_0_event_callback,
        .gatts_interface = ESP_GATT_IF_NONE, // profile not linked to any client type yet
        },
    [1] = {
        .gatts_callback = gatts_profile_1_event_callback,
        .gatts_interface = ESP_GATT_IF_NONE,
        },
};

// static const esp_attr_control_t READ_AND_WRITE_AUTO_RESPONSE_SETTING = { 
//     .auto_rsp = ESP_GATT_AUTO_RSP
// };

// Initial values being given the Characteristic & Characteristic Descriptor
static uint8_t characteristic_data_value[] = {0x11,0x22,0x33};  // dummy data
// static uint8_t characteristic_description_data_value[] ={0x44,0x55,0x66};  // dummy data

static esp_attr_value_t characteristic_data_value_handle = {
    .attr_max_len = 0x40,
    .attr_len     = sizeof(characteristic_data_value),
    .attr_value   = characteristic_data_value
};

// static esp_attr_value_t characteristic_description_data_value_handle = {
//     .attr_max_len = 0x40,
//     .attr_len     = sizeof(characteristic_description_data_value),
//     .attr_value   = characteristic_description_data_value
// };

static uint8_t read_response_value[] = {0xDE,0xED,0xBE,0xEF}; // dummy data


////////////////// end GLOBALS ////////////////////




static void wifi_connection_events_handler(void* arg,
                                           esp_event_base_t event_base,
                                           int32_t event_id,
                                           void* event_data) {

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


//////////////////// BLE ///////////////////////

void init_BLE() {
    esp_log_level_set(BLE, ESP_LOG_INFO);

    // Free up the BSS (Bluetooth Simple Setup) & Classic BT data memory from the ESP's Bluetooth Controller, as you only plan to use BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Sse default configs for the ESP's Bluetooth Controller
    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_config));

    // Set Bluetooth Controller to BLE mode
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // use Bluedroid, a BT stack for Android
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Tell ESP to use this callback to handle all GAP events
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    // Tell ESP to use this callback to handle all GATT Server events
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // Creates 2 Application Profiles, so this GATT Server (the ESP) is able to handle 2 different types of clients
    // Profiles' Application UUIDs are 0 and 1
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(1));

    // set max transmission unit (packet) size in bytes
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));

    ESP_LOGI(BLE, "BLE initialized");
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t* params) {
    switch(event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // Triggered by: esp_ble_gatts_app_register(gap_event_handler) in init_BLE()
            advertising_done_flag &= (~advertising_done_flag);
            if(advertising_done_flag == 0) {
                ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&advertising_params));
            }
        break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // Triggered by: esp_ble_gap_start_advertising() in ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT
            // Just inform user that GAP advertising was successfully started
            ESP_LOGI(BLE, "GAP advertising started.");
        break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // Triggered by: esp_ble_gap_update_conn_params() in ESP_GATTS_CONNECT_EVT
            // just print the client-server connection's info
            ESP_LOGI(BLE, "Client-Server BT connection established");
        break;

        default:
        break;
    }
}

/* Finds the appropriate GATT Server Profile & calls its event handling callback.
If, however, no Profile exists yet for the detected server-client interface type
(and we have room to create one more Profile), creates a Profile to support it. */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_interface,
                                esp_ble_gatts_cb_param_t* params) {

    // if given a GATTS register event, get its current server-client interface type
    if(event == ESP_GATTS_REG_EVT) {
        if(params->reg.status == ESP_GATT_OK) {
            gatts_profiles[params->reg.app_id].gatts_interface = gatts_interface;
        }
        else {
            ESP_LOGI(BLE, "GATTS registering failed for Profile %d", params->reg.app_id);
            return;
        }
    }

    // TODO why do while(0) here?
    do {
        // Go thru all gatts_profiles
        for(int i=0; i < 2; i++) {
            // Find the server-client interface type we want, if it exists already
            // or the first profile not assigned a server-client interface yet
            if( (gatts_interface == ESP_GATT_IF_NONE)
             || (gatts_interface == gatts_profiles[i].gatts_interface) ) {
                
                // If the server-client interface we wanted exists,
                // call its callback with the passed args
                if(gatts_profiles[i].gatts_callback) {
                    gatts_profiles[i].gatts_callback(event, gatts_interface, params);
                }
            }
        }
    }while(0);
}

/* Callback called when gatts_event_handler() detects a Profile 0 event.
Sets GAP advertisement data, */ 
static void gatts_profile_0_event_callback(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_interface,
                                          esp_ble_gatts_cb_param_t* params) {
    switch(event) {
        case ESP_GATTS_REG_EVT:
        // Triggered by: esp_ble_gatts_app_register(gatts_event_handler) in init_BLE() (and directed to this call back thru gatts_event_handler())
            // Create a service
            gatts_profiles[0].service_id.is_primary = true; // this is a primary service, not a secondary
            gatts_profiles[0].service_id.id.inst_id = 0x00; // instance ID
            gatts_profiles[0].service_id.id.uuid.len = ESP_UUID_LEN_16; // UUID length
            gatts_profiles[0].service_id.id.uuid.uuid.uuid16 = 0x00FF;   // UUID

            // set device name, as it will show up on central devices
            ESP_ERROR_CHECK(esp_ble_gap_set_device_name("ESP32 Hydro Homies BLE"));

            // Set GAP Advertising Data Payload configs
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&advertising_data_payload_configs));
            advertising_done_flag |= (1 << 0);

            ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_interface, &gatts_profiles[0].service_id, NUM_HANDLES_PER_SERVICE));
        break;

        case ESP_GATTS_CREATE_EVT:
        // Triggered by: esp_ble_gatts_create_service() in ESP_GATTS_REG_EVT
            ESP_LOGI(BLE, "Service ID %d created",  gatts_profiles[0].service_id.id.uuid.uuid.uuid16);
            gatts_profiles[0].service_handle = params->create.service_handle;
            gatts_profiles[0].characteristic_uuid.len = ESP_UUID_LEN_16;
            gatts_profiles[0].characteristic_uuid.uuid.uuid16 = 0xFF01;
        
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(gatts_profiles[0].service_handle));

            // Enable characteristic read, write, & notify properties (client is allowed to do these)
            gatts_profiles[0].characteristic_properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            
            // Add Characteristic to the Service
            ESP_ERROR_CHECK(esp_ble_gatts_add_char(gatts_profiles[0].service_handle,
                                                   &gatts_profiles[0].characteristic_uuid,
                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // give Service permission to read & write this Characteristic
                                                   gatts_profiles[0].characteristic_properties,
                                                   &characteristic_data_value_handle, // give Characteristic this inital val
                                                   ESP_GATT_RSP_BY_APP));  // no auto reponse to a client's requests to read or write this Characteristic
        break;

        case ESP_GATTS_ADD_CHAR_EVT:
        // Triggered by: esp_ble_gatts_add_char() in ESP_GATTS_CREATE_EVT
            gatts_profiles[0].characteristic_handle = params->add_char.attr_handle;
            gatts_profiles[0].characteristic_descriptor_uuid.len = ESP_UUID_LEN_16;
            // auto-generate a handle for the Characteristic Descriptor you're about to add
            gatts_profiles[0].characteristic_descriptor_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

            uint16_t length = 0;
            const uint8_t* attribute_value_payload;
            // Grab Characteristic's length & payload, and store in length & *attribute_value_payload variables
            ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(params->add_char.attr_handle,
                                                         &length,
                                                         &attribute_value_payload));

            ESP_LOGI(BLE, "Characteristic payload = ");
            for(int i=0; i < length; i++) {
                ESP_LOGI(BLE, "%X", attribute_value_payload[i]);
            }

            // Add Characteristic Description to the Service
            ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(gatts_profiles[0].service_handle,
                                                         &gatts_profiles[0].characteristic_descriptor_uuid,
                                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // give Service permission to read & write this Characteristic Description
                                                         &characteristic_description_data_value_handle,  // give this initial value
                                                         ESP_GATT_RSP_BY_APP)); // no auto reponse to a client's requests to read or write this Characteristic Description
        break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // Triggered by: esp_ble_gatts_add_char_descr() in ESP_GATTS_ADD_CHAR_EVT
            // just log a message
            ESP_LOGI(BLE, "You added a characteristic & a characteristic description to the service.");
            ESP_LOGI(BLE, "The characteristic's handle is %X", gatts_profiles[0].characteristic_handle);
            ESP_LOGI(BLE, "And the characteristic descriptor's handle is %X", gatts_profiles[0].characteristic_descriptor_handle);
        break;

        case ESP_GATTS_START_EVT:
        // Triggered by: esp_ble_gatts_start_service() in ESP_GATTS_CREATE_EVT
            ESP_LOGI(BLE, "Service started, has handle %d", params->start.service_handle);
        break;

        case ESP_GATTS_CONNECT_EVT:
        // Triggered by: a client connecting to the GATT Server
            esp_ble_conn_update_params_t connection_parameters = {0};
            connection_parameters.latency = 0;
            connection_parameters.min_int = 0x10;    // val * 1.25 ms = 20 ms
            connection_parameters.max_int = 0x30;   // val * 1.25 ms = 40 ms
            connection_parameters.timeout = 400;    // val * 10 ms = 4,000 ms
            // Copy params->connect.remote_bda to connection_parameters.bda
            memcpy(connection_parameters.bda,   // bda = BT Device Address
                   params->connect.remote_bda,
                   sizeof(esp_bd_addr_t));

            gatts_profiles[0].connection_id = params->connect.conn_id;

            ESP_LOGI(BLE, "Connection to a central device made. BDA (BT Device Address) is "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(params->connect.remote_bda));

            // Set connection parameters for the new server-client connection
            ESP_ERROR_CHECK(esp_ble_gap_update_conn_params(&connection_parameters));
        break;

        case ESP_GATTS_READ_EVT:
        // Triggered by: client sent a Read Request (is trying to read an attribute (a Service, a Charcteristic, or a Characteristic Descriptor))
            // Retrieve attribute that client wants to read
            esp_gatt_rsp_t retrieved_attribute = {0};
            retrieved_attribute.attr_value.handle = params->read.handle;

            // Array of pointers to attribute values
            // TODO increase array size to hold more attributes
            const uint8_t* attribute_value_ptrs_array[1] = {retrieved_attribute.attr_value.value};

            // Pointer-to-pointer (pointer to a pointer element in the attribute_value_pointers_array)
            // TODO have a table (struct) to associate an attribute handle with its value pointer (so we can do [handle] below)
            const uint8_t** attribute_value_ptr_ptr = &attribute_value_ptrs_array[0];

            // Get attribute at the given handle, then store its length in retrieved_attribute struct
            // & store its value in attribute_value_ptr_ptr
            esp_gatt_status_t read_return_status = 
                    esp_ble_gatts_get_attr_value(params->read.handle,
                                                 &retrieved_attribute.attr_value.len,
                                                 attribute_value_ptr_ptr);
            
            // Copy over value stored at attribute_value_ptr_ptr to retrieved_atrribute struct
            memcpy(&retrieved_attribute.attr_value.value, *attribute_value_ptr_ptr, retrieved_attribute.attr_value.len);


            // ESP_LOGI(BLE, "Retrieved value %#X (or %#X) from handle %#X", **attribute_value_ptr_ptr, *retrieved_attribute.attr_value.value, retrieved_attribute.attr_value.handle);
            ESP_LOGI(BLE, "Retrieved value");
            // ESP_LOG_BUFFER_HEX(BLE, *attribute_value_ptr_ptr, retrieved_attribute.attr_value.len);
            // ESP_LOGI(BLE, "or");
            ESP_LOG_BUFFER_HEX(BLE, retrieved_attribute.attr_value.value, retrieved_attribute.attr_value.len);
            ESP_LOGI(BLE, "from handle");
            ESP_LOG_BUFFER_HEX(BLE, &retrieved_attribute.attr_value.handle, sizeof(retrieved_attribute.attr_value.handle));
            
            // Respond to client's Read Request by sending back that attribute's data (needed bc auto-response turned off for our attributes)
            ESP_ERROR_CHECK(esp_ble_gatts_send_response(gatts_interface,
                                                        params->read.conn_id,
                                                        params->read.trans_id,
                                                        read_return_status,
                                                        &retrieved_attribute));
            
            // TODO why notify/indicate switch happens after a write event and not a read event ????
        break;

        case ESP_GATTS_RESPONSE_EVT:
        // Triggered by: esp_ble_gatts_send_response() in ESP_GATTS_READ_EVT and ESP_GATTS_WRITE_EVT
            ESP_LOGI(BLE, "ESP just sent a Response msg in reaction to a client Request");
        break;

        case ESP_GATTS_SET_ATTR_VAL_EVT:
        // Triggered by: esp_ble_gatts_set_attr_value() in
            
        break;

        case ESP_GATTS_WRITE_EVT:
        // Triggered by: client sent a type of Request related to writing

            // To indicate what characteristic the client wants to write, it sends
            // the characteristic descriptor's handle as write.handle
            // & the characteristic's value & descriptor as write.value
            
            // ESP_LOGI(BLE, "Client wants to write Characteristic %d as the value = ");
            // ESP_LOG_BUFFER_HEX(BLE, params->write.value, params->write.len);

            // Client sent anything but a Write Prepare Request (sent a Write Request or Write Command)
            if(params->write.is_prep == 0) {

                if( (params->write.handle == gatts_profiles[0].characteristic_descriptor_handle)
                && (params->write.len == 2) ) {
                // Confirm there's a characteristic descriptor with that handle on the ESP
                // and that the client sent a write.value of 16 bits

                    // Grab the characteristic descriptor that the client sent
                    uint8_t characteristic_properties_value = params->write.value[0];
                    ESP_LOGI(BLE, "Write event's characteristic_properties_value is %d", characteristic_properties_value);

                    // See if we need to send a Notification or an Indication
                    switch(characteristic_properties_value) {   // is big endian
                        case 0x01:  // Notify enabled
                            if(gatts_profiles[0].characteristic_properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                            // Confirm that the characteristic has Notify enabled, like the client claimed
                                uint8_t notify_data[15];
                                // dummy data
                                for(int i=0; i < sizeof(notify_data); i++) { notify_data[i] = i % 0xFF; }
                                // send data once it's available
                                ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(gatts_interface,
                                                                            params->write.conn_id,
                                                                            gatts_profiles[0].characteristic_handle,
                                                                            sizeof(notify_data),
                                                                            notify_data,
                                                                            false));
                                ESP_LOGI(BLE, "Notification with data sent");
                            }
                        break;

                        case 0x02:  // Indicate enabled
                            if(gatts_profiles[0].characteristic_properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
                            // Confirm that the characteristic has Indicate enabled, like the client claimed
                                uint8_t indicate_data[15];
                                // dummy data
                                for(int i=0; i < sizeof(indicate_data); i++) { indicate_data[i] = i % 0xFF; }
                                // send data once it's available & require an acknowledgement back from the central device for receiving this data
                                ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(gatts_interface,
                                                                            params->write.conn_id,
                                                                            gatts_profiles[0].characteristic_handle,
                                                                            sizeof(indicate_data),
                                                                            indicate_data,
                                                                            true));
                            }
                        break;

                        case 0x00:
                            ESP_LOGI(BLE, "Characteristic's notify & indicate properties are disabled. Sending no response to client.");

                        default:
                            ESP_LOGW(BLE, "Default reached in ESP_GATTS_WRITE_EVT's switch case.  Property given (%X) not supported.", characteristic_properties_value);
                        break;
                    }
                }
            }

            else {
                ESP_LOGW(BLE, "Client sent a Write Prepare.  Not supported by ESP_GATTS_WRITE_EVT right now");
            }


            // example_write_event_env():

            // Client sent a Write Request
            if(params->write.need_rsp && !params->write.is_prep) {

                // Overwrite the attribute
                ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(params->write.handle,
                                                             params->write.len,
                                                             params->write.value));
                // Send response with no data back (response only needed here bc auto-response disabled)
                ESP_ERROR_CHECK(esp_ble_gatts_send_response(gatts_interface,
                                                            params->write.conn_id,
                                                            params->write.trans_id,
                                                            ESP_GATT_OK,
                                                            NULL));
                ESP_LOGI(BLE, "ESP fulfilled client's Write Request to write to handle %#X the value", params->write.handle);
                ESP_LOG_BUFFER_HEX(BLE, params->write.value, params->write.len);
            }

            // Client sent a Write Prepare, so need to prepare data & send a response
            else if(params->write.need_rsp && params->write.is_prep) {
                ESP_LOGW(BLE, "Client sent a Write Prepare, which is not supported by ESP_GATTS_WRITE_EVT right now");
                // TODO if client wants to do a long write (write more than the MTU), need to write code here
                // to prepare for it by creating a buffer, check some things, 
                // write data to buffer, write buffer to characteristic, & then send a response
            }

            // Client sent a Write Command, so no response needed
            else {  // !params->write.need_rsp && params->write.is_prep is anything
                // Overwrite the attribute
                ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(params->write.handle,
                                                             params->write.len,
                                                             params->write.value));
                ESP_LOGW(BLE, "ESP fulfilled client's Write Command to write to handle %#X the value", params->write.handle);
                ESP_LOG_BUFFER_HEX(BLE, params->write.value, params->write.len);
                // TODO add code here to change that attribute's value to be params->write.value
                ESP_LOGW(BLE, "ESP fulfilled Write Command without a response");
            }

        break;

        case ESP_GATTS_EXEC_WRITE_EVT:
        // Triggered by: client sent an Execute Write Request (which they should send after we respond to their Prepare Write Request)
        // Either confirm or cancel the long write procedure done in ESP_GATTS_WRITE_EVT 
        break;

        case ESP_GATTS_CONF_EVT:
        // Triggered by: client sent an acknowledgement that it received an Indication msg (should happen after esp_ble_gatts_send_indicate(..., ..., ..., ..., ..., true))
            if(params->conf.status == ESP_GATT_OK) {
                ESP_LOGI(BLE, "Client sent an acknowledgment that it received attribute handle %d", params->conf.handle);
                ESP_LOGI(BLE, "Acknowledgement value is ");
                ESP_LOG_BUFFER_HEX(BLE, params->conf.value, params->conf.len);
            }
            else {
                ESP_LOGI(BLE, "Client tried to send an acknowledgement, but the acknowledgement failed");
            }
        break;

        case ESP_GATTS_MTU_EVT:
        // Triggered by: client sends an MTU Config Request (happens during the connection establishment process if the client wants to make a change to the MTU size)
            // TODO set the MTU to the max possible val that both devices can support

        break;

        case ESP_GATTS_DISCONNECT_EVT:
        // Triggered by: client disconnecting
            ESP_LOGI(BLE, "Disconnected from "ESP_BD_ADDR_STR" for %02X reason",
                            ESP_BD_ADDR_HEX(params->disconnect.remote_bda),
                            params->disconnect.reason);
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&advertising_params));
        break;

        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
        case ESP_GATTS_DELETE_EVT:
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        case ESP_GATTS_SET_ATTR_VAL_EVT:
        case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
        default:
        break;
    }

}

static void gatts_profile_1_event_callback(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_interface,
                                          esp_ble_gatts_cb_param_t* params) {

}

//////////////////// end BLE ///////////////////////


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
            // BUG flaws:   backspace not supported (will move on to next state)
            //              if double digits not entered very quickly, will only get first digit
            //              no error checking for entering letters instead of numbers
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

// Task to
void vBLETask() {

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
    // Non-Volatile Storage needed to hold Wifi SSID, password, and GATT 
    esp_err_t nvs_return_handle = nvs_flash_init();
    if (nvs_return_handle == ESP_ERR_NVS_NO_FREE_PAGES || nvs_return_handle == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      nvs_return_handle = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_return_handle);


    init_wifi_STA();
    init_SNTP();

    init_BLE();

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

     // BLE Task
    taskCreateStatus = xTaskCreate(vRxTask, "BLE task", 1024 * 4,
                                   NULL, 2, NULL);
    if(taskCreateStatus == pdPASS) {
        ESP_LOGI(GPIO, "vBLETask() task creation successful");
    }

    // esp idf's task.h API automatically runs vTaskStartScheduler() at end of app_main()
}

