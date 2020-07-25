#include <Config.h>
#include "Wifi.h"




#define CHECK(x)                                                               \
    do {                                                                       \
        esp_err_t __err_rc = (x);                                              \
        if (__err_rc != ESP_OK) {                                              \
            WARN("%s = %d ", #x, __err_rc);        \
        }                                                                      \
    } while (0);




Wifi::Wifi(Thread& thr) : Actor(thr),password(S(WIFI_PASS)),prefix(S(WIFI_SSID))
{
    rssi = 0;
}

Wifi::~Wifi()
{
}

void Wifi::init()
{
    wifiInit();
    union {
        uint8_t macBytes[6];
        uint64_t macInt;
    };
    macInt=0L;
    if ( esp_read_mac(macBytes,ESP_MAC_WIFI_STA) != ESP_OK) WARN(" esp_base_mac_addr_get() failed.");;
    std::string macs;
    string_format(macs,"%02X:%02X:%02X:%02X:%02X:%02X",macBytes[5],macBytes[4],macBytes[3],macBytes[2],macBytes[1],macBytes[0]);
    macAddress = macs;
    mac = macInt;
    connected = false;
}

void Wifi::ip_event_handler(void* event_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    Wifi& wifi = *(Wifi*) event_arg;
    switch (event_id) {

    case IP_EVENT_STA_GOT_IP: {
        INFO("IP_EVENT_STA_GOT_IP");
        ip_event_got_ip_t* got_ip = (ip_event_got_ip_t*)event_data;
        char my_ip_address[20];
        ip4addr_ntoa_r((ip4_addr_t*)&got_ip->ip_info.ip, my_ip_address, 20);
        wifi.ipAddress = my_ip_address;
        wifi.connected = true;
        break;
    }

    case IP_EVENT_STA_LOST_IP: {
        INFO("IP_STA_LOST_IP");
        wifi.connected=false;
        break;
    }
    default:
        WARN("unknown IP event %d ", event_id);
        break;
    }
}



void Wifi::wifi_event_handler(void* event_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    Wifi& wifi = *(Wifi*) event_arg;
    switch (event_id) {

    case WIFI_EVENT_SCAN_DONE: {
        INFO("WIFI_EVENT_SCAN_DONE");
        if ( wifi.scanDoneHandler() ) {
            wifi.connectToAP(wifi.ssid().c_str());
        } else {
            wifi.startScan();
        }
        break;
    }
    case WIFI_EVENT_STA_STOP: {
        INFO("WIFI_EVENT_STA_STOP");
//				esp_wifi_start();
//				wifi.wifiInit();
        break;
    }
    case WIFI_EVENT_STA_START: {
        INFO("WIFI_EVENT_STA_START");
        wifi.startScan();
        break;
    }

    case WIFI_EVENT_STA_CONNECTED: {
        INFO("WIFI_EVENT_STA_CONNECTED");
        break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
        INFO("WIFI_EVENT_STA_DISCONNECTED");
        esp_wifi_connect();
        break;
    }

    default:
        WARN("unknown WiFi event %d ", event_id);
        break;
    }

}

void Wifi::connectToAP(const char* ssid)
{
    INFO(" connecting to SSID : %s", ssid);

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config)); // needed !!
    strncpy((char*) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid)
            - 1);
    strncpy((char*) wifi_config.sta.password, S(WIFI_PASS), sizeof(wifi_config.sta
            .password) - 1);
    CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    esp_wifi_connect();
}

bool Wifi::scanDoneHandler()
{
    uint16_t sta_number;
    esp_wifi_scan_get_ap_num(&sta_number);
    INFO(" found %d AP's , size : %d ", sta_number,sizeof(wifi_ap_record_t));
    if (sta_number == 0) {
        WARN(" no AP found , restarting scan.");
        return false;
    }
    wifi_ap_record_t apRecords[sta_number];
    esp_wifi_scan_get_ap_records(&sta_number, apRecords);
    int strongestAP = -1;
    rssi = -200;
    for (uint32_t i = 0; i < sta_number; i++) {
        INFO(" %s : %d ", apRecords[i].ssid, apRecords[i].rssi);
        std::basic_string<char> ssid = (const char*) apRecords[i].ssid;
        if ((apRecords[i].rssi > rssi()) && (ssid.find(prefix()) == 0)) {
            strongestAP = i;
            rssi = apRecords[i].rssi;
        }
    }
    if (strongestAP == -1) {
        WARN(" no AP found matching pattern '%s', restarting scan.", prefix().c_str());
        return false;
    }
    this->ssid = (const char*) apRecords[strongestAP].ssid;
    INFO(" found strongest AP : '%s'.",ssid().c_str());
    return true;
}

void Wifi::startScan()
{
    INFO(" starting WiFi scan."); //https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/wifi.html#scan-configuration
    wifi_scan_config_t scanConfig = {
        NULL, NULL, 0, false, WIFI_SCAN_TYPE_ACTIVE, {{ 0, 0 },0} };
    CHECK(esp_wifi_scan_start(&scanConfig, false));

}

void Wifi::wifiInit()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        WARN(" erasing NVS flash ");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    CHECK(esp_netif_init());
    CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    _wifiConfig = WIFI_INIT_CONFIG_DEFAULT();
    CHECK(esp_wifi_init(&_wifiConfig));

    CHECK( esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_handler,this));
    CHECK( esp_event_handler_register(IP_EVENT,ESP_EVENT_ANY_ID,ip_event_handler,this));


//	CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//	CHECK(esp_wifi_set_promiscuous(false));
//	CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B));
    CHECK(esp_wifi_start());
}
