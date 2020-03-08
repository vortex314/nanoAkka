#include <Config.h>
#include "Wifi.h"




#define CHECK(x)                                                               \
	do {                                                                       \
		esp_err_t __err_rc = (x);                                              \
		if (__err_rc != ESP_OK) {                                              \
			WARN("%s = %d ", #x, __err_rc);        \
		}                                                                      \
	} while (0);




Wifi::Wifi() : password(S(WIFI_PASS)),prefix(S(WIFI_SSID)) {
	rssi = 0;
}

Wifi::~Wifi() {
}

void Wifi::init() {
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
}


//#define BZERO(x) ::memset(&x, sizeof(x), 0)
static const char* TAG = "MQTT_EXAMPLE";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

//const char* getIpAddress() { return my_ip_address; }

esp_err_t Wifi::wifi_event_handler(void* ctx, system_event_t* event) {
	Wifi& wifi = *(Wifi*) ctx;
	switch (event->event_id) {

		case SYSTEM_EVENT_SCAN_DONE: {
				INFO("SYSTEM_EVENT_SCAN_DONE");
				if ( wifi.scanDoneHandler() ) {
					wifi.connectToAP(wifi.ssid().c_str());
				} else {
					wifi.startScan();
				}
				break;
			}
		case SYSTEM_EVENT_STA_STOP: {
				INFO("SYSTEM_EVENT_STA_STOP");
				esp_wifi_start();
//				wifi.wifiInit();
				break;
			}
		case SYSTEM_EVENT_STA_START: {
				INFO("SYSTEM_EVENT_STA_START");
				wifi.startScan();
				break;
			}
		case SYSTEM_EVENT_STA_GOT_IP: {
				INFO("SYSTEM_EVENT_STA_GOT_IP");
				system_event_sta_got_ip_t* got_ip = &event->event_info.got_ip;
				char my_ip_address[20];
				ip4addr_ntoa_r((ip4_addr_t*)&got_ip->ip_info.ip, my_ip_address, 20);
				wifi.ipAddress = my_ip_address;
				wifi.connected = true;
				break;
			}
		case SYSTEM_EVENT_STA_CONNECTED: {
				INFO("SYSTEM_EVENT_STA_CONNECTED");
				break;
			}
		case SYSTEM_EVENT_STA_DISCONNECTED: {
				INFO("SYSTEM_EVENT_STA_DISCONNECTED");
				wifi.connected = false;
				esp_wifi_connect();
				break;
			}

		default:
			INFO("unknown wifi event %d ", event->event_id);
			break;
	}

	return ESP_OK;
}

void Wifi::connectToAP(const char* ssid) {
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

bool Wifi::scanDoneHandler() {
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
		WARN(" no AP found matching pattern '%s', restarting scan.", prefix());
		return false;
	}
	this->ssid = (const char*) apRecords[strongestAP].ssid;
	INFO(" found strongest AP : '%s'.",ssid().c_str());
	return true;
}

void Wifi::startScan() {
	INFO(" starting WiFi scan.");
	wifi_scan_config_t scanConfig = {
		NULL, NULL, 0, false, WIFI_SCAN_TYPE_ACTIVE, { 0, 0 }
	};
	CHECK(esp_wifi_scan_start(&scanConfig, false));
}

void Wifi::wifiInit() {
	nvs_flash_init();

	tcpip_adapter_init();
	CHECK(esp_event_loop_init(wifi_event_handler, this));
	wifi_init_config_t wifiInitializationConfig = WIFI_INIT_CONFIG_DEFAULT()
	        ;
	CHECK(esp_wifi_init(&wifiInitializationConfig));
	CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	CHECK(esp_wifi_set_promiscuous(false));
	CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B));
	CHECK(esp_wifi_start());
}
