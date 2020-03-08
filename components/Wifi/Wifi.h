#ifndef WIFI_H
#define WIFI_H
#define ESP_IDF

#include <Log.h>
#include <NanoAkka.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#ifndef WIFI_SSID
#error "WIFI_SSID should be defined !"
#endif

#ifndef WIFI_PASS
#error "WIFI_PASS should be defined !"
#endif

#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)

class Wifi {
		uint8_t _mac[6];

	public:
		ValueSource<bool> connected=false;
		ValueSource<int> rssi;
		ValueSource<std::string> ipAddress;
		ValueSource<std::string> ssid;
		ValueSource<uint64_t> mac;
		ValueSource<std::string> macAddress;
		ValueSource<std::string> password;
		ValueSource<std::string> prefix;

		Wifi( );
		~Wifi();
		void init();
		static esp_err_t wifi_event_handler(void* ctx, system_event_t* event);
		bool scanDoneHandler();
		void connectToAP(const char* AP);
		void startScan();
		void wifiInit();
};

#endif // WIFI_H
