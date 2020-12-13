#ifndef _MQTT_WIFI_H_
#define _MQTT_WIFI_H_
extern "C" {
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
//#include "mqtt_client.h"
#include "mqtt_client.h"
}
#include <NanoAkka.h>
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <ArduinoJson.h>
#include <Mqtt.h>
#include <MqttOta.h>

#define QOS 0
#define TIMEOUT 10000L

class MqttWifi : public Mqtt {
  StaticJsonDocument<3000> _jsonBuffer;
  std::string _clientId;
  std::string _address;
  esp_mqtt_client_handle_t _mqttClient;
  std::string _lwt_topic;
  std::string _lwt_message;
  TimerSource _reportTimer;
  TimerSource _keepAliveTimer;
  std::string _lastTopic;
  MqttOta mqttOta;

 public:
  Sink<bool, 2> wifiConnected;
  ValueSource<bool> connected;
  TimerSource keepAliveTimer;
  MqttWifi(Thread& thread);
  ~MqttWifi();
  void init();

  void mqttPublish(const char* topic, const char* message);
  void mqttSubscribe(const char* topic);
  void mqttConnect();
  void mqttDisconnect();

  bool handleMqttMessage(const char* message);
  static int mqtt_event_handler(esp_mqtt_event_t* event);

  void onNext(const TimerMsg&);
  void onNext(const MqttMessage&);
  void request();
 /* template <class T>
  Subscriber<T>& toTopic(const char* name) {
    auto flow = new ToMqtt<T>(name,srcPrefix);
    *flow >> outgoing;
    return *flow;
  }
  template <class T>
  Source<T>& fromTopic(const char* name) {
    auto newSource = new FromMqtt<T>(name,dstPrefix);
    incoming >> *newSource;
    return *newSource;
  }*/
  /*
                          template <class T>
                          MqttFlow<T>& topic(const char* name) {
                                  auto newFlow = new MqttFlow<T>(name);
                                  incoming >> newFlow->mqttIn;
                                  newFlow->mqttOut >> outgoing;
                                  return *newFlow;
                          }
                          void observeOn(Thread& thread);*/
};

//_______________________________________________________________________________________________________________
//

#endif
