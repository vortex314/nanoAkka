#ifndef MQTT_OTA_H
#define MQTT_OTA_H
#include <Base64.h>
#include <Bytes.h>
#include <NanoAkka.h>
#include <Mqtt.h>
#include <esp_https_ota.h>
#include <esp_ota_ops.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#define OTA_BUF_SIZE 300

class MqttOta {
  esp_partition_t* update_partition = 0;
  esp_ota_handle_t update_handle = 0;
  uint32_t _lengthWritten = 0;
  typedef enum { READY, INIT, WRITE, END } State;
  State _state = READY;

 public:
  ValueFlow<MqttBlock> blocks;
  MqttOta();
  void init();
  int initUpgrade();
  int writeUpgrade(uint8_t* data, uint32_t dataLength);
  int endUpgrade();
  void checkUpgrade();
  int execUpgrade();
  void onBlock(const MqttBlock& block);
};
#endif