#include <MqttOta.h>

MqttOta::MqttOta() {}

void MqttOta::init() {
  blocks >> [&](const MqttBlock &block) {
    if (block.topic.find("system/ota") != std::string::npos) onBlock(block);
  };
}

void MqttOta::onBlock(const MqttBlock &block) {
  if (block.offset == 0) {
    init();
    initUpgrade();
  }
  writeUpgrade(block.data, block.length);
  if (block.offset + block.length == block.length) {
    endUpgrade();
    execUpgrade();
  }
}

int MqttOta::initUpgrade() {
  INFO("initUpgrade");
  _state = READY;
  update_partition = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
  if (update_partition == NULL) {
    ERROR("Passive OTA partition not found");
    return ENOBUFS;
  };
  INFO("Writing to partition subtype %d at offset 0x%x",
       update_partition->subtype, update_partition->address);
  esp_err_t err =
      esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
  if (err != ESP_OK) {
    ERROR("esp_ota_begin failed, error=%d", err);
    return err;
  }
  _state = INIT;
  INFO("initUpgrade OK");
  return 0;
}

int MqttOta::writeUpgrade(uint8_t *data, uint32_t length) {
  esp_err_t ota_write_err =
      esp_ota_write(update_handle, (const void *)data, length);
  if (ota_write_err != ESP_OK) return ENOBUFS;
  _lengthWritten += length;
  return 0;
}

int MqttOta::endUpgrade() {
  _state = READY;
  INFO("endUpgrade");
  esp_err_t ota_end_err = esp_ota_end(update_handle);
  if (ota_end_err != ESP_OK) {
    ERROR("Error: esp_ota_end failed! err=0x%d. Image is invalid", ota_end_err);
    return ota_end_err;
  }
  INFO("endUpgrade OK");
  return 0;
}

esp_err_t MqttOta::execUpgrade() {
  _state = READY;
  INFO("execUpgrade");
  esp_err_t err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    ERROR("esp_ota_set_boot_partition failed! err=0x%d", err);
    return err;
  }
  INFO("esp_ota_set_boot_partition succeeded");
  esp_restart();
  return ESP_OK;
}