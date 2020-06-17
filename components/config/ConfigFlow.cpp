#include "ConfigFlow.h"


#if defined(ESP32_IDF) || defined(ESP8266_IDF) 
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
nvs_handle _nvs = 0;

void ConfigStore::init()
{
    if (_nvs != 0)
        return ;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    err = nvs_open("storage", NVS_READWRITE, &_nvs);
    if (err != ESP_OK)
        WARN(" non-volatile storage open fails.");
}
bool ConfigStore::load(const char *name, void *value, uint32_t length)
{
    size_t required_size = length;
    esp_err_t err = nvs_get_blob(_nvs, name, value, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        WARN(" nvs_get_blob() : %d ",err);
        return false;
    }
    return true;
}

bool ConfigStore::save(const char *name, void *value, uint32_t length)
{
    INFO(" Config saved : %s ", name);
    esp_err_t err = nvs_set_blob(_nvs, name, value, length);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        WARN(" nvs_set_blob() : %d ",err);
        return false;
    }
    nvs_commit(_nvs);
    return true;
}
bool ConfigStore::load(const char *name, std::string* value,uint32_t length)
{
    char buffer[256];
    size_t required_size = sizeof(buffer);
    esp_err_t err = nvs_get_str(_nvs, name, buffer, &required_size);
    if (err == ESP_OK) {
        *value = buffer;
        return true;
    } else {
        WARN(" nvs_get_str(%s) : %d ",name,err);
        return false;
    }
}

bool ConfigStore::save(const char *name, std::string* value,uint32_t length)
{
    char buffer[256];
    strncpy(buffer, value->c_str(), sizeof(buffer) - 1);
    esp_err_t err = nvs_set_str(_nvs, name, buffer);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        WARN(" nvs_set_str(%s,%s) : %d ",name,buffer,err);
        return false;
    }
    nvs_commit(_nvs);
    return true;
}

#endif
