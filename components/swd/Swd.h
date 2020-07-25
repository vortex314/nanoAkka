#ifndef SWD_H
#define SWD_H
#include <Mqtt.h>
#include <NanoAkka.h>
#include <driver/spi_master.h>
extern "C" {
#include <libswd.h>
};
class Ota {
  virtual bool startOta(const MqttBlock&) = 0;
  virtual bool stopOta(const MqttBlock&) = 0;
  virtual bool writeOta(const MqttBlock&) = 0;
};

class Swd : public Actor, public Ota {
  libswd_ctx_t* libswdctx = NULL;
  spi_device_handle_t deviceSPI;
  spi_device_interface_config_t confSPI;
  spi_bus_config_t pinsSPI;

 public:
  Sink<MqttBlock, 10> ota;

  Swd(Thread&, uint32_t, uint32_t, uint32_t);
  void init();
  void test();
  bool startOta(const MqttBlock&);
  bool stopOta(const MqttBlock&);
  bool writeOta(const MqttBlock&);
};
#endif