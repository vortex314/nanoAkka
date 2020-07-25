#include <Swd.h>

Swd::Swd(Thread& thr, uint32_t pinSwdio, uint32_t pinSwclk, uint32_t pinSwo)
    : Actor(thr) {
  BZERO(pinsSPI);
  pinsSPI.mosi_io_num = pinSwdio;  // SWD I/O
  pinsSPI.miso_io_num = pinSwo;    // not connected
  pinsSPI.sclk_io_num = pinSwclk;  // SWD CLK
  pinsSPI.quadwp_io_num = -1;
  pinsSPI.quadhd_io_num = -1;
  pinsSPI.max_transfer_sz = 0;

  INFO("[spi_bus_initialize]");
  if (ESP_OK != spi_bus_initialize(HSPI_HOST, &pinsSPI, 0)) {  // No DMA
    INFO("[spi_bus_initialize] fail");
    return;  // Warning, this example does not close handles correctly
  }
  BZERO(confSPI);
  confSPI.command_bits = 0;
  confSPI.address_bits = 0;
  confSPI.dummy_bits = 0;
  confSPI.mode = 0;
  confSPI.duty_cycle_pos = 0;
  confSPI.cs_ena_pretrans = 0;
  confSPI.cs_ena_posttrans = 0;
  confSPI.clock_speed_hz = 1000000;
  confSPI.spics_io_num = -1;
  confSPI.flags =
      SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST;
  confSPI.queue_size = 24;
  confSPI.pre_cb = NULL;
  confSPI.post_cb = NULL;

  BZERO(deviceSPI);
  INFO("[spi_bus_add_device]");
  if (ESP_OK != spi_bus_add_device(HSPI_HOST, &confSPI, &deviceSPI)) {
    INFO("[spi_bus_add_device] fail");
    return;  // Warning, this example does not close handles correctly
  }

  INFO("[libswd_init]");
  libswdctx = libswd_init();
  if (libswdctx == nullptr) {
    INFO("[libswd_init] returned empty context");
    return;  // Warning, this example does not close handles correctly
  }

  libswd_log_level_set(libswdctx, LIBSWD_LOGLEVEL_DEBUG);
  libswdctx->driver->device = &deviceSPI;
}

void Swd::init() {
  ota.sync([&](const MqttBlock& msg) {
    if (msg.offset == 0) startOta(msg);
    writeOta(msg);
    if (msg.offset + msg.length == msg.total) stopOta(msg);
  });
}

void Swd::test() {
  uint32_t address = 0x08000000; // 0x20001000;  // Example address
  INFO("[libswd_memap_init]");
  auto memmap_res = libswd_memap_init(libswdctx, LIBSWD_OPERATION_EXECUTE);
  if (LIBSWD_OK != memmap_res) {
    INFO("[libswd_memap_init] failed");
    return;  // Warning, this example does not close handles correctly
  }

  const uint16_t buffCnt = 4;
  uint8_t buff[buffCnt] = {0};
  int read_res = libswd_memap_read_char(libswdctx, LIBSWD_OPERATION_EXECUTE,
                                        address, buffCnt, (char*)&buff);
  if (read_res < LIBSWD_OK) {
    INFO("[libswd_memap_read_char] FAILED");
    return;
  }

  char stringBuff[128];
  sprintf(stringBuff, "MEMAP read at %08X: %02X %02X %02X %02X", address,
          buff[0], buff[1], buff[2], buff[3]);
  INFO("%s", stringBuff);

  int idcode = 0;
  int* idcode_ptr = &idcode;
  INFO("[libswd_dap_detect]");
  auto dap_res =
      libswd_dap_detect(libswdctx, LIBSWD_OPERATION_EXECUTE, &idcode_ptr);
  if (LIBSWD_OK != dap_res) {
    INFO("[libswd_dap_detect] failed with code %d\n", dap_res);
    return;  // Warning, this example does not close handles correctly
  }

  char buff2[128];
  sprintf(buff2, "Detected IDCODE: 0x%08X\n", *idcode_ptr);
  INFO("%s", buff2);
}

bool Swd::startOta(const MqttBlock& block) {
  test();
  return true;
}
bool Swd::stopOta(const MqttBlock& block) { return true; }
bool Swd::writeOta(const MqttBlock& block) { return true; }
