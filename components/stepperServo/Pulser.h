#ifndef HWTIMER_H
#define HWTIMER_H

#include <Device.h>
#include <Hardware.h>
#include <NanoAkka.h>

#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_types.h"

class Pulser {
  bool _autoReload = true;
  double _intervalSec;
  timer_idx_t _timerIdx;
  timer_group_t _timerGroup;
  uint32_t _counter = 0;
  DigitalOut& _pulsePin;
  void wiring();
  void config(uint32_t divider, bool autoReload, double interval);
  static void timer_group0_isr(void* para);
  void isr();

 public:
  Pulser(uint32_t pin);
  ~Pulser();
  ValueFlow<bool> autoReload = true;
  ValueFlow<double> intervalSec;
  ValueFlow<uint32_t> divider = 8000;
  ValueFlow<uint32_t> ticks = 0;
  bool busy = false;
  void tick();
  void init();
  void start();
  void stop();
};

#endif  // HWTIMER_H
