#include "Pulser.h"

Pulser::Pulser(uint32_t pin) : _pulsePin(DigitalOut::create(pin)) {
  _timerGroup = TIMER_GROUP_0;
  _timerIdx = (timer_idx_t)TIMER_0;
  intervalSec = 0.1;
  divider = 16;
}

Pulser::~Pulser() {}

#define TIMER_DIVIDER \
  800  //  Hardware timer clock divider, 80 Mhz => 10kHz => 0.1 mS
#define TIMER_SCALE \
  (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITHOUT_RELOAD 0       // testing will be done without auto reload
#define TEST_WITH_RELOAD 1          // testing will be done with auto reload

#define CHECK(xxx) \
  if (xxx != ESP_OK) WARN(#xxx " failed.")

void Pulser::init() {
  INFO(" Pulser::init() pin:%d ", _pulsePin.getPin());
  _pulsePin.setMode(DigitalOut::DOUT_PULL_UP);
  _pulsePin.init();
  _pulsePin.write(0);
  config(divider(), autoReload(), intervalSec());
  wiring();
  busy = false;
}
void Pulser::config(uint32_t div, bool reload, double interval) {
  INFO(" Timer config group:%d timer:%d divider:%d interval:%f reload:%d",
       _timerGroup, _timerIdx, div, interval, reload);

  timer_config_t config;
  BZERO(config);
  config.divider = div;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.auto_reload = reload ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS;
  config.intr_type = TIMER_INTR_LEVEL;

  // default clock source is APB
  CHECK(timer_init(_timerGroup, _timerIdx, &config));

  /* Timer's counter will initially start from value below.
     Also, if auto_reload is set, this value will be automatically reload on
     alarm */
  CHECK(timer_set_counter_value(_timerGroup, _timerIdx, 0x00000000ULL));

  /* Configure the alarm value and the interrupt on alarm. */
  CHECK(timer_set_alarm_value(_timerGroup, _timerIdx,
                              interval * (TIMER_BASE_CLK / div)));
  CHECK(timer_enable_intr(_timerGroup, _timerIdx));
}

void Pulser::start() {
  if (!busy) {
    INFO("start %u", ticks());
    CHECK(timer_start(_timerGroup, _timerIdx));
    busy = true;
  }
}

void Pulser::wiring() {
  intervalSec >> [&](const double& interval) {
    if (!busy && intervalSec() != interval) {
      config(divider(), autoReload(), interval);
      INFO(" interval %.4f sec", intervalSec);
    }
  };
  autoReload >> [&](const bool& reload) {
    if (!busy && autoReload() != reload) {
      config(divider(), reload, intervalSec());
      INFO(" reload : %s ", reload ? "true" : "false");
    }
  };
  divider >> [&](const uint32_t& div) {
    if (!busy && div != divider()) {
      config(div, autoReload(), intervalSec());
      INFO(" divider : %u ", div);
    }
  };

  ticks >> [&](const uint32_t& t) { _counter = 2 * t; };

  CHECK(timer_isr_register(_timerGroup, _timerIdx, timer_group0_isr, this,
                           ESP_INTR_FLAG_IRAM, NULL));
  ticks.pass(true);
  divider.pass(true);
  autoReload.pass(true);
  intervalSec.pass(true);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with
 * IRAM_ATTR. If we're okay with the timer irq not being serviced while SPI
 * flash cache is disabled, we can allocate this interrupt without the
 * ESP_INTR_FLAG_IRAM flag and use the normal API.
 *
 *
 */
void IRAM_ATTR Pulser::stop() {
  CHECK(timer_pause(_timerGroup, _timerIdx));
  busy = false;
}

void IRAM_ATTR Pulser::timer_group0_isr(void* para) {
  Pulser* pulser = (Pulser*)(para);
  pulser->isr();
}

void IRAM_ATTR Pulser::tick() {
  if (_counter & 1)
    _pulsePin.write(1);
  else
    _pulsePin.write(0);

  if (_counter == 0)
    stop();
  else
    _counter--;
}

void IRAM_ATTR Pulser::isr() {
  //   timer_group_t _timerGroup = TIMER_GROUP_0;
  //   timer_idx_t _timerIdx = TIMER_0;
  timer_spinlock_take(_timerGroup);

  /* Retrieve the interrupt status and the counter value
     from the timer that reported the interrupt */
  //   uint32_t timer_intr = timer_group_get_intr_status_in_isr(_timerGroup);
  //    uint64_t timer_counter_value =
  //    timer_group_get_counter_value_in_isr(_timerGroup, _timerIdx);

  tick();
  /* Clear the interrupt
     and update the alarm time for the timer with without reload */
  /*    if (timer_intr & TIMER_INTR_T0) {
          timer_group_clr_intr_status_in_isr(_timerGroup, TIMER_0);
  //        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC *
  TIMER_SCALE);
  //        timer_group_set_alarm_value_in_isr(_timerGroup, _timerIdx,
  timer_counter_value); } else if (timer_intr & TIMER_INTR_T1) {
          timer_group_clr_intr_status_in_isr(_timerGroup, TIMER_1);
      } else {
      }*/

  /* After the alarm has been triggered
    we need enable it again, so it is triggered the next time */
  timer_group_clr_intr_status_in_isr(_timerGroup, _timerIdx);
  timer_group_enable_alarm_in_isr(_timerGroup, _timerIdx);
  timer_spinlock_give(_timerGroup);
}
