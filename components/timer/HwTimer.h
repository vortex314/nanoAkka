#ifndef HWTIMER_H
#define HWTIMER_H

#include <NanoAkka.h>
#include <Device.h>
#include <Hardware.h>
#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"


class HwTimer
{
    bool _autoReload=false;
    double _intervalSec;
    timer_idx_t _timerIdx;
    timer_group_t _timerGroup;
    uint32_t _counter=0;
    DigitalOut& _pulsePin;
public:
    HwTimer(uint32_t pin);
    ~HwTimer();
    static void  timer_group0_isr(void *para);
    void isr();
    ValueFlow <bool>autoReload=false;
    ValueFlow <double>intervalSec;
    ValueFlow <uint32_t> divider = 8000;
    ValueFlow <uint32_t> ticks =0;
    void tick();
    void init();
    void wiring();
    void config(uint32_t divider,bool autoReload,double interval);
    void start();
    void stop();
};

#endif // HWTIMER_H
