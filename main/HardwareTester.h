#ifndef HARDWARETESTER_H
#define HARDWARETESTER_H

#include <NanoAkka.h>
#include <Hardware.h>
#include <Register.h>
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/rtc.h"
#include "driver/pcnt.h"

typedef struct {
    const char* desc;
    const char* name;
    const char* format;
    uint32_t* address;
} Reg;

class HardwareTester
{
    int pinPwm1=18;
    int pinPwm2=16;
    float duty_cycle = 50;
    mcpwm_unit_t mcpwm_num = MCPWM_UNIT_0; // MCPWM_UNIT_1
    mcpwm_timer_t timer_num = MCPWM_TIMER_0;
    esp_err_t rc ;
    uint32_t mcpwm_intr_status;
    Connector _uext;

public:
    HardwareTester();
    ~HardwareTester();
    ValueSource<uint32_t> capts ;
    static Reg regs[];
    uint32_t captureNumberOfPulse=500;
    uint32_t pwmFrequency=1000;

    bool fromToGpio(int out,int in);
    void gpioTest();
    int mcpwmTest();
    int pwm(int dutyCycle);
    int captureTest();
    static void isrCaptureHandler(void* pv);

};

#endif // HARDWARETESTER_H
