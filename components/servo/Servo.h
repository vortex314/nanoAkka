#ifndef MOTORSERVO_H
#define MOTORSERVO_H

#include <Hardware.h>
#include <NanoAkka.h>
#include <MedianFilter.h>
#include <BTS7960.h>
#include <Device.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define SERVO_MAX_SAMPLES 16

class Servo : public Device
{

// D34 : L_IS
// D35 : R_IS
// D25 : ENABLE
// D26 : L_PWM
// D27 : R_PWM
// D32 : ADC POT
    BTS7960 _bts7960;
    ADC& _adcPot;
    MedianFilter<int,11> _potFilter;

    mcpwm_unit_t _mcpwm_num;
    mcpwm_timer_t _timer_num;

    float _bias=0;
    float _error=0;
    float _errorPrior=0;
    float _iteration_time=1;
//   float _angleSamples[SERVO_MAX_SAMPLES];
    uint32_t _indexSample=0;
    int _watchdogCounter;
    int _directionTargetLast;
    TimerSource _pulseTimer; // generate test cycle
    TimerSource _reportTimer; // report to MQTT
    TimerSource _controlTimer; // PID loop interval
public:
    ValueFlow <int> adcPot=0;
    ValueFlow<int> angleTarget=0;
    ValueFlow<int> angleMeasured=0;
    ValueFlow<float> KP= 1;
    ValueFlow<float> KI=0.75;
    ValueFlow<float> KD= 0.0;
    ValueFlow<float> pwm=0.0,error=0.0;
    ValueFlow<float> proportional=0.0,integral=0.0,derivative=0.0;
    ValueFlow<float> current=0.0;
    ValueFlow<bool> keepGoing;
    Servo(Thread& thr,Connector* connector);
    Servo(Thread& thr,uint32_t pinPot, uint32_t pinIS,
          uint32_t pinLeftEnable, uint32_t pinRightEnable,
          uint32_t pinLeftPwm, uint32_t pinRightPwm);
    ~Servo();
    void init();
    void observeOn(Thread& thread);

    void calcTarget(float);
    float PID(float error, float interval);
    float filterAngle(float inp);
    void round(float& f,float resol);
    bool measureAngle();
    float scale(float x,float x1,float x2,float y1,float y2);
    bool stopOutOfRange(int adc);
};


#endif // MOTORSERVO_H
