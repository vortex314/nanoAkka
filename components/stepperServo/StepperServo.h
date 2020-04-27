#ifndef STEPPER_SERVO_H
#define STEPPER_SERVO_H
#include <NanoAkka.h>
#include <Hardware.h>
#include <Device.h>
#include <MedianFilter.h>


class StepperServo : public Actor,public Device
{
    Connector& _uext;
    DigitalOut& _pinStep;
    DigitalOut& _pinDir;
    DigitalOut& _pinEnable;
    ADC& _adcPot;
    TimerSource _stepTimer;
    int _stepCounter=0;
    int _stepUp=0;
    int _direction=0;
    TimerSource _measureTimer; // ADC multi sample and median
    TimerSource _controlTimer; // PID loop interval
    TimerSource _reportTimer; // report to MQTT
    MedianFilter<int,10> _potFilter;
    float _error=0;
    float _errorPrior=0;

public:
    ValueFlow <int> adcPot=0;
    ValueFlow<int> angleTarget=0;
    ValueFlow<int> angleMeasured=0;
    ValueFlow<int> stepTarget=0;
    ValueFlow<int> stepMeasured=0;
    ValueFlow<float> error=0.0;
    ValueFlow<float> proportional=0.0,integral=0.0,derivative=0.0;
    ValueFlow<float> KP= 0.5;
    ValueFlow<float> KI=2.0;
    ValueFlow<float> KD= 0.0;
    StepperServo(Thread& thr,Connector& uext);
    ~StepperServo();
    void init();
    bool measureAngle();
    bool stopOutOfRange(int adc);
    float scale(float x,float x1,float x2,float y1,float y2);
    float PID(float error, float interval);


};

#endif // STEPPER_H
