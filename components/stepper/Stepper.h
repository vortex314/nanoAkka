#ifndef STEPPER_H
#define STEPPER_H
#include <NanoAkka.h>
#include <Hardware.h>


class Stepper : public Actor
{
    Connector& _uext;
    DigitalOut& _pinStep;
    DigitalOut& _pinDir;
    DigitalOut& _pinEnable;
    TimerSource _stepTimer;
    int _stepCounter=0;
    int _stepUp=0;
    int _direction=0;
public:
    ValueFlow<int> angleTarget=0;
    ValueFlow<int> stepTarget=0;
    ValueFlow<int> stepMeasured=0;
    Stepper(Thread& thr,Connector& uext);
    ~Stepper();
    void init();


};

#endif // STEPPER_H
