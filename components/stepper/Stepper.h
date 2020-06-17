#ifndef STEPPER_H
#define STEPPER_H
#include <NanoAkka.h>
#include <Hardware.h>
#include <Pulser.h>

class Stepper : public Actor, public Device
{
    Connector &_uext;
    Pulser _pulser;
    DigitalOut &_pinDir;
    DigitalOut &_pinEnable;
    int _stepUp = 0;
    int _direction = 0;

public:
    ValueFlow<int> angleTarget = 0;
    ValueFlow<int> stepTarget = 0;
    ValueFlow<int> stepMeasured = 0;
    Stepper(Thread &thr, Connector &uext);
    ~Stepper();
    void init();
    void holdAngle();
    void stopStepper();
};

#endif // STEPPER_H
