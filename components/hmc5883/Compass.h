#ifndef COMPASS_H
#define COMPASS_H
#include <NanoAkka.h>
#include <HMC5883L.h>
#include <Device.h>



class Compass : public Actor,public Device
{
    Connector& _uext;
    HMC5883L* _hmc;
    TimerSource measureTimer;
    TimerSource reportTimer;
    struct Vector<float> _v;

public:
    ValueSource<int32_t> x,y,z,status;
    Compass(Thread&,Connector&);
    virtual ~Compass() ;
    void init();
};

#endif // COMPASS_H
