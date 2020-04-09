#ifndef COMPASS_H
#define COMPASS_H
#include <NanoAkka.h>
#include <HMC5883L.h>



class Compass : public Actor
{
    Connector& _uext;
    HMC5883L* _hmc;
    TimerSource measureTimer;
    struct Vector<float> _v;

public:
    ValueSource<int32_t> x,y,z,status;
    Compass(Thread&,Connector&);
    virtual ~Compass() ;
    void init();
};

#endif // COMPASS_H
