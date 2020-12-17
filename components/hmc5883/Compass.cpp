#include "Compass.h"

Compass::Compass(Thread& thread,Connector& connector) :
    Actor(thread),
    Device(thread),
    _uext(connector),
    measureTimer(thread,100,true),
    _v(
{
    0, 0, 0
})
{
    _uext=connector;
    _hmc = new 	HMC5883L(_uext);
    x=0;
    y=0;
    z=0;
    status=0;
};

Compass::~Compass()
{
}
void Compass::init()
{

    if(_hmc->init()) {
        INFO("HMC5883L initialized.");
    } else {
        ERROR("HMC5883L initialization failed.");
        stop("HMC5883L initialization failed.");
        return;
    }
    _hmc->setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    _hmc->setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    _hmc->setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    _hmc->setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    _hmc->setOffset(0, 0);
    measureTimer >> [&](const TimerMsg& tm) {
        if ( isRunning() ) {
            _v = _hmc->readNormalize();
//		INFO("%f:%f:%f", _v.XAxis, _v.YAxis, _v.ZAxis);
            x = x() + (_v.x - x()) / 4;
            y = y() + (_v.y - y()) / 4;
            z = z() + (_v.z - z()) / 4;
            status = _hmc->readRegister8(HMC5883L_REG_STATUS);
        }
    };
    run();
}
