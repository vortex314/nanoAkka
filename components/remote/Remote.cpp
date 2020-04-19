#include "Remote.h"



Remote::Remote(Thread& thr) : Actor(thr),
    _ledLeft(DigitalOut::create(32)),
    _ledRight(DigitalOut::create(23)),
    _buttonLeft(DigitalIn::create(13)),
    _buttonRight(DigitalIn::create(16)),
    _adcLeft(ADC::create(36)),
    _adcRight(ADC::create(39)),
    _measureTimer(thr,1,100,true),
    _defaultTimer(thr,1,500,true),
    _leftButton( _buttonLeft,buttonLeft),
    _rightButton( _buttonRight,buttonRight)
{

    ledLeft.on(false);
    ledRight.on(false);
    ledLeft.sync([&](bool b) {
        _ledLeft.write(b?0:1);
    });
    ledRight.sync([&](bool b) {
        _ledRight.write(b?0:1);
    });

    _defaultTimer >> *(new Sink<TimerMsg,2>([&](const TimerMsg& tm) {
        buttonLeft.request();
        buttonRight.request();
        potLeft.request();
        potRight.request();
    }));
    _measureTimer >> *(new Sink<TimerMsg,2>([&](const TimerMsg& tm) {
        int pot = _adcLeft.getValue();
        if ( abs(pot-potLeft())>10) potLeft = pot;
        pot = _adcRight.getValue();
        if ( abs(pot-potRight())>10) potRight = pot;
    }));
}

void Remote::buttonChange(void* pButton)
{
    ButtonStruct* me = (ButtonStruct*)pButton;
    me->valueSource= (me->digitalIn.read() == 0);
}

void Remote::init()
{
    _ledLeft.init();
    _ledLeft.write(1);
    _ledRight.init();
    _ledRight.write(1);

    _buttonLeft.setMode(DigitalIn::DIN_PULL_UP);
    _buttonLeft.onChange(DigitalIn::DIN_CHANGE, buttonChange, &_leftButton);
    _buttonLeft.init();

    _buttonRight.setMode(DigitalIn::DIN_PULL_UP);
    _buttonRight.onChange(DigitalIn::DIN_CHANGE, buttonChange, &_rightButton);
    _buttonRight.init();

    _adcLeft.init();
    _adcRight.init();
};

Remote::~Remote()
{
}
