#include "Stepper.h"

#define MICROSTEP 1
#define STEPS_PER_DEG (200 * MICROSTEP * 20) / 360

Stepper::Stepper(Thread &thr, Connector &uext)
    : Actor(thr),
      Device(thr),
      _uext(uext),
      _pulser(uext.toPin(LP_TXD)),
      _pinDir(uext.getDigitalOut(LP_SCL)),
      _pinEnable(uext.getDigitalOut(LP_SDA)) {
  _pulser.divider = 80;
  _pulser.intervalSec = 0.001;
  _pulser.autoReload = true;
  angleTarget >> ([&](const int &deg) { stepTarget = deg * STEPS_PER_DEG; });

  _pulser.busy >> ([&](const bool &busy) {
    INFO(" pulser : %s target : %d vs measured : %d", busy ? "busy" : "free",
         stepTarget(), stepMeasured());
    if (!busy) {
      if (stepTarget() == stepMeasured()) {
        _pinEnable.write(1);
      } else {
        stepTarget.request(); 
      }
    }
  });

  auto stepHandler = new Sink<int,3>();
  stepHandler->async(thread(),[&](const int &st) {
    INFO(" target:%d measured:%d dir:%d pulser:%d", stepTarget(),
         stepMeasured(), _direction, _pulser.busy());
    if (!_pulser.busy()) {  // previous stepped stopped
      int delta = st - stepMeasured();
      stepMeasured = st;
      if (delta > 0) {
        _direction = 1;
        _pinDir.write(1);
        _pulser.ticks = delta;
        _pinEnable.write(0);
        _pulser.start();
      } else {
        _direction = -1;
        _pinDir.write(0);
        _pulser.ticks = -delta;
        _pinEnable.write(0);
        _pulser.start();
      }
      INFO(" target:%d measured:%d dir:%d", stepTarget(), stepMeasured(),
           _direction);
    }
  });

  stepTarget >> stepHandler;
  angleTarget.pass(true);
  stepTarget.pass(true);
}

void Stepper::holdAngle() {
  _pulser.stop();
  _pulser.ticks = 0;
  _pinEnable.write(0);  // enable current
}

void Stepper::stopStepper() {
  _pulser.stop();
  _pulser.intervalSec = 0.001;
  _pulser.ticks = 0;
  _pinEnable.write(1);  // disable: stop current through stepper motor
}

Stepper::~Stepper() {}

void Stepper::init() {
  INFO("init()..");
  _pulser.init();
  _pinEnable.setMode(DigitalOut::DOUT_PULL_UP);
  _pinDir.init();
  _pinDir.write(1);
  _pinEnable.init();
  _pinEnable.write(1);
  INFO("init() done.");
}
