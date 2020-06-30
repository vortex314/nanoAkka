#include "Stepper.h"

#define MICROSTEP 1
#define STEPS_PER_DEG (200 * MICROSTEP * 20) / 360

Stepper::Stepper(Thread &thr, Connector &uext)
    : Actor(thr),
      Device(thr),
      _uext(uext),
      _pulser(uext.toPin(LP_TXD)),
      _pinDir(uext.getDigitalOut(LP_SCL)),
      _pinEnable(uext.getDigitalOut(LP_SDA)),
      _pinCenter(uext.getDigitalIn(LP_SCK)),
      _pinLeft(uext.getDigitalIn(LP_MOSI)),
      _pinRight(uext.getDigitalIn(LP_CS)) {
  _pulser.divider = 80;
  _pulser.intervalSec = 0.001;
  _pulser.autoReload = true;
  angleTarget >> ([&](const int &deg) { stepTarget = deg * STEPS_PER_DEG; });

  auto busyHandler = new Sink<bool, 3>();
  busyHandler->async(thread(), [&](const bool &busy) {
    INFO(" pulser : %s target : %d vs measured : %d", busy ? "busy" : "free",
         stepTarget(), stepMeasured());
    if (!busy) {
      if (stepTarget() == stepMeasured()) {  // target has been reached
        _pinEnable.write(1);
      } else {  // invoke last Target
        stepTarget.request();
      }
    }
  });
  _pulser.busy >> busyHandler;

  msg.async(thread(), [&](const std::string &m) {
    INFO("%s", m.c_str());
    INFO("L:%d C:%d R:%d", _pinLeft.read(), _pinCenter.read(),
         _pinRight.read());
  });

  auto stepHandler = new Sink<int, 3>();
  stepHandler->async(thread(), [&](const int &st) {
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

void IRAM_ATTR Stepper::isrCenter(void *ptr) {
  Stepper *me = (Stepper *)ptr;
  me->stepMeasured = 0;
  me->msg.on("ISR-C");
}

void IRAM_ATTR Stepper::isrLeft(void *ptr) {
  Stepper *me = (Stepper *)ptr;
  me->msg.on("ISR-L");
}

void IRAM_ATTR Stepper::isrRight(void *ptr) {
  Stepper *me = (Stepper *)ptr;
  me->msg.on("ISR-R");
}

void Stepper::init() {
  INFO("init()..");
  _pulser.init();

  _pinDir.init();
  _pinDir.write(1);

  _pinEnable.setMode(DigitalOut::DOUT_PULL_UP);
  _pinEnable.init();
  _pinEnable.write(1);

  _pinCenter.setMode(DigitalIn::DIN_PULL_UP);
  _pinCenter.onChange(DigitalIn::DIN_FALL, isrCenter, this);
  _pinCenter.init();

  _pinLeft.setMode(DigitalIn::DIN_PULL_UP);
  _pinLeft.onChange(DigitalIn::DIN_FALL, isrLeft, this);
  _pinLeft.init();

  _pinRight.setMode(DigitalIn::DIN_PULL_UP);
  _pinRight.onChange(DigitalIn::DIN_FALL, isrRight, this);
  _pinRight.init();

  INFO("init() done.");
}
