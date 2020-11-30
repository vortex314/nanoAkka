#include "StepperServo.h"

#define CONTROL_INTERVAL_MS 10000
#define ANGLE_MIN -90.0
#define ANGLE_MAX 90.0

#define STEPS 200       // steper motor steps per rotation
#define STEP_FACTOR 16  // reduction factor of gear
#define STEP_MAX (+STEP_FACTOR * STEPS / 4)  // 90 degrees
#define STEP_MIN (-STEP_FACTOR * STEPS / 4)  // -90 degrees

#define MAX_INTEGRAL 30

#define TEETH_BIG_GEAR 82.0
#define TEETH_SMALL_GEA 10.0

StepperServo::StepperServo(Thread &thr, Connector &uext, As5600 &as5600)
    : Actor(thr),
      Device(thr),
      _as5600(as5600),
      _uext(uext),
      _pulser(uext.toPin(LP_TXD)),
      _pinDir(uext.getDigitalOut(LP_SCL)),
      _pinEnable(uext.getDigitalOut(LP_SDA)),
      _measureTimer(thr, 4, 100, true),
      _controlTimer(thr, 3, CONTROL_INTERVAL_MS, true),
      _reportTimer(thr, 2, 500, true),
      stepsPerRotation("stepper/stepsPerRotation", STEPS * STEP_FACTOR) {
  _pulser.divider = 80;
  _pulser.intervalSec = 0.0008;
  _pulser.autoReload = true;
  stepsPerRotation = STEPS * STEP_FACTOR;
}

StepperServo::~StepperServo() {}

void StepperServo::init() {
  INFO(" servo init().");
  _as5600.onFailure(this, [](void *pv, const char *s) {
    StepperServo *me = (StepperServo *)pv;
    me->errorCount = me->errorCount() + 1;
    me->_measurementError = true;
    WARN(" StepperServo AS5600 : %s ", s);
    if (me->errorCount() > 10000) me->stop(s);
  });

  _pulser.init();
  _pinDir.setMode(DigitalOut::DOUT_PULL_UP);
  _pinEnable.setMode(DigitalOut::DOUT_PULL_UP);
  _pinDir.init();
  _pinDir.write(1);
  _pinEnable.init();
  _pinEnable.write(0);

  angleTarget >> ([&](const int &angle) {
    if (angle < -90) angleTarget = -90;
    if (angle > 90) angleTarget = 90;
  });

  _measureTimer >> ([&](TimerMsg tm) {
    int angle = _as5600.degrees();
    if (!_measurementError) { // drop i2c errors
      angleMeasured = angle;
      stepTarget = angleToSteps(angleMeasured() - angleTarget());
      INFO(" angle measured:%d target:%d steps:%d ", angleMeasured(),
           angleTarget(), stepTarget());
    } else {
      _measurementError = false;
    }
  });

  stepTarget >> ([&](const int &delta) {
    if (isRunning()) {
      if (true /*isDriving()*/) {
 //       if (!_pulser.busy()) {  // previous stepped stopped
          if (delta < 0) {
            _direction = -1;
            _pinDir.write(0);
            _pulser.ticks = -delta;
          } else if ( delta > 0) {
            _direction = 1;
            _pinDir.write(1);
            _pulser.ticks = delta;
          } else {
            stopStepper();
          }
          if (_pulser.ticks()) {
            _pinEnable.write(0);
            _pulser.start();
          }
  //      } else {
   //       INFO("pulser busy ");
 //       }
      } else {
        INFO("not DRIVING");
        stopStepper();
      }
    } else {
      INFO("not RUNNING");
      stopStepper();
    }
  });

  _controlTimer >> ([&](TimerMsg tm) {

  });
  angleTarget.pass(true);
  angleMeasured.pass(true);
  stepTarget.pass(true);
  stepMeasured.pass(true);
  run();
}

int StepperServo::angleToSteps(int angle) {
  float steps = (angle * STEPS * STEP_FACTOR) / 360.0;
  return steps;
}

void StepperServo::holdAngle() {
  _pulser.stop();
  _pulser.ticks = 0;
  _pinEnable.write(0);  // enable current
}

void StepperServo::stopStepper() {
  _pulser.stop();
  _pulser.intervalSec = 0.001;
  _pulser.ticks = 0;
  _pinEnable.write(1);  // disable: stop current through stepper motor
}

bool StepperServo::stopOutOfRange(float angle) {
  if (angle < -180 || angle > 180) {
    if (deviceState() != STOPPED) {
      std::string s;
      string_format(s, "  angle out of Range : %f  ", angle);
      stop(s.c_str());
    }
    return true;
  }
  return false;
}

float StepperServo::scale(float x, float x1, float x2, float y1, float y2) {
  float y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
  return y;
}
