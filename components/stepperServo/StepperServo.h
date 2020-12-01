#ifndef STEPPER_SERVO_H
#define STEPPER_SERVO_H
#include <As5600.h>
#include <ConfigFlow.h>
#include <Device.h>
#include <Hardware.h>
#include <MedianFilter.h>
#include <NanoAkka.h>
#include <Pulser.h>

class StepperServo : public Actor, public Device {
  As5600 &_as5600;
  Connector &_uext;
  Pulser _pulser;
  DigitalOut &_pinDir;
  DigitalOut &_pinEnable;
  int _stepUp = 0;
  int _direction = 0;
  TimerSource _measureTimer;  // ADC multi sample and median
  TimerSource _controlTimer;  // PID loop interval
  TimerSource _reportTimer;   // report to MQTT
  MedianFilter<int, 10> _potFilter;
  float _error = 0;
  float _errorPrior = 0;
  bool _measurementError = false;

 public:
  ConfigFlow<int> stepsPerRotation;
  ValueFlow<int> errorCount = 0;
  ValueFlow<int> angleTarget = 0;
  ValueFlow<int> angleMeasured = 0;
  ValueFlow<int> stepTarget = 0;
  ValueFlow<bool> isDriving = false;
  ValueFlow<int> stepDirection;
  StepperServo(Thread &thr, Connector &uext, As5600 &as5600);
  ~StepperServo();
  void init();
  bool measureAngle();
  bool stopOutOfRange(float angle);
  float scale(float x, float x1, float x2, float y1, float y2);
  int angleToSteps(int angle);
  float PID(float error, float interval);
  void holdAngle();
  void stopStepper();
};

#endif  // STEPPER_H
