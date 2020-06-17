#ifndef STEPPER_SERVO_H
#define STEPPER_SERVO_H
#include <ConfigFlow.h>
#include <Device.h>
#include <Hardware.h>
#include <MedianFilter.h>
#include <NanoAkka.h>
#include <Pulser.h>

class StepperServo : public Actor, public Device
{
  Connector &_uext;
  Pulser _pulser;
  DigitalOut &_pinDir;
  DigitalOut &_pinEnable;
  ADC &_adcPot;
  int _stepUp = 0;
  int _direction = 0;
  TimerSource _measureTimer; // ADC multi sample and median
  TimerSource _controlTimer; // PID loop interval
  TimerSource _reportTimer;  // report to MQTT
  MedianFilter<int, 10> _potFilter;
  float _error = 0;
  float _errorPrior = 0;

public:
  ConfigFlow<int> stepsPerRotation;
  ValueFlow<int> adcPot = 0;
  ValueFlow<int> angleTarget = 0;
  ValueFlow<int> angleMeasured = 0;
  ValueFlow<int> stepTarget = 0;
  ValueFlow<int> stepMeasured = 0;
  ValueFlow<float> error = 0.0;
  ValueFlow<float> proportional = 0.0, integral = 0.0, derivative = 0.0;
  ValueFlow<float> KP = 1.0;
  ValueFlow<float> KI = 0.0;
  ValueFlow<float> KD = 0.0;
  ValueFlow<float> output = 0.0;
  ValueFlow<bool> isDriving = false;
  StepperServo(Thread &thr, Connector &uext);
  ~StepperServo();
  void init();
  bool measureAngle();
  bool stopOutOfRange(int adc);
  float scale(float x, float x1, float x2, float y1, float y2);
  float PID(float error, float interval);
  void holdAngle();
  void stopStepper();
};

#endif // STEPPER_H
