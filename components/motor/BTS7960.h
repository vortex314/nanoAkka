#ifndef BTS7960_H
#define BTS7960_H
#include <Hardware.h>
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/rtc.h"

class BTS7960
{
    // D34 : L_IS
    // D35 : R_IS
    // D25 : ENABLE
    // D26 : L_PWM
    // D27 : R_PWM
    // D32 : ADC POT

    ADC& _adcLeftIS;
    ADC& _adcRightIS;
    DigitalOut& _pinLeftEnable;
    DigitalOut& _pinRightEnable;
    uint32_t _pinPwmLeft;
    uint32_t _pinPwmRight;

    mcpwm_unit_t _mcpwm_num;
    mcpwm_timer_t _timer_num;
    pcnt_unit_t pcnt_unit;
    pcnt_channel_t pcnt_channel;
    pcnt_config_t pcnt_config;
    esp_err_t _rc;

    float _currentLeft, _currentRight;


    int _min, _max, _zero;
    int _angleTarget;
    int _angleCurrent;
    int _directionTargetLast;
    float _lastDutyCycle;
    uint32_t MAX_PWM=80;

public:
    BTS7960(Connector* conn);
    BTS7960(uint32_t pinLeftIS, uint32_t pinrightIS, uint32_t pinLeftEnable,
            uint32_t pinRightEnable, uint32_t pinLeftPwm,
            uint32_t pinRightPwm);
    ~BTS7960();
    Erc initialize();
//    void loop();
//    void calcTarget(float v);
//    int32_t getAngle();
////    int32_t getAngleCurrent();
//   void setAngleTarget(int32_t target);
//    int32_t getAngleTarget();
    void motorStop();
    void left(float);
    void right(float);
    void stop();
    void setMaxPwm(uint32_t);
    void setOutput(float dutyCycle);
    void setDirection(float dutyCycle);
    float measureCurrentLeft();
    float measureCurrentRight();
    void round(float& f, float resolution) ;
    void setPwmUnit(uint32_t );
    void showReg();

};

#endif // BTS7960_H
