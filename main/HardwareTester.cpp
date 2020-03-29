#include "HardwareTester.h"

HardwareTester::HardwareTester()
{
}

HardwareTester::~HardwareTester()
{
}


#define CAPTURE_DIVIDER 10
#define CAP0_INT_EN BIT(27) // Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28) // Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29) // Capture 2 interrupt bit

static mcpwm_dev_t* MCPWM[2] = {&MCPWM0, &MCPWM1};

void IRAM_ATTR HardwareTester::isrPwmHandler(void* pv)
{
    HardwareTester* hw = (HardwareTester*)pv;
    uint32_t mcpwm_intr_status;
    static uint32_t prevCapt;

    mcpwm_intr_status = MCPWM[hw->mcpwm_num]->int_st.val; // Read interrupt

    // Check for interrupt on rising edge on CAP0 signal
    if(mcpwm_intr_status & CAP0_INT_EN) {
        uint32_t capt = mcpwm_capture_signal_get_value(hw->mcpwm_num,MCPWM_SELECT_CAP0);
        hw->capts=capt-prevCapt;
        prevCapt=capt;
    }
    MCPWM[hw->mcpwm_num]->int_clr.val = mcpwm_intr_status;
}



int HardwareTester::captureTest(int pinTacho)
{
    rc = mcpwm_gpio_init(mcpwm_num, MCPWM_CAP_0, pinTacho);
    if(rc != ESP_OK) {
        WARN("mcpwm_gpio_init()=%d", rc);
    }

    rc = mcpwm_capture_enable(mcpwm_num, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, CAPTURE_DIVIDER);
    if(rc != ESP_OK) {
        WARN("mcpwm_capture_enable()=%d", rc);
    }
    MCPWM[mcpwm_num]->int_ena.val = CAP0_INT_EN;
    // Set ISR Handler
    rc = mcpwm_isr_register(mcpwm_num, isrPwmHandler, this, ESP_INTR_FLAG_IRAM, NULL);
    if(rc) {
        WARN("mcpwm_capture_enable()=%d", rc);
    }
    return E_OK;
}

int HardwareTester::mcpwmTest()
{
    int pinPwm1=19; // was 18
    int pinPwm2=17; // was 16
    float duty_cycle = 50;
    mcpwm_num = MCPWM_UNIT_0; // MCPWM_UNIT_1
    timer_num = MCPWM_TIMER_0;

    rc = mcpwm_gpio_init(mcpwm_num, MCPWM0A, pinPwm1);
    if ( rc != ESP_OK ) {
        WARN("mcpwm_gpio_init()=%d",rc);
        return EIO;
    }
    rc=mcpwm_gpio_init(mcpwm_num, MCPWM0B, pinPwm2);
    if ( rc != ESP_OK ) {
        WARN("mcpwm_gpio_init()=%d",rc);
        return EIO;
    };
    mcpwm_config_t pwm_config;
    BZERO(pwm_config);
    pwm_config.frequency = 1000; // frequency = 1000Hz,
    pwm_config.cmpr_a = 50;        // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 50;        // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    rc= mcpwm_init(mcpwm_num, timer_num,&pwm_config);
    if(rc != ESP_OK) {
        WARN("mcpwm_init()=%d",rc);
        return EIO;
    }
    /*   rc = mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
       if ( rc != E_OK ) {
           WARN("mcpwm_set_signal_low()=%d",rc);
       }*/
    rc = mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    if ( rc != E_OK ) {
        WARN("mcpwm_set_duty_type()=%d",rc);
    }
    rc = mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    if ( rc != E_OK ) {
        WARN("mcpwm_set_duty_type()=%d",rc);
    }
    return E_OK;
}

int HardwareTester::pwm(int dutyCycle)
{
    rc = mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, dutyCycle);
    if ( rc != E_OK ) {
        WARN("mcpwm_set_duty_type()=%d",rc);
    }
    rc = mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, dutyCycle);
    if ( rc != E_OK ) {
        WARN("mcpwm_set_duty_type()=%d",rc);
    }
    return E_OK;
}

void HardwareTester::gpioTest()
{
    INFO(" GPIO tester ======== UEXT1 =============>");
    fromToGpio(19,36);
    fromToGpio(25,26);
    fromToGpio(26,25);
    fromToGpio(23,34);
    fromToGpio(17,32);
    fromToGpio(32,17);
    INFO(" GPIO tester ======== UEXT2 =============>");
    fromToGpio(18,39);
    fromToGpio(27,14);
    fromToGpio(14,27);
    fromToGpio(22,35);
    fromToGpio(16,33);
    fromToGpio(33,16);
    INFO(" GPIO tester <=====================");
}

bool HardwareTester::fromToGpio(int pin1,int pin2)
{
    bool success=true;
    DigitalOut& gpioOut=DigitalOut::create(pin1);
    DigitalIn& gpioIn=DigitalIn::create(pin2);
    gpioOut.init();
    gpioIn.init();
    gpioOut.write(0);
    if ( gpioIn.read() != 0) {
        WARN(" ======> gpio %d => %d for 0 failed.",pin1,pin2);
        success=false;
    }
    gpioOut.write(1);
    if ( gpioIn.read() != 1) {
        WARN(" ======> gpio %d => %d for 1 failed.",pin1,pin2);
        success=false;
    }
    gpioIn.deInit();
    gpioOut.deInit();
    return success;
}
