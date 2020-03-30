#include "HardwareTester.h"

#define PWM 0

Reg HardwareTester::regs[] = {
    {
        "Configuration of the prescaler",
        "PWM_CLK_CFG_REG",
        "- - - - - - - - - - - - - - - - - - - - - - - - + + + + + + + PWM_CLK_PRESCALE",
        (uint32_t*)MCPWM_CLK_CFG_REG(PWM)
    },
    {
        "Timer period and update method",
        "PWM_TIMER0_CFG0_REG",
        "- - - - - - + PWM_TIMER0_PERIOD_UPMETHOD + + + + + + + + + + + + + + + PWM_TIMER0_PERIOD + + + + + + + PWM_TIMER0_PRESCALE",
        (uint32_t*)MCPWM_TIMER0_CFG0_REG(PWM)
    },
    {
        "Working mode and start/stop control",
        "PWM_TIMER0_CFG1_REG",
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - + PWM_TIMER0_MOD + + PWM_TIMER0_START",
        (uint32_t*)MCPWM_TIMER0_CFG1_REG(PWM)
    },
    {
        "Synchronization settings",
        "PWM_TIMER0_SYNC_REG",
        "- - - - - - - - - - - + + + + + + + + + + + + + + + + PWM_TIMER0_PHASE + PWM_TIMER1_SYNCO_SEL PWM_TIMER1_SYNC_SW PWM_TIMER1_SYNCI_EN",
        (uint32_t*)MCPWM_TIMER0_SYNC_REG(PWM)
    },
    {
        "Timer status",
        "PWM_TIMER0_STATUS_REG",
        "- - - - - - - - - - - - - - - PWM_TIMER0_DIRECTION + + + + + + + + + + + + + + + PWM_TIMER0_VALUE",
        (uint32_t*)MCPWM_TIMER0_STATUS_REG(PWM)
    },
    {
        "Transfer status and update method for time stamp registers A and B",
        "PWM_GEN0_STMP_CFG_REG",
        "- - - - - - - - - - - - - - - - - - - - - - PWM_GEN0_B_SHDW_FULL PWM_GEN0_A_SHDW_FULL + + + PWM_GEN0_B_UPMETHOD + + + PWM_GEN0_A_UPMETHOD",
        (uint32_t*)MCPWM_GEN0_STMP_CFG_REG(PWM)
    },
    {
        "Configure capture timer",
        "PWM_CAP_TIMER_CFG_REG",
        "- - - - - - - - - - - - - - - - - - - - - - - - - - PWM_CAP_SYNC_SW + + PWM_CAP_SYNCI_SEL PWM_CAP_SYNCI_EN PWM_CAP_TIMER_EN",
        (uint32_t*)MCPWM_CAP_TIMER_CFG_REG(PWM)
    },
    {
        "Phase for capture timer sync",
        "PWM_CAP_TIMER_PHASE_REG",
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ",
        (uint32_t*)MCPWM_CAP_TIMER_PHASE_REG(PWM)
    },
    {
        "Capture channel 0 configuration and enable",
        "PWM_CAP_CH0_CFG_REG",
        "- - - - - - - - - - - - - - - - - - - PWM_CAP0_SW PWM_CAP0_IN_INVERT + + + + + + + PWM_CAP0_PRESCALE + PWM_CAP0_MODE PWM_CAP0_EN",
        (uint32_t*)MCPWM_CAP_CH0_CFG_REG(PWM)
    },
    {NULL,NULL,NULL,NULL}
    //   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
};
Connector uext1(1),uext2(2);

HardwareTester::HardwareTester() : _uext(uext1)
{
}

HardwareTester::~HardwareTester()
{
}


#define CAP0_INT_EN BIT(27) // Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28) // Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29) // Capture 2 interrupt bit

static mcpwm_dev_t* MCPWM[2] = {&MCPWM0, &MCPWM1};
static uint32_t prevCapt;


void IRAM_ATTR HardwareTester::isrCaptureHandler(void* pv)
{
    HardwareTester* hw = (HardwareTester*)pv;
    uint32_t mcpwm_intr_status;

    mcpwm_intr_status = MCPWM[hw->mcpwm_num]->int_st.val; // Read interrupt (st)

    // Check for interrupt on rising edge on CAP0 signal
    if(mcpwm_intr_status & CAP0_INT_EN) {
       uint32_t capt = mcpwm_capture_signal_get_value(hw->mcpwm_num,MCPWM_SELECT_CAP0);
        hw->capts=capt-prevCapt;
        prevCapt=capt;
    }
    MCPWM[hw->mcpwm_num]->int_clr.val = mcpwm_intr_status; // Clear interrupt (clr)
}



int HardwareTester::captureTest()
{
    int pinTacho = _uext.toPin(LP_CS);
    uint64_t 	apbClock = rtc_clk_apb_freq_get();
    INFO(" APB clock : %lu Hz",apbClock);
    INFO(" number of pulses : %d ", captureNumberOfPulse );
    rc = mcpwm_gpio_init(mcpwm_num, MCPWM_CAP_0, pinTacho);
    if(rc != ESP_OK) {
        WARN("mcpwm_gpio_init()=%d", rc);
    }

    rc = mcpwm_capture_enable(mcpwm_num, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, captureNumberOfPulse);
    if(rc != ESP_OK) {
        WARN("mcpwm_capture_enable()=%d", rc);
    }
    MCPWM[mcpwm_num]->int_ena.val = CAP0_INT_EN;
    // Set ISR Handler
    rc = mcpwm_isr_register(mcpwm_num, isrCaptureHandler, this, ESP_INTR_FLAG_IRAM, NULL);
    if(rc) {
        WARN("mcpwm_capture_enable()=%d", rc);
    }
    return E_OK;
}

int HardwareTester::mcpwmTest()
{
    int pinPwm1=_uext.toPin(LP_TXD) ;// 19; // was 18
    int pinPwm2=_uext.toPin(LP_SCK); // 17; // was 16
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
    pwm_config.frequency = pwmFrequency; // frequency = 500Hz,
    pwm_config.cmpr_a = 50;        // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 70;        // duty cycle of PWMxb = 0
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
    INFO(" GPIO tester ======== UEXT =============>");
    fromToGpio(_uext.toPin(LP_TXD),_uext.toPin(LP_RXD));
    fromToGpio(_uext.toPin(LP_SCL),_uext.toPin(LP_SDA));
    fromToGpio(_uext.toPin(LP_SDA),_uext.toPin(LP_SCL));
    fromToGpio(_uext.toPin(LP_MOSI),_uext.toPin(LP_MISO));
    fromToGpio(_uext.toPin(LP_CS),_uext.toPin(LP_SCK));
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
