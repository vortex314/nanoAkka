
#include <Hardware.h>
#include <LedBlinker.h>
#include "freertos/task.h"
#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)

// ___________________________________________________________________________
//
//____________________________________________________________________________
//
#define PRO_CPU 0
#define APP_CPU 1

//______________________________________________________________________
//
class Pinger : public Actor
{
    int _counter=0;
public:
    ValueSource<int> out;
    Sink<int,4> in;
    Pinger(Thread& thr) : Actor(thr)
    {
        in.async(thread(),[&](const int& i) {
            out=_counter++;
        });
    }
    void start()
    {
        out=_counter++;
    }
};
#define DELTA 50000
class Echo : public Actor
{
    uint64_t _startTime;
public:
    ValueSource<int> msgPerMsec=0;
    ValueSource<int> out;
    Sink<int,4> in;
    Echo(Thread& thr) : Actor(thr)
    {
        in.async(thread(),[&](const int& i) {
            if ( i %DELTA == 0 ) {
                uint64_t endTime=Sys::millis();
                uint32_t delta = endTime - _startTime;
                msgPerMsec = DELTA / delta;
                INFO(" handled %lu messages in %u msec = %d msg/msec ",DELTA,delta,msgPerMsec());
                _startTime=Sys::millis();
            }
            out=i+1;
        });
    }
};




class Poller : public Actor,public Sink<TimerMsg,2>
{
    TimerSource _pollInterval;
    std::vector<Requestable*> _publishers;
    uint32_t _idx=0;
    bool _connected;
public:
    Sink<bool,2> connected;
    Poller(Thread& t) : Actor(t),_pollInterval(t,1,100,true)
    {
        _pollInterval >> this;
        connected.async(thread(),[&](const bool& b) {
            _connected=b;
        });
        async(thread(),[&](const TimerMsg tm) {
            if( _publishers.size() && _connected ) _publishers[_idx++ % _publishers.size()]->request();
        });
    };
    void setInterval(uint32_t t)
    {
        _pollInterval.interval(t);
    }
    Poller& operator()(Requestable& rq )
    {
        _publishers.push_back(&rq);
        return *this;
    }
};

Log logger(1024);
// ---------------------------------------------- THREAD
Thread thisThread("main");
Thread ledThread("led");
Thread  mqttThread("mqtt");
Thread  workerThread("worker");

//  --------------------------------------------- ACTOR
#define PIN_LED 2

LedBlinker led(ledThread,PIN_LED, 301);
Pinger pinger(workerThread);
Echo echo(workerThread);

#ifdef MQTT_SERIAL
#include <MqttSerial.h>
MqttSerial mqtt(mqttThread);
#else
#include <Wifi.h>
#include <MqttWifi.h>
Wifi wifi(mqttThread);
MqttWifi mqtt(mqttThread);
#endif


#ifdef US
#include <UltraSonic.h>
Connector uextUs(US);
UltraSonic ultrasonic(thisThread,&uextUs);
#endif

#ifdef GPS
#include <Neo6m.h>
Connector uextGps(GPS);
Neo6m gps(thisThread,&uextGps);
#endif

#ifdef DWM1000_TAG
#include <DWM1000_Tag.h>
#endif

#ifdef REMOTE
#include <Remote.h>
Remote remote(thisThread);
#endif

#ifdef MOTOR
#include <RotaryEncoder.h>
#include <Motor.h>
Connector uextMotor(MOTOR);
#endif

#ifdef SERVO
#include <Servo.h>
Connector uextServo(SERVO);
#endif

#include <ConfigFlow.h>
ConfigFlow<std::string> configHost("system/host","unknown");
ConfigFlow<float> configFloat("system/float",3.141592653);

// ---------------------------------------------- system properties
ValueSource<std::string> systemBuild("NOT SET");
ValueSource<std::string> systemHostname("NOT SET");
ValueSource<bool> systemAlive=true;
LambdaSource<uint32_t> systemHeap([]()
{
    return xPortGetFreeHeapSize();
});
LambdaSource<uint64_t> systemUptime([]()
{
    return Sys::millis();
});
Poller poller(mqttThread);

ArrayQueue<int,16> q;

#ifdef GPIO_TEST
#include <HardwareTester.h>
HardwareTester hw;
#endif

#ifdef STEPPER
#include <Stepper.h>
Connector uextStepper(STEPPER);
Stepper stepper(workerThread,uextStepper);
#endif

#ifdef COMPASS
#include <Compass.h>
Connector uextCompass(COMPASS);
Compass compass(workerThread,uextCompass);
#endif


extern "C" void app_main(void)
{
    //    ESP_ERROR_CHECK(nvs_flash_erase());

#ifdef HOSTNAME
    Sys::hostname(S(HOSTNAME));
#else
    std::string hn;
    union {
        uint8_t macBytes[6];
        uint64_t macInt;
    };
    macInt=0L;
    if ( esp_read_mac(macBytes,ESP_MAC_WIFI_STA) != ESP_OK) WARN(" esp_base_mac_addr_get() failed.");;
    string_format(hn, "ESP32-%d", macInt & 0xFFFF);
    Sys::hostname(hn.c_str());
#endif
    systemHostname = Sys::hostname();;
    systemBuild = __DATE__ " " __TIME__;
    INFO("%s : %s ",Sys::hostname(),systemBuild().c_str());
    for( int cnt=0; cnt<5; cnt++) {
        uint32_t max=100000;
        int x;
        uint64_t start=Sys::millis();
        for(int i=0; i<max; i++) {
            x=i;
            if ( q.push(x) ) ERROR("write failed");
            if ( q.pop(x) ) ERROR("read failed");
            if ( x!=i ) ERROR(" x!=i ");
        }
        uint64_t end = Sys::millis();
        uint32_t delta = end-start;
        uint32_t mpms = max / delta;
        INFO(" time taken for %u iterations : %u msec  = %u msg/msec",max,delta,mpms);
    }
    led.init();
#ifdef MQTT_SERIAL
    mqtt.init();
#else
    wifi.init();
    mqtt.init();
    wifi.connected >> mqtt.wifiConnected;
//-----------------------------------------------------------------  WIFI props
    wifi.macAddress >> mqtt.toTopic<std::string>("wifi/mac");
    wifi.ipAddress >> mqtt.toTopic<std::string>("wifi/ip");
    wifi.ssid >> mqtt.toTopic<std::string>("wifi/ssid");
    wifi.rssi >> mqtt.toTopic<int>("wifi/rssi");
    poller(wifi.macAddress)(wifi.ipAddress)(wifi.ssid)(wifi.rssi);
#endif
    mqtt.connected >> led.blinkSlow;
    mqtt.connected >> poller.connected;
//-----------------------------------------------------------------  SYS props
    configHost == mqtt.topic<std::string>("system/host");
    configFloat == mqtt.topic<float>("system/float");
    systemUptime >> mqtt.toTopic<uint64_t>("system/upTime");
    systemHeap >> mqtt.toTopic<uint32_t>("system/heap");
    systemHostname >> mqtt.toTopic<std::string>("system/hostname");
    systemBuild >> mqtt.toTopic<std::string>("system/build");
    systemAlive >> mqtt.toTopic<bool>("system/alive");
    poller(systemUptime)(systemHeap)(systemHostname)(systemBuild)(systemAlive)(configHost)(configFloat);

    Sink<int,3> intSink([](int i) {
        INFO("received an int %d",i);
    });
    mqtt.fromTopic<int>("os/int") >> intSink;

    TimerSource logTimer(thisThread,1,20000,true) ;
    logTimer >> ([](const TimerMsg& tm) {
        INFO(" ovfl : %u busyPop : %u busyPush : %u threadQovfl : %u  Cas : %u / %u , retries : %u",
             stats.bufferOverflow,
             stats.bufferPopBusy,
             stats.bufferPushBusy,
             stats.threadQueueOverflow,
             stats.bufferPushCasFailed,
             stats.bufferPopCasFailed,
             stats.bufferCasRetries
            );
    });

#ifdef GPIO_TEST
    hw.gpioTest();
    hw.pwmFrequency=10000;
    hw.captureNumberOfPulse=249;
    hw.mcpwmTest();
    hw.captureTest();

    TimerSource pulser(thisThread,1,10,true);
    pulser >> ([](const TimerMsg& tm) {
        static int i=0;
        int pwm = i%200;
        if ( pwm > 100 ) pwm = 200-i;
        hw.pwm(50);
        if (i++==200) i=0;
    });

    TimerSource regTimer(thisThread,1,1000,true);
    LambdaFlow<TimerMsg,MqttMessage> regFlow;
    regFlow.lambda([](MqttMessage& mq,const TimerMsg& tm ) {
        static int cnt=0;
        cnt++;
        if (hw.regs[cnt].name==0 ) cnt=0;
        mq.topic = hw.regs[cnt].name;
        Register reg(mq.topic.c_str(),hw.regs[cnt].format);
        reg.value(*hw.regs[cnt].address);
        reg.format(mq.message);
        return E_OK;
    });
    regTimer >> regFlow ;
    regFlow >> mqtt.outgoing;

    Sink<uint32_t,20> sinker;
    sinker.async(thisThread,[](const uint32_t& cpt) {
        INFO("count : %u , freq : %f Hz",cpt,(160000000.0/cpt)*(hw.captureNumberOfPulse+1));
    });
    hw.capts >> sinker;
#endif

#ifdef STEPPER
    stepper.init();
    mqtt.fromTopic<int>("stepper/steps") >> stepper.steps;
#endif

#ifdef COMPASS
    compass.init();
    compass.x >> mqtt.toTopic<int32_t>("compass/x");
    compass.y >> mqtt.toTopic<int32_t>("compass/y");
    compass.z >> mqtt.toTopic<int32_t>("compass/z");
    compass.status >> mqtt.toTopic<int32_t>("compass/status");
#endif

#ifdef US
    ultrasonic.init();
    ultrasonic.distance >> mqtt.toTopic<int32_t>("us/distance");
#endif

#ifdef GPS
    gps.init(); // no thread , driven from interrupt
    gps >>  mqtt.outgoing;
#endif

#ifdef REMOTE
    remote.init();
    mqtt.fromTopic<bool>("remote/ledLeft") >> remote.ledLeft;             // timer driven
    mqtt.fromTopic<bool>("remote/ledRight") >> remote.ledRight;             // timer driven
    remote.buttonLeft >> mqtt.toTopic<bool>("remote/buttonLeft");
    remote.buttonRight >> mqtt.toTopic<bool>("remote/buttonRight");
    remote.potLeft >> mqtt.toTopic<int>("remote/potLeft");
    remote.potRight >> mqtt.toTopic<int>("remote/potRight");
#endif

#ifdef MOTOR
    RotaryEncoder& rotaryEncoder = *new RotaryEncoder(thisThread,uextMotor.toPin(LP_SCL), uextMotor.toPin(LP_SDA));
    Motor& motor = *new Motor(thisThread,&uextMotor); // cannot init as global var because of NVS
    INFO(" init motor ");
    motor.watchdogTimer.interval(3000);
    mqtt.fromTopic<bool>("motor/watchdogReset") >> motor.watchdogReset;

    rotaryEncoder.init();
    rotaryEncoder.isrCounter >> mqtt.toTopic<uint32_t>("motor/isrCounter");
    poller(rotaryEncoder.isrCounter);

    motor.init();
    rotaryEncoder.rpmMeasured >> motor.rpmMeasured;

    motor.pwm >>  mqtt.toTopic<float>("motor/pwm");
    motor.rpmMeasured2 >>  mqtt.toTopic<int>("motor/rpmMeasured");
    motor.rpmTarget == mqtt.topic<int>("motor/rpmTarget");

    motor.KI >> mqtt.toTopic<float>("motor/KI");
    motor.KP == mqtt.topic<float>("motor/KP");
    motor.KD == mqtt.topic<float>("motor/KD");
    motor.current >> mqtt.toTopic<float>("motor/current");

    motor.deviceState >> mqtt.toTopic<int>("motor/state");
    motor.deviceMessage >> mqtt.toTopic<std::string>("motor/message");
    poller(motor.KI)(motor.KP)(motor.KD);
    poller(motor.deviceMessage)(motor.deviceState)(motor.current);
#endif

#ifdef SERVO
    Servo& servo = *new Servo(thisThread,&uextServo);
    servo.watchdogTimer.interval(3000);
    servo.init();
    mqtt.fromTopic<bool>("servo/watchdogReset") >> servo.watchdogReset;
    servo.pwm >> mqtt.toTopic<float>("servo/pwm");
    servo.adcPot >> mqtt.toTopic<int>("servo/adcPot");
    servo.angleMeasured >>  mqtt.toTopic<int>("servo/angleMeasured");
    servo.KI == mqtt.topic<float>("servo/KI");
    servo.KP == mqtt.topic<float>("servo/KP");
    servo.KD == mqtt.topic<float>("servo/KD");
    servo.current == mqtt.topic<float>("servo/current");
    servo.angleTarget == mqtt.topic<int>("servo/angleTarget");
    servo.deviceState >> mqtt.toTopic<int>("servo/state");
    servo.deviceMessage >> mqtt.toTopic<std::string>("servo/message");
    poller(servo.KI)(servo.KP)(servo.KD)(servo.angleTarget);
    poller(servo.deviceMessage)(servo.deviceState);
#endif

#ifdef DWM1000_TAG
    DWM1000_Tag& tag=*(new DWM1000_Tag(workerThread,new Connector(DWM1000_TAG)));
    tag.preStart();
    tag.mqttMsg >> mqtt.outgoing;
//    tag.blink >> ledBlue.pulse;
    tag.blinks >> mqtt.toTopic<uint32_t>("tag/blinks");
    tag.polls >> mqtt.toTopic<uint32_t>("tag/polls");
    tag.resps >> mqtt.toTopic<uint32_t>("tag/resps");
    tag.finals >> mqtt.toTopic<uint32_t>("tag/finals");
    tag.interruptCount >> mqtt.toTopic<uint32_t>("tag/interrupts");
    tag.errs >> mqtt.toTopic<uint32_t>("tag/errs");
    tag.timeouts >> mqtt.toTopic<uint32_t>("tag/timeouts");
    poller(tag.blinks)(tag.polls)(tag.resps)(tag.finals)(tag.interruptCount)(tag.errs)(tag.timeouts);


#endif

//   pinger.out >> echo.in; // the wiring
//    echo.out >> pinger.in;
//    echo.msgPerMsec >> mqtt.toTopic<int>("system/msgPerMSec");
    echo.out >> mqtt.toTopic<int>("system/echo");
    mqtt.fromTopic<int>("system/echo") >> echo.in;
    /*    mqtt.incoming >> ([](const MqttMessage mm) {
            INFO(" %s = %s",mm.topic.c_str(),mm.message.c_str());
        });*/
    //  pinger.start();
    ledThread.start();
    mqttThread.start();
    workerThread.start();
    thisThread.run(); // DON'T EXIT , local variable will be destroyed

}
