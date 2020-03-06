#include <MqttSerial.h>

#define TIMER_KEEP_ALIVE 1
#define TIMER_CONNECT 2
#define TIMER_SERIAL 3

MqttSerial::MqttSerial() :_uart(UART::create(UART_NUM_0,1,3))
    , connected(false)
    , incoming(20)
    , outgoing(20)
    , keepAliveTimer(TIMER_KEEP_ALIVE, 1000, true)
    , connectTimer(TIMER_CONNECT, 3000, true)
    , serialTimer(TIMER_SERIAL, 10, true)
{
    _rxdString.reserve(256);
}
MqttSerial::~MqttSerial() {}

void MqttSerial::init()
{
    INFO("MqttSerial started. ");
    txd.clear();
    rxd.clear();
    _hostPrefix = "src/";
    _hostPrefix += Sys::hostname();
    _hostPrefix += "/";
    _loopbackTopic += "dst/";
    _loopbackTopic+= Sys::hostname();
    _loopbackTopic += "/system/loopback";
    _loopbackReceived = 0;
    outgoing >> *this;
    *this >> incoming;
    Sink<TimerMsg>& me = *this;
    keepAliveTimer >> me;
    connectTimer >> me;
    serialTimer >> me;
    connected.emitOnChange(true);
    _uart.setClock(115200);
    _uart.onRxd(onRxd,this);
    _uart.mode("8N1");
    _uart.init();
}

void MqttSerial::onNext(const TimerMsg& tm)
{
    // LOG(" timer : %lu ",tm.id);
    if(tm.id == TIMER_KEEP_ALIVE) {
        publish(_loopbackTopic, std::string("true"));
        outgoing.onNext({"system/alive", "true"});
    } else if(tm.id == TIMER_CONNECT) {
        if(Sys::millis() > (_loopbackReceived + 2000)) {
            connected = false;
            std::string topic;
            string_format(topic, "dst/%s/#", Sys::hostname());
            subscribe(topic);
            publish(_loopbackTopic, "true");
        } else {
            connected = true;
        }
    } else if(tm.id == TIMER_SERIAL) {
        /*	//   LOG("TIMER_SERIAL");
                if(_stream.available()) {
                    String s = _stream.readString();
                    rxdSerial(s);
                };*/
    } else {
        WARN("Invalid Timer Id");
    }
}

void MqttSerial::onNext(const MqttMessage& m)
{
    if(connected()) {
        std::string topic = _hostPrefix+m.topic;
        publish(topic, m.message);
    };
}

void MqttSerial::observeOn(Thread& thread)
{
    keepAliveTimer.observeOn(thread);
    connectTimer.observeOn(thread);
    serialTimer.observeOn(thread);
    outgoing.observeOn(thread);
    incoming.observeOn(thread);
}

void MqttSerial::request()
{

}

void MqttSerial::onRxd(void* me)
{
    static Bytes bytes(100);
    MqttSerial* mqttSerial=(MqttSerial*)me;
    while(mqttSerial->_uart.hasData()) {
        bytes.clear();
        mqttSerial->_uart.read(bytes);
        bytes.offset(0);
        while ( bytes.hasData()) mqttSerial->handleSerialByte(bytes.read());
    }
}

void MqttSerial::handleSerialByte(uint8_t b)
{
    if ( b=='\r' || b=='\n') {
        if ( _rxdString.length()>0)  {
            INFO(" RXD : %s ",_rxdString.c_str());
            rxdSerial(_rxdString);
        }
        _rxdString.clear();
    } else {
        _rxdString+=(char)b;
    }
}

void MqttSerial::rxdSerial(std::string&  rxdString)
{
    deserializeJson(rxd, rxdString);
    JsonArray array = rxd.as<JsonArray>();
    if(!array.isNull()) {
        if(array[1].as<std::string>() == _loopbackTopic) {
            _loopbackReceived = Sys::millis();
        } else {
            std::string topic = array[1];
            emit({topic.substr(_hostPrefix.length()), array[2]});
        }
    } else {
        WARN(" parsing JSON array failed ");
    }
}

void MqttSerial::publish(std::string& topic, std::string message)
{
    txd.clear();
    txd.add((int)CMD_PUBLISH);
    txd.add(topic);
    txd.add(message);
    txdSerial(txd);
}

void MqttSerial::subscribe(std::string& topic)
{
    txd.clear();
    txd.add((int)CMD_SUBSCRIBE);
    txd.add(topic);
    txdSerial(txd);
}

void MqttSerial::txdSerial(JsonDocument& txd)
{
    std::string output = "";
    serializeJson(txd, output);
    printf("%s\n",output.c_str());
}
