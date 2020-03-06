#ifndef MQTTSERIAL_H
#define MQTTSERIAL_H

#include <string>
#include <Streams.h>
#include <Hardware.h>
#include "driver/uart.h"

#define QOS 0
#define TIMEOUT 10000L
//____________________________________________________________________________________________________________
//
typedef struct MqttMessage {
    std::string topic;
    std::string message;
} MqttMessage;
//____________________________________________________________________________________________________________
//
template <class T>
class MqttFlow : public Flow<T, T>
{
    std::string _name;

public:
    LambdaSink<MqttMessage> mqttIn;
    ValueFlow<MqttMessage> mqttOut;
    MqttFlow(std::string name)
        : _name(name)
    {
        mqttIn = *new LambdaSink<MqttMessage>([&](const MqttMessage& msg) {
            onNext(msg);
        });
    };

    void onNext(const T& event)
    {
        //       INFO(" topic : %s ",_name.c_str());
        std::string s;
        DynamicJsonDocument doc(100);
        JsonVariant variant = doc.to<JsonVariant>();
        variant.set(event);
        serializeJson(doc, s);
        mqttOut.emit({_name, s});
        // emit doesn't work as such
        // https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
    }

    void onNext(const MqttMessage& mqttMessage)
    {
        if(mqttMessage.topic != _name) return;
        INFO(" MqttFlow topic : %s ",mqttMessage.topic.c_str());

        DynamicJsonDocument doc(100);
        auto error = deserializeJson(doc, mqttMessage.message);
        if(error) {
            WARN(" failed JSON parsing '%s' : '%s' ", mqttMessage.message.c_str(), error.c_str());
            return;
        }
        JsonVariant variant = doc.as<JsonVariant>();
        if(variant.isNull()) {
            WARN(" is not a JSON variant '%s' ", mqttMessage.message.c_str());
            return;
        }
        if(variant.is<T>() == false) {
            WARN(" message '%s' JSON type doesn't match.", mqttMessage.message.c_str());
            return;
        }
        T value = variant.as<T>();
        this->emit(value);
        // emit doesn't work as such
        // https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
    }

    void request() {};

};
//____________________________________________________________________________________________________________
//
template <class T>
class ToMqtt : public Flow<T, MqttMessage>
{
    std::string _name;

public:
    ToMqtt(std::string name)
        : _name(name) {};
    void onNext(const T& event)
    {
        std::string s;
        DynamicJsonDocument doc(100);
        JsonVariant variant = doc.to<JsonVariant>();
        variant.set(event);
        serializeJson(doc, s);
        this->emit({_name, s});
        // emit doesn't work as such
        // https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
    }
    void request() {};
};

//_______________________________________________________________________________________________________________
//
template <class T>
class FromMqtt : public Flow<MqttMessage, T>
{
    std::string _name;

public:
    FromMqtt(std::string name)
        : _name(name) {};

    void onNext(const MqttMessage& mqttMessage)
    {
        INFO("MQTT RXD : %s",mqttMessage.topic.c_str());
        if(mqttMessage.topic != _name) {
            return;
        }
        DynamicJsonDocument doc(100);
        auto error = deserializeJson(doc, mqttMessage.message);
        if(error) {
            WARN(" failed JSON parsing '%s' : '%s' ", mqttMessage.message.c_str(), error.c_str());
            return;
        }
        JsonVariant variant = doc.as<JsonVariant>();
        if(variant.isNull()) {
            WARN(" is not a JSON variant '%s' ", mqttMessage.message.c_str());
            return;
        }
        if(variant.is<T>() == false) {
            WARN(" message '%s' JSON type doesn't match.", mqttMessage.message.c_str());
            return;
        }
        T value = variant.as<T>();
        this->emit(value);
        // emit doesn't work as such
        // https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
    }
    void request() {};
};

class MqttSerial : public Sink<TimerMsg>, public Flow<MqttMessage, MqttMessage>
{

    StaticJsonDocument<3000> _jsonBuffer;
    std::string _clientId;
    std::string _address;
    std::string _lwt_topic;
    std::string _lwt_message;
    UART& _uart;

private:
    StaticJsonDocument<256> txd;
    StaticJsonDocument<256> rxd;
    std::string _rxdString;
    std::string _loopbackTopic;
    uint64_t _loopbackReceived;
    std::string _hostPrefix;

    enum { CMD_SUBSCRIBE = 0, CMD_PUBLISH };

    static void onRxd(void*);
    void handleSerialByte(uint8_t);
    void rxdSerial(std::string& );
    void txdSerial(JsonDocument& );
    void publish(std::string& topic, std::string message);
    void subscribe(std::string& topic);

public:
    AsyncFlow<MqttMessage> outgoing;
    AsyncFlow<MqttMessage> incoming;
    LambdaSink<bool> wifiConnected;
    ValueFlow<bool> connected;
    TimerSource keepAliveTimer;
    TimerSource connectTimer;
    TimerSource serialTimer;
    MqttSerial();
    ~MqttSerial();
    void init();

    void mqttPublish(const char* topic, const char* message);
    void mqttSubscribe(const char* topic);
    void mqttConnect();
    void mqttDisconnect();

    bool handleMqttMessage(const char* message);
    void observeOn(Thread&);

    void onNext(const TimerMsg&);
    void onNext(const MqttMessage&);
    void request();
    template <class T>
    Sink<T>& toTopic(const char* name)
    {
        return *(new ToMqtt<T>(name)) >> outgoing;
    }
    template <class T>
    Source<T>& fromTopic(const char* name)
    {
        auto newSource = new FromMqtt<T>(name);
        incoming >> *newSource;
        return *newSource;
    }

    template <class T>
    MqttFlow<T>& topic(const char* name)
    {
        auto newFlow = new MqttFlow<T>(name);
        incoming >> newFlow->mqttIn;
        newFlow->mqttOut >> outgoing;
        return *newFlow;
    }
};


#endif // MQTTSERIAL_H
