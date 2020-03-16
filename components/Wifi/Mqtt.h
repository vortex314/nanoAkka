#ifndef MQTT_ABSTRACT_H
#define MQTT_ABSTRACT_H
#include <NanoAkka.h>
#include <ArduinoJson.h>
typedef struct MqttMessage {
	std::string topic;
	std::string message;
} MqttMessage;
//____________________________________________________________________________________________________________
//
template <class T>
class ToMqtt : public Flow<T, MqttMessage> {
		std::string _name;
	public:
		ToMqtt(std::string name)
			: _name(name) {};
		int  convert(MqttMessage&  msg,const T& event) {
			std::string s;
			DynamicJsonDocument doc(100);
			JsonVariant variant = doc.to<JsonVariant>();
			variant.set(event);
			serializeJson(doc, s);
			msg = {_name,s};
			return 0;
		}
		void request() {};
};
//_______________________________________________________________________________________________________________
//
template <class T>
class FromMqtt : public Flow<MqttMessage, T> {
		std::string _name;
	public:
		FromMqtt(std::string name)
			: _name(name) {};
		int convert(T& t,const MqttMessage& mqttMessage) {
//			INFO(" '%s' <> '%s'",mqttMessage.topic.c_str(),_name.c_str());
			if(mqttMessage.topic != _name) {
				return EINVAL;
			}
			DynamicJsonDocument doc(100);
			auto error = deserializeJson(doc, mqttMessage.message);
			if(error) {
				WARN(" failed JSON parsing '%s' : '%s' ", mqttMessage.message.c_str(), error.c_str());
				return ENODATA;
			}
			JsonVariant variant = doc.as<JsonVariant>();
			if(variant.isNull()) {
				WARN(" is not a JSON variant '%s' ", mqttMessage.message.c_str());
				return ENODATA;
			}
			if(variant.is<T>() == false) {
				WARN(" message '%s' JSON type doesn't match.", mqttMessage.message.c_str());
				return ENODATA;
			}
			t = variant.as<T>();
			return 0;
			// emit doesn't work as such
			// https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
		}
		void request() {};
};
//____________________________________________________________________________________________________________
//
template <class T>
class MqttFlow : public ValueFlow<T>,public ToMqtt<T>,public FromMqtt<T> {
	public:
		MqttFlow(const char* topic) :  ToMqtt<T>(topic),FromMqtt<T>(topic) {};
};
//____________________________________________________________________________________________________________
//
class Mqtt : public Actor {
	public:
		QueueSource<MqttMessage,5> incoming;
		Sink<MqttMessage,10> outgoing;
		ValueSource<bool> connected;
		TimerSource keepAliveTimer;
		Mqtt(Thread& thr) : Actor(thr) {};
		~Mqtt() {};
		void init();
		template <class T>
		Subscriber<T>& toTopic(const char* name) {
			auto flow = new ToMqtt<T>(name);
			*flow >> outgoing;
			return *flow;
		}
		template <class T>
		Source<T>& fromTopic(const char* name) {
			auto newSource = new FromMqtt<T>(name);
			incoming >> *newSource;
			return *newSource;
		}
		template <class T>
		ValueFlow<T>& topic(const char* name) {
			auto flow = new MqttFlow<T>(name);
			incoming.subscribe(flow);
			Publisher<MqttMessage>& mqpub  =  *flow;
			mqpub.subscribe(((Subscriber<MqttMessage>*)&outgoing));
			return *flow;
		}
};
#endif
