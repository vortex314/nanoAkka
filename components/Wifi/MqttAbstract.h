#ifndef MQTT_ABSTRACT_H
#define MQTT_ABSTRACT_H

typedef struct MqttMessage {
	std::string topic;
	std::string message;
} MqttMessage;
//____________________________________________________________________________________________________________
//

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

class Mqtt : public Actor {

		StaticJsonDocument<3000> _jsonBuffer;
		std::string _clientId;
		std::string _address;
		esp_mqtt_client_handle_t _mqttClient;
		std::string _lwt_topic;
		std::string _lwt_message;
		std::string _hostPrefix;

		TimerSource _reportTimer;
		TimerSource _keepAliveTimer;

	public:
		QueueSource<MqttMessage,5> incoming;
		Sink<MqttMessage,10> outgoing;

		Sink<bool,2> wifiConnected;
		ValueSource<bool> connected;
		TimerSource keepAliveTimer;
		Mqtt(Thread& thread);
		~Mqtt();
		void init();

		void mqttPublish(const char* topic, const char* message);
		void mqttSubscribe(const char* topic);
		void mqttConnect();
		void mqttDisconnect();

		bool handleMqttMessage(const char* message);
		static int mqtt_event_handler(esp_mqtt_event_t* event);

		void onNext(const TimerMsg&);
		void onNext(const MqttMessage&);
		void request();
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
};

#endif
