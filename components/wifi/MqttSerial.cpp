#include <MqttSerial.h>



MqttSerial::MqttSerial(Thread& thr) : Mqtt(thr), _uart(UART::create(UART_NUM_0,1,3))
	, connected(false)
	, keepAliveTimer(thr,TIMER_KEEP_ALIVE, 500, true)
	, connectTimer(thr,TIMER_CONNECT, 3000, true) {
	_rxdString.reserve(256);

}
MqttSerial::~MqttSerial() {}

void MqttSerial::init() {
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

	outgoing.async(thread(),[&](const MqttMessage& m) {
		if ( connected()) {
			std::string topic = _hostPrefix;
			topic += m.topic;
			publish(topic,m.message);
		}
	});

	Sink<TimerMsg,3>& me = *this;
	keepAliveTimer >> me;
	connectTimer >> me;
	_uart.setClock(115200);
	_uart.onRxd(onRxd,this);
	_uart.mode("8N1");
	_uart.init();
}

void MqttSerial::on(const TimerMsg& tm) {
	if(tm.id == TIMER_KEEP_ALIVE) {
		publish(_loopbackTopic, std::string("true"));
		outgoing.on({"system/alive", "true"});
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
	}  else {
		WARN("Invalid Timer Id");
	}
}

void MqttSerial::request() {}

void MqttSerial::onRxd(void* me) {
	static Bytes bytes(100);
	MqttSerial* mqttSerial=(MqttSerial*)me;
	while(mqttSerial->_uart.hasData()) {
		bytes.clear();
		mqttSerial->_uart.read(bytes);
		bytes.offset(0);
		while ( bytes.hasData()) mqttSerial->handleSerialByte(bytes.read());
	}
}

void MqttSerial::handleSerialByte(uint8_t b) {
	if ( b=='\r' || b=='\n') {
		if ( _rxdString.length()>0)  {
			DEBUG(" RXD : %s ",_rxdString.c_str());
			rxdSerial(_rxdString);
		}
		_rxdString.clear();
	} else {
		_rxdString+=(char)b;
	}
}

void MqttSerial::rxdSerial(std::string&  rxdString) {
	deserializeJson(rxd, rxdString);
	JsonArray array = rxd.as<JsonArray>();
	if(!array.isNull()) {
		if(array[1].as<std::string>() == _loopbackTopic) {
			_loopbackReceived = Sys::millis();
			connected = true;
		} else {
			std::string topic = array[1];
			incoming.on({topic.substr(_hostPrefix.length()), array[2]});
		}
	} else {
		WARN(" parsing JSON array failed ");
	}
}

void MqttSerial::publish(std::string& topic, std::string message) {
	txd.clear();
	txd.add((int)CMD_PUBLISH);
	txd.add(topic);
	txd.add(message);
	txdSerial(txd);
}

void MqttSerial::subscribe(std::string& topic) {
	txd.clear();
	txd.add((int)CMD_SUBSCRIBE);
	txd.add(topic);
	txdSerial(txd);
}

void MqttSerial::txdSerial(JsonDocument& txd) {
	std::string output = "";
	serializeJson(txd, output);
	printf("%s\n",output.c_str());
}
