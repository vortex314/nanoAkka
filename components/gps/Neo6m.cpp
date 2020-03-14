#include "Neo6m.h"

Neo6m::Neo6m(Thread& thr,Connector* connector)
	: Actor(thr),_connector(connector),_uart(connector->getUART()) {
}

Neo6m::~Neo6m() {
}

std::string stringify(std::string in) {
	std::string out="\"";
	out+=in;
	out+="\"";
	return out;
}


void Neo6m::init() {
	_uart.setClock(9600);
	_uart.onRxd(onRxd,this);
	_uart.init();
}

void Neo6m::request() {
	WARN(" data will be send async");
}


void Neo6m::onRxd(void* me) {
	((Neo6m*) me)->handleRxd();
}

void Neo6m::handleRxd() {

	while ( _uart.hasData() ) {
		char ch = _uart.read();
		if ( ch=='\n' || ch=='\r') {
			if ( _line.size()>8 ) { 
				std::string topic="neo6m/";
				topic+=_line.substr(1,5);
				emit({topic,stringify(_line.substr(7))});
			}
			_line.clear();
		} else {
			_line +=ch;
		}
	}
}
