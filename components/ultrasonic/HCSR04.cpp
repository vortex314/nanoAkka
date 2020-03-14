#include <HCSR04.h>

#include <Log.h>
#include <Sys.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

HCSR04::HCSR04(DigitalOut& triggerPin, DigitalIn& echoPin)
	: _trigger(triggerPin), _echo(echoPin) {}

HCSR04::HCSR04(Connector& connector)
	: _trigger(connector.getDigitalOut(LP_SCL)),
	  _echo(connector.getDigitalIn(LP_SDA)) {}

Erc HCSR04::init() {
	_trigger.init();
	_echo.onChange(DigitalIn::DIN_FALL, onEcho, this);
	_echo.init();
	INFO(" HCSR04 init ( trigger = %d, echo = %d ) ", _trigger.getPin(),
	     _echo.getPin());
	return E_OK;
}

Erc HCSR04::trigger() {
	_trigger.write(1);
	_trigger.write(0);
	_trigger_usec = Sys::micros();
	return E_OK;
}

uint64_t HCSR04::getTime() {
	//  INFO(" %lld ", _echo_usec - _trigger_usec);
	return _delta_usec;
}

uint32_t HCSR04::getCentimeters() {
	float ctm = getTime();
	ctm /= 58.0;
	return ctm;
}

void IRAM_ATTR HCSR04::onEcho(void* obj) {
//	INFO(" ------- INTERRUPT ------- ");
	HCSR04* me = (HCSR04*)obj;
	me->_delta_usec = Sys::micros() - me->_trigger_usec;
}
