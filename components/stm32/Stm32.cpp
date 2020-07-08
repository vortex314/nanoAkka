/*
 * Stm32.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: lieven
 */

#include "Stm32.h"

Stm32::Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset)
    : Actor(thr),
      _uart(UART::create(2, pinTxd, pinRxd)),
      _reset(DigitalOut::create(pinReset)),
      _boot0(DigitalOut::create(pinBoot0)),
      _timer(thread(), 1, 1000, false),
      _testTimer(thread(), 1, 1000, true) {
  
}

void Stm32::init() {
  _timer.stop();

  _uart.setClock(9600);
  _uart.onRxd(onReceive, this);
  _uart.mode("8N1");
  _uart.init();
  _reset.setMode(DigitalOut::DOUT_PULL_UP);
  _reset.init();
  _reset.write(1);
  _boot0.setMode(DigitalOut::DOUT_PULL_UP);
  _boot0.init();
  _boot0.write(0);
}

void Stm32::reset() {
  _reset.write(0);
  vTaskDelay(2);
  _reset.write(1);
  vTaskDelay(2);
}

void Stm32::wiring() {
  ota.async(thread(), [&](const MqttStream& msg) {
    if (msg.offset == 0) {
      startOta(msg);
    } else if (msg.offset + msg.length == msg.total) {
      stopOta(msg);
    } else {
      writeOta(msg);
    }
  });
  _timer >> [&](const TimerMsg& tm) { dispatch({TO, 0}); };
  rxd.async(thread(), [&](const std::string& data) {
    dispatch({RXD, (void*)&data});
  });
  dispatch({GET_ID, 0});
  _testTimer >> [&](const TimerMsg& tm) { dispatch({GET_ID, 0}); };
}

void Stm32::onReceive(void* ptr) {
  Stm32* me = (Stm32*)ptr;
  std::string bytes;
  while (me->_uart.hasData()) {
    uint8_t b=me->_uart.read();
    INFO("RXD>>0x%X",b);
    bytes.append(1,b);
    vTaskDelay(1);  // assure all data is captured
  }
  me->rxd.on(bytes);
}

void Stm32::startOta(const MqttStream& msg) {
  _boot0.write(0);
  reset();
  writeOta(msg);
}

void Stm32::stopOta(const MqttStream& msg) { writeOta(msg); }
void Stm32::writeOta(const MqttStream& msg) {}
void Stm32::write(std::string& b) {
  for (int i = 0; i < b.length(); i++) _uart.write(b[i]);
};

int Stm32::stm32GetId(const Event& ev) {
  INFO("GetId... %d : %d", ev._type, _subState.lc);
  PT_BEGIN(&_subState);
  resetProg();
  request(10, syncRequest);
  PT_WAIT_UNTIL(&_subState, ev.isOneOf(RXD, TO));
  stopTimer();
  if (ev.isRxd(ackReply)) {
    request(10, getRequest);
    PT_WAIT_UNTIL(&_subState, ev.isOneOf(RXD, TO));
    stopTimer();
    if (ev.isRxd(ackReply))
      message.on("RESET succeeded.");
    else
      message.on("GET failed : timeout .");
  } else {
    message.on("RESET failed : timeout on ACK .");
  }
  PT_END(&_subState);
}

int Stm32::dispatch(const Event& ev) {
  /*  INFO(" dispatch (%d,%X) isOneOf %d ", ev._type, ev._ptr,
         ev.isOneOf(GET_ID, WRITE_MEMORY, READ_MEMORY, ERASE_MEMORY));*/
  PT_BEGIN(&_mainState);
  while (true) {
    PT_WAIT_UNTIL(&_mainState, ev.isCommand());
    if (ev == GET_ID) PT_SPAWN(&_mainState, &_subState, stm32GetId(ev));
  }
  PT_END(&_mainState);
}

void Stm32::request(int timeout, std::string& data) {
  write(data);
  _timer.start(timeout);
}

void Stm32::resetProg() {
  _boot0.write(1);
  reset();
}

void Stm32::resetRun() {
  _boot0.write(0);
  reset();
}

void Stm32::stopTimer() { _timer.stop(); }