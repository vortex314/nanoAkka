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
  _timer.stop();
  ackReply.write(BL_ACK);
  nackReply.write(BL_NACK);
  getCmd.write((uint8_t)BL_GET);
  getCmd.write(XOR(BL_GET));
}

void Stm32::init() {
  _uart.setClock(9600);
  _uart.onRxd(onReceive, this);
  _uart.mode("8N1");
  _uart.init();
  _reset.setMode(DigitalOut::DOUT_PULL_UP);
  _reset.init();
  _reset.write(1);
  _boot0.setMode(DigitalOut::DOUT_PULL_UP);
  _boot0.init();
  _boot0.write(1);
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
  rxd.async(thread(), [&](const Bytes& data) {
    INFO(">>RXD");
    dispatch({RXD, (void*)&data});
  });
  dispatch({ANALYZE, 0});
  _testTimer >> [&](const TimerMsg& tm) { dispatch({ANALYZE, 0}); };
}

void Stm32::onReceive(void* ptr) {
  Stm32* me = (Stm32*)ptr;
  Bytes bytes;
  INFO("");
  while (me->_uart.hasData()) {
    bytes.write(me->_uart.read());
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
void Stm32::write(int size, uint8_t* b) {
  for (int i = 0; i < size; i++) _uart.write(b[i]);
};
void Stm32::write(Bytes& b) {
  b.offset(0);
  while (b.hasData()) _uart.write(b.read());
};

bool Stm32::dispatch(const Event& ev) {
  INFO(" dispatch (%d,%X) isOneOf %d ", ev._type, ev._ptr,
       ev.isOneOf(ANALYZE, WRITE, READ, ERASE));
  PT_BEGIN();
  while (true) {
    PT_WAIT_UNTIL(ev.isOneOf(ANALYZE, WRITE, READ, ERASE));
    INFO("");
    if (ev == ANALYZE) {
      message.on("Analyzing...");
      resetProg();
      _uart.write(0x7F);  // send sync for bootloader
      _timer.start(10);
      PT_WAIT_UNTIL(ev.isOneOf(RXD, TO));
      _timer.stop();
      if (ev.isRxd(ackReply)) {
        write(getCmd);
        _timer.start(10);
        PT_WAIT_UNTIL(ev.isOneOf(RXD, TO));
        _timer.stop();
        if (ev.isRxd(ackReply))
          message.on("succeeded.");
        else
          message.on(" TO : erase Cmd failed.");
      } else {
        message.on(" TO : reset failed.");
      }
    }
  }
  PT_END();
}

void Stm32::resetProg(){
  _boot0.write(1);
  reset();
}

void Stm32::resetRun(){
  _boot0.write(0);
  reset();
}