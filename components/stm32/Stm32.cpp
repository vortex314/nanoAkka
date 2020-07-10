/*
 * Stm32.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: lieven
 */
#include "Stm32.h"

std::string string_to_hex(const std::string& input) {
  static const char hex_digits[] = "0123456789ABCDEF";

  std::string output;
  output.reserve(input.length() * 2);
  for (unsigned char c : input) {
    output.push_back(hex_digits[c >> 4]);
    output.push_back(hex_digits[c & 15]);
    output.push_back(' ');
  }
  return output;
}

const char* strEvents[] = {"NOP",          "RXD",          "TO",
                           "ERASE_MEMORY", "WRITE_MEMORY", "READ_MEMORY",
                           "GET_ID",       "GET_REQUEST",  "GET_VERSION"};


  bool isOneOf(int v1, int v2 = NOP, int v3 = NOP, int v4 = NOP) const {
    return _type == v1 || _type == v2 || _type == v3 || _type == v4;
  };
  bool is(int v) const { return _type == v; };
  bool isCommand() const { return _type >= ERASE_MEMORY; };
  bool isRxd(std::string& bytes) const {
    return _type == RXD && bytes.compare(*data) == 0;
  };

  bool isCommand(int command) const { return _type == command; };

  void operator=(int v) { _type = v; }
  Event(int tpe, void* ptr = 0) {
    _type = tpe;
    _ptr = ptr;
  }
  bool operator==(int v) const { return _type == v; }
};

Stm32::Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset)
    : Actor(thr),
      _uart(UART::create(2, pinTxd, pinRxd)),
      _reset(DigitalOut::create(pinReset)),
      _boot0(DigitalOut::create(pinBoot0)),
      _timer(thread(), 1, 1000, false),
      _testTimer(thread(), 1, 1000, true) {}

void Stm32::init() {
  _timer.stop();

  _uart.setClock(115200);
  _uart.onRxd(onReceive, this);
  _uart.mode("8E1");
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
  vTaskDelay(1);
  _reset.write(1);
  vTaskDelay(1);
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
    INFO("RXD >>>> %s", string_to_hex(data).c_str());
    dispatch({RXD, (void*)&data});
  });
  _testTimer >> [&](const TimerMsg& tm) { dispatch({GET_VERSION, 0}); };
}

void Stm32::onReceive(void* ptr) {
  Stm32* me = (Stm32*)ptr;
  std::string bytes;
  while (me->_uart.hasData()) {
    uint8_t b = me->_uart.read();
    bytes.append(1, b);
    //    vTaskDelay(1);  // assure all data is captured
  }
  me->rxd.on(bytes);
}

void Stm32::startOta(const MqttStream& msg) {
  _boot0.write(0);
  reset();
  writeOta(msg);
}

void Stm32::dump(std::string& s) {
  INFO(" RXD : %s", string_to_hex(s).c_str());
}
void Stm32::stopOta(const MqttStream& msg) { writeOta(msg); }
void Stm32::writeOta(const MqttStream& msg) {}
void Stm32::write(std::string& b) {
  for (int i = 0; i < b.length(); i++) _uart.write(b[i]);
};

int Stm32::stm32Request(const Event& ev,std::string& req,std::string& response) {
  INFO("Get... [%s] line : %d", strEvents[ev._type], _subState.lc);
  PT_BEGIN(&_subState);
  resetToProg();
  request(50, syncRequest);
  PT_YIELD_UNTIL(&_subState, ev.isOneOf(RXD, TO));
  stopTimer();
  if (ev.isRxd(ackReply)) {
    INFO(" RXD : %s", string_to_hex(*ev.data).c_str());
    request(50, req);
    PT_YIELD_UNTIL(&_subState, ev.isOneOf(RXD, TO));
    stopTimer();
    if (ev.is(RXD)) {
      INFO(" RXD : %s", string_to_hex(*ev.data).c_str());
      if (ev.data->length() > 3) {
        response = * ev.data;
        message.on("Request succeeded.");
      } else {
        WARN(" Request response too short %d", ev.data->length());
        message.on(" GET failed : too short answer");
      }
    } else
      message.on("Request failed : timeout .");
  } else {
    message.on("RESET failed : timeout on ACK .");
  }
  PT_END(&_subState);
}

int Stm32::dispatch(const Event& ev) {
  std::string response;
  INFO(" dispatch [%s] line %d ", strEvents[ev._type], _mainState.lc);
  PT_BEGIN(&_mainState);
  while (true) {
    PT_YIELD_UNTIL(&_mainState, ev.isCommand());
    if (ev == GET_ID) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(ev,getIdRequest),response);
    }
    if (ev == GET_REQUEST) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(ev,getRequest),response);
    }
    if (ev == GET_VERSION) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(ev,getVersionRequest),response);
    }
  }
  PT_END(&_mainState);
}

void Stm32::request(int timeout, std::string& data) {
  write(data);
  _timer.start(timeout);
}

void Stm32::resetToProg() {
  _boot0.write(1);
  reset();
}

void Stm32::resetToRun() {
  _boot0.write(0);
  reset();
}

void Stm32::stopTimer() { _timer.stop(); }