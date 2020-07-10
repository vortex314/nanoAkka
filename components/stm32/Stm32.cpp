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

  Event::Event(int tpe, void* ptr = 0) {
    _type = tpe;
    _ptr = ptr;
  }
  bool Event::isOneOf(int v1, int v2 = NOP, int v3 = NOP, int v4 = NOP) const {
    return _type == v1 || _type == v2 || _type == v3 || _type == v4;
  };
  bool Event::is(int v) const { return _type == v; };
  bool Event::isCommand() const { return _type >= ERASE_MEMORY; };
  bool Event::isRxd(std::string& bytes) const {
    return _type == RXD && bytes.compare(*data) == 0;
  };
  void Event::operator=(int v) { _type = v; }
  bool Event::operator==(int v) const { return _type == v; }
  const char* toString() { return strEvents[_type];}

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

  resetToRun();

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
  }
  if ( bytes.length()) me->rxd.on(bytes);
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

int Stm32::stm32Request(struct pt* state,const Event& ev,std::string& req,std::string& response) {
  INFO("Get... [%s] line : %d", strEvents[ev._type], _subState.lc);
  response="";
  PT_BEGIN(state);
  request(50, syncRequest);
  PT_YIELD_UNTIL(state, ev.isRxd(ackReply) || ev.is(TO)));
  stopTimer();
  if (ev.is(RXD)) {
    INFO(" RXD : %s", string_to_hex(*ev.data).c_str());
    request(50, req);
    PT_YIELD_UNTIL(state, ev.isOneOf(RXD, TO));
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

int Stm32::stm32ReadMemory(struct pt* state,const Event& ev,uint32_t address=0x08000000,uint32_t length=256,std::string& memory){
  static struct pt subState;
  std::string response;
  PT_BEGIN(state);
  PT_SPAWN(state,&subState,stm32Request(&subState,ev,readMemoryRequest,response);
  if ( response.compare(ackReply)==0 ){
      PT_SPAWN(state,&subState,stm32Request(&subState,ev,stm32Address(address),response);
      if ( response.compare(ackReply)==0){
        PT_SPAWN(state,&subState,stm32Request(&subState,ev,stm32Length(length),response);
        if ( response.compare(ackReply)==0){
            PT_YIELD_UNTIL(state,ev.is(RXD)){
              memory = *ev.data;
              PT_EXIT(state);
          }
      }
    }
  }
  PT_END(state);
}

int Stm32::dispatch(const Event& ev) {
  std::string response;
  INFO(" dispatch [%s] line %d ", strEvents[ev._type], _mainState.lc);
  PT_BEGIN(&_mainState);
  while (true) {
    PT_YIELD_UNTIL(&_mainState, ev.isCommand());
    resetToProg();
    if (ev == GET_ID) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(&_subState,ev,getIdRequest),response);
    }
    if (ev == GET_REQUEST) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(&_subState,ev,getRequest),response);
    }
    if (ev == GET_VERSION) {
      PT_SPAWN(&_mainState, &_subState, stm32Request(&_subState,ev,getVersionRequest),response);
    }
  }
  PT_END(&_mainState);
}

void Stm32::request(int timeout, std::string& data) {
  write(data);
  _timer.start(timeout);
}

void Stm32::resetToProg() {
if ( _mode == RUN ){
  _mode = PROG;
  _boot0.write(1);
  reset();
    }
}

void Stm32::resetToRun() {
  if ( _mode == PROG ){
  _mode=RUN;
  _boot0.write(0);
  reset();
  }
}

void Stm32::stopTimer() { _timer.stop(); }
byte slice(uint32_t word, int offset) {
	return (byte) ((word >> (offset * 8)) & 0xFF);
}
byte xorBytes(byte* data, uint32_t count) {
	byte x = data[0];
	int i = 1;
	while (i < count) {
		x = x ^ data[i];
		i++;
	}
	return x;
}

std::string Stm32::stm32Address(uint32_t address){
  byte ADDRESS[] = { slice(address, 3), slice(address, 2), slice(address, 1),
			slice(address, 0), xorBytes(ADDRESS, 4) };
      ADDRESS[4]=xorBytes(ADDRESS,4);
      return std::string(ADDRESS,4);
}

std::string Stm32::stm32Length(uint8_t l){
  return std::string(1,l);
}