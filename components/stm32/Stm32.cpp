/*
 * Stm32.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: lieven
 */
#include "Stm32.h"

std::string string_to_hex(const std::string& input, char filler = ' ') {
  static const char hex_digits[] = "0123456789ABCDEF";

  std::string output;
  output.reserve(input.length() * 3);
  for (unsigned char c : input) {
    output.push_back(hex_digits[c >> 4]);
    output.push_back(hex_digits[c & 15]);
    output.push_back(filler);
  }
  return output;
}

uint8_t xorBytes(uint8_t* data, uint32_t count) {
  uint8_t x = data[0];
  int i = 1;
  while (i < count) {
    x = x ^ data[i];
    i++;
  }
  return x;
}

const char* strEvents[] = {
    "NOP",         "RXD",    "TO",          "ERASE_MEMORY", "WRITE_MEMORY",
    "READ_MEMORY", "GET_ID", "GET_REQUEST", "GET_VERSION",  "RUN"};

Event::Event(int tpe, void* ptr) {
  _type = tpe;
  _ptr = ptr;
}
bool Event::isOneOf(int v1, int v2, int v3, int v4) const {
  //  INFO(" %d in %d %d %d %d result : %d",_type,v1,v2,v3,v4,_type == v1 ||
  //  _type == v2 || _type == v3 || _type == v4);
  return _type == v1 || _type == v2 || _type == v3 || _type == v4;
};
bool Event::is(int v) const { return _type == v; };
bool Event::isCommand() const { return _type >= ERASE_MEMORY; };
bool Event::isRxd(std::string& bytes) const {
  return (_type == RXD) && (bytes.compare(*data) == 0);
};
void Event::operator=(int v) { _type = v; }
bool Event::operator==(int v) const { return _type == v; }
const char* Event::toString() { return strEvents[_type]; }

Stm32::Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset)
    : Actor(thr),
      _uart(UART::create(2, pinTxd, pinRxd)),
      _reset(DigitalOut::create(pinReset)),
      _boot0(DigitalOut::create(pinBoot0)),
      _timer(thread(), 1000, false),
      _testTimer(thread(), 1000, true),
      progBaudrate("stm32/progBaudrate", 512000),
      logBaudrate("stm32/logBaudrate", 115200),
      flashStartAddress("stm32/flashStartAddress", 0x8000000) {}

void Stm32::init() {
  _timer.stop();

  _uart.setClock(512000);
  //  _uart.onRxd(onReceive, this);
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
  vTaskDelay(0);
  _reset.write(1);
  vTaskDelay(0);
}

void Stm32::wiring() {
  // ota needs to be sync, as not enough memory to store
  ota.sync([&](const MqttBlock& msg) {
    if (msg.offset == 0) startOta(msg);
    writeOta(msg);
    if (msg.offset + msg.length == msg.total) stopOta(msg);
  });
  _timer >> [&](const TimerMsg& tm) { dispatch({TO, 0}); };

  _testTimer >> [&](const TimerMsg& tm) {};
}

int Stm32::blRequest(struct pt* state, const Event& ev, std::string req,
                     std::string& response) {
  INFO("Request... [%s] line : %d", strEvents[ev._type], _subState.lc);
  response = "";
  PT_BEGIN(state);

  request(50, req);
  PT_YIELD_UNTIL(state, ev.isOneOf(RXD, TO));
  stopTimer();
  if (ev.is(RXD)) {
    if (ev.data->length() > 0) {
      response = *ev.data;
      message.on("Request succeeded.");
    } else {
      message.on("GET failed : too short answer");
    }
  } else {
    message.on("Request failed : timeout .");
  }
  PT_END(&_subState);
}

int Stm32::blEraseMemory(struct pt* state, const Event& ev) {
  INFO("EraseMemory... [%s] line : %d", strEvents[ev._type], state->lc);
  static struct pt subState;
  std::string response = "";
  PT_BEGIN(state);
  PT_SPAWN(state, &subState,
           blRequest(&subState, ev, eraseMemoryRequest, response));
  if (response.compare(ackReply) == 0) {
    PT_SPAWN(state, &subState,
             blRequest(&subState, ev, blLength(255), response));
    if (response.compare(ackReply) == 0) {
      message.on("Memory erased");
      PT_EXIT(state);
    } else {
      _mode = ERROR;
    }
  } else {
    _mode = ERROR;
  }
  PT_END(state);
}

bool Stm32::startOta(const MqttBlock& msg) {
  message.on("OTA start.");
  resetToProg();
  waitFor("flush");
  write(syncRequest);
  _nextAddress = flashStartAddress();
  writeBuffer.clear();
  if (!waitFor(ackReply)) return false;
  return eraseMemorySync();
}

bool Stm32::stopOta(const MqttBlock& msg) {
  // complete fragment with 0xFF and write
  uint8_t filler[] = {0xFF};
  while (writeBuffer.length() % 4 != 0) writeBuffer.append((char*)filler, 1);
  writeMemorySync(_nextAddress, (uint8_t*)writeBuffer.data(),
                  writeBuffer.length());
  _nextAddress += writeBuffer.length();
  message.on("OTA end.");
  resetToRun();
  return true;
}

bool Stm32::writeOta(const MqttBlock& msg) {
  uint32_t offset = 0;
  while (offset < msg.length) {
    uint32_t bufferSpace = 256 - writeBuffer.length();
    uint32_t available = msg.length - offset;
    uint32_t toWrite = bufferSpace > available ? available : bufferSpace;
    writeBuffer.append((char*)msg.data + offset, toWrite);
    if (writeBuffer.length() == 256) {
      writeMemorySync(_nextAddress, (uint8_t*)writeBuffer.data(), 256);
      writeBuffer.clear();
      _nextAddress += 256;
    }
    offset += toWrite;
  }

  return true;
}

bool Stm32::waitFor(std::string reply, uint32_t timeout) {
  uint64_t endTime = Sys::millis() + timeout;
  std::string data;
  while (Sys::millis() < endTime) {
    while (_uart.hasData()) data += _uart.read();
    if (data.compare(reply) == 0) {
      DEBUG("RXD OK  : %s", string_to_hex(data).c_str());
      return true;
    }
    vTaskDelay(0);
  }
  INFO("RXD NOK : %s", string_to_hex(data).c_str());
  return false;
}

bool Stm32::waitData(std::string& data, uint32_t length, uint32_t timeout) {
  uint64_t endTime = Sys::millis() + timeout;
  data.clear();
  while (Sys::millis() < endTime && data.size() < length) {
    while (_uart.hasData()) data += _uart.read();
    if (data.size() == length) {
      DEBUG("RXD OK  : %s", string_to_hex(data).c_str());
      return true;
    }
    vTaskDelay(0);
  }
  INFO("RXD NOK : %s", string_to_hex(data).c_str());
  return false;
}

bool Stm32::eraseMemorySync() {
  return write(eraseMemoryRequest) && waitFor(ackReply) &&
         write(blLength(255)) && waitFor(ackReply);
}

bool Stm32::write(uint8_t data) { return _uart.write(data) == E_OK; }

bool Stm32::write(uint8_t* data, uint32_t length) {
  return _uart.write(data, length) == E_OK;
}

bool Stm32::write(std::string s) {
  DEBUG("TXD : %s", string_to_hex(s).c_str());
  return _uart.write((uint8_t*)s.data(), s.length()) == E_OK;
};

bool Stm32::writeBlock(uint8_t* data, uint32_t length) {
  return write(length - 1) && write(data, length) &&
         write(((uint8_t)(length - 1)) ^ xorBytes(data, length));
}

// data should be multiple of 4
bool Stm32::writeMemorySync(uint32_t address, uint8_t data[], uint32_t length) {
  INFO(" 0x%X : %d ", address, length);
  return write(writeMemoryRequest) && waitFor(ackReply) &&
         write(blAddress(address)) && waitFor(ackReply) &&
         writeBlock(data, length) && waitFor(ackReply, 200);
}

bool Stm32::readMemorySync(uint32_t startAddress, uint32_t length) {
  std::string buffer;
  for (uint32_t offset = 0; offset < length; offset += 256) {
    uint32_t size = offset + 256 < length ? 256 : length - offset;
    bool success = write(readMemoryRequest) && waitFor(ackReply) &&
                   write(blAddress(startAddress + offset)) &&
                   waitFor(ackReply) && write(blLength(size - 1)) &&
                   waitData(buffer, size, 200);
    if (!success) return false;
  }
  return true;
}

bool Stm32::getId() { return write(getIdRequest) && waitFor(ackReply); }
/*
int Stm32::dispatch(const Event& ev) {
  std::string response;
  INFO("dispatch [%s] line %d ", strEvents[ev._type], _mainState.lc);
  PT_BEGIN(&_mainState);
  while (true) {
    PT_YIELD_UNTIL(&_mainState, ev.isCommand());
    while (_mode != PROG) {
      resetToProg();
      request(50, syncRequest);
      PT_YIELD_UNTIL(&_mainState, ev.isOneOf(RXD, TO));
      stopTimer();
      if (ev.isRxd(ackReply)) {
        INFO(" PROG mode ");
        _mode = PROG;
      } else {
        INFO(" ERROR mode ");
        _mode = ERROR;
      }
    }
    if (ev == GET_ID) {
      PT_SPAWN(&_mainState, &_subState,
               blRequest(&_subState, ev, getIdRequest, response));
    }
    if (ev == GET_REQUEST) {
      PT_SPAWN(&_mainState, &_subState,
               blRequest(&_subState, ev, getRequest, response));
    }
    if (ev == GET_VERSION) {
      PT_SPAWN(&_mainState, &_subState,
               blRequest(&_subState, ev, getVersionRequest, response));
    }
    if (ev == ERASE_MEMORY) {
      PT_SPAWN(&_mainState, &_subState, blEraseMemory(&_subState, ev));
    }
    if (ev == READ_MEMORY) {
      PT_SPAWN(
          &_mainState, &_subState,
          blReadMemory(&_subState, ev, flashStartAddress(), 256, readMemory));
      INFO(" readMemory length : %d", readMemory.length());
    }
  }
  PT_END(&_mainState);
}

*/

void Stm32::request(int timeout, std::string& data) {
  write(data);
  _timer.start(timeout);
}

bool Stm32::resetToProg() {
  if (_mode != PROG) {
    _mode = PROG;
    _boot0.write(1);
    reset();
  }
  return true;
}

void Stm32::resetToRun() {
  if (_mode != RUN) {
    _mode = RUN;
    _boot0.write(0);
    reset();
  }
}

void Stm32::stopTimer() { _timer.stop(); }
uint8_t slice(uint32_t word, int offset) {
  return (uint8_t)((word >> (offset * 8)) & 0xFF);
}

std::string Stm32::blAddress(uint32_t address) {
  uint8_t ADDRESS[] = {slice(address, 3), slice(address, 2), slice(address, 1),
                       slice(address, 0), 0};
  ADDRESS[4] = xorBytes(ADDRESS, 4);
  std::string a((const char*)ADDRESS, sizeof(ADDRESS));
  return a;
}

std::string Stm32::blLength(uint8_t length) {
  std::string bytes = {(char)length, XOR(length)};
  return bytes;
}