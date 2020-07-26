/*
 * Stm32.h
 *
 *  Created on: Jun 28, 2016
 *      Author: lieven
 */

#ifndef STM32_H_
#define STM32_H_

#include <Bytes.h>
#include <ConfigFlow.h>
#include <Hardware.h>
#include <Mqtt.h>
#include <NanoAkka.h>
#include <pt.h>

#define BL_GET 0
#define BL_GET_VERSION 1
#define BL_GET_ID 2
#define BL_READ_MEMORY 0x11
#define BL_GO 0x21
#define BL_WRITE_MEMORY 0x31
#define BL_ERASE_MEMORY 0x43
#define BL_EXTENDED_ERASE_MEMORY 0x44
#define BL_WRITE_PROTECT 0x63
#define BL_WRITE_UNPROTECT 0x73
#define BL_READOUT_PROTECT 0x82
#define BL_READOUT_UNPROTECT 0x92
#define BL_ACK 0x79
#define BL_NACK 0x1F
#define BL_SYNC 0x7F
#define XOR(xxx) (char)(xxx ^ 0xFF)

#define DELAY 100

// don't forget to update strEvents if you add type
enum {
  NOP,
  RXD,
  TO,
  ERASE_MEMORY,
  WRITE_MEMORY,
  READ_MEMORY,
  GET_ID,
  GET_REQUEST,
  GET_VERSION,
  RUN
};

class Event {
 public:
  int _type;
  union {
    void* _ptr;
    std::string* data;
  };

 public:
  Event(int tpe, void* ptr = 0);
  bool isOneOf(int v1, int v2 = NOP, int v3 = NOP, int v4 = NOP) const;
  bool is(int v) const;
  bool isCommand() const;
  bool isRxd(std::string& bytes) const;
  bool isTimeout() const;
  void operator=(int v);
  bool operator==(int v) const;
  const char* toString();
};

class Stm32 : public Actor {
  UART& _uart;
  DigitalOut& _reset;
  DigitalOut& _boot0;
  TimerSource _timer;
  TimerSource _testTimer;
  Bytes _supportedCommands;  // to be filled at analyze phase
  uint8_t _bootloaderVersion;
  uint16_t _chipId;
  struct pt _mainState, _subState;
  enum { PROG, RUN, ERROR } _mode;
  uint32_t _nextAddress;

 public:
  std::string getRequest = {BL_GET, XOR(BL_GET)};
  std::string getIdRequest = {BL_GET_ID, XOR(BL_GET_ID)};  // 0x02+0xFD
  std::string getVersionRequest = {BL_GET_VERSION,
                                   XOR(BL_GET_VERSION)};  // 0x02+0xFD
  std::string ackReply = {BL_ACK};
  std::string nackReply = {BL_NACK};
  std::string syncRequest = {BL_SYNC};  // 0x7F
  std::string readMemoryRequest = {BL_READ_MEMORY, XOR(BL_READ_MEMORY)};
  std::string writeMemoryRequest = {BL_WRITE_MEMORY, XOR(BL_WRITE_MEMORY)};
  std::string eraseMemoryRequest = {BL_ERASE_MEMORY, XOR(BL_ERASE_MEMORY)};
  std::string readMemory;
  std::string writeBuffer;

  Sink<MqttBlock, 10> ota;
  Sink<std::string, 5> rxd;
  ValueFlow<std::string> message;
  ValueFlow<uint32_t> startAddress;
  ValueFlow<uint32_t> baudrate;
  ValueFlow<MqttBlock> blocks;
  ConfigFlow<uint32_t> progBaudrate;
  ConfigFlow<uint32_t> logBaudrate;
  ConfigFlow<uint32_t> flashStartAddress;

  Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset);
  void init();
  void wiring();
  static void onReceive(void*);
  void reset();
  void resetToRun();
  bool resetToProg();
  void setBoot0(bool);
  bool timeout();
  void timeout(uint32_t delta);

  bool startOta(const MqttBlock&);
  bool stopOta(const MqttBlock&);
  bool writeOta(const MqttBlock&);

  bool write(std::string);
  bool write(uint8_t data);
  bool write(uint8_t* data, uint32_t length);
  bool writeBlock(uint8_t* data, uint32_t length);
  bool saveAnyQuadFragment(uint8_t* data, uint32_t length);
  uint32_t completeAndWriteQuadFragment(uint8_t* data, uint32_t length);
  void request(int timeout, std::string&);
  void stopTimer();
  int dispatch(const Event& ev);
  int blRequest(struct pt* state, const Event&, std::string request,
                std::string& response);
  int blReadMemory(struct pt* state, const Event& ev, uint32_t address,
                   uint32_t length, std::string& memory);
  int blWriteMemory(struct pt* state, const Event& ev, uint32_t address,
                    uint32_t length, std::string& memory);
  int blEraseMemory(struct pt* state, const Event& ev);
  bool resetToProgSync();
  bool resetToRunSync();
  bool eraseMemorySync();
  bool writeMemorySync(uint32_t address, uint8_t* data, uint32_t length);
  bool waitFor(std::string response, uint32_t timeout = 50);
  std::string blAddress(uint32_t);
  std::string blLength(uint8_t);
  void dump(std::string&);
};

#endif /* STM32_H_ */