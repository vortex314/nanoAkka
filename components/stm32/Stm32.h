/*
 * Stm32.h
 *
 *  Created on: Jun 28, 2016
 *      Author: lieven
 */

#ifndef STM32_H_
#define STM32_H_

#include <Bytes.h>
#include <Hardware.h>
#include <NanoAkka.h>
#include <pt.h>

class MqttStream {
 public:
  std::string topic;
  //  Bytes data;
  uint32_t offset;
  uint32_t length;
  uint32_t total;
  MqttStream operator=(const MqttStream& other) {
    topic = other.topic;
    //    data=other.data;
    offset = other.offset;
    length = other.length;
    total = other.total;
    return *this;
  }
};

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
  GET_VERSION
};

class Event {
 public:
  int _type;
  union {
    void* _ptr;
    MqttStream* _mqttStream;
    std::string* data;
  };

 public:
  Event(int tpe, void* ptr = 0);
  bool isOneOf(int v1, int v2 = NOP, int v3 = NOP, int v4 = NOP) const ;
  bool is(int v) const ;
  bool isCommand() const ;
  bool isRxd(std::string& bytes) const;
  bool isTimeout() const;
  void operator=(int v) ;
  bool operator==(int v) const ;
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
  enum { PROG, RUN } _mode;

 public:
  std::string getRequest = {BL_GET, XOR(BL_GET)};
  std::string getIdRequest = {BL_GET_ID, XOR(BL_GET_ID)};  // 0x02+0xFD
  std::string getVersionRequest = {BL_GET_VERSION,XOR(BL_GET_VERSION)};  // 0x02+0xFD
  std::string ackReply = {BL_ACK};
  std::string nackReply = {BL_NACK};
  std::string syncRequest = {BL_SYNC};  // 0x7F
  std::string readMemoryRequest = {BL_READ_MEMORY,XOR(BL_READ_MEMORY)}

  Sink<MqttStream, 10> ota;
  Sink<std::string, 5> rxd;
  ValueFlow<std::string> message;
  Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset);
  void init();
  void wiring();
  static void onReceive(void*);
  void reset();
  void resetToRun();
  void resetToProg();
  void setBoot0(bool);
  bool timeout();
  void timeout(uint32_t delta);
  void startOta(const MqttStream&);
  void stopOta(const MqttStream&);
  void writeOta(const MqttStream&);
  void write(std::string&);
  void request(int timeout, std::string&);
  void stopTimer();
  int dispatch(const Event& ev);
  int stm32Request(const Event&, std::string& request,std::string& response);
  int stm32ReadMemory(uint32_t address);
  std::string  stm32Address(uint32_t);
  std::string stm32Length(uint8_t);
  void dump(std::string&);
};

#endif /* STM32_H_ */