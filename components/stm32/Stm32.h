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
#include <ProtoThread.h>

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
/*  BL_GET_VERSION = 1,
  BL_GET_ID = 2,
  BL_READ_MEMORY = 0x11,
  BL_GO = 0x21,
  BL_WRITE_MEMORY = 0x31,
  BL_ERASE_MEMORY = 0x43,
  BL_EXTENDED_ERASE_MEMORY = 0x44,
  BL_WRITE_PROTECT = 0x63,
  BL_WRITE_UNPROTECT = 0x73,
  BL_READOUT_PROTECT = 0x82,
  BL_READOUT_UNPROTECT = 0x92,
  BL_ACK = 0x79,
  BL_NACK = 0x1F
} BL;*/
#define XOR(xxx) (xxx ^ 0xFF)

#define DELAY 100

enum { NOP, RXD, TO, ERASE, WRITE, READ, ANALYZE };

class Event {
 public:
  int _type;
  union {
    void* _ptr;
    MqttStream* _mqttStream;
    Bytes* _rxdBytes;
  };

 public:
  bool isOneOf(int v1, int v2 = NOP, int v3 = NOP, int v4 = NOP) const {
    return _type == v1 || _type == v2 || _type == v3 || _type == v4;
  };
  bool is(int v) const { return _type == v; };
  bool isCommand() const { return _type >= ERASE; };
  bool isRxd(const char* s) const {
    return _type == RXD && memcmp(s, _rxdBytes->data(), strlen(s)) == 0;
  };
  bool isRxd(Bytes& bytes) const {
    return _type == RXD &&
           memcmp(bytes.data(), _rxdBytes->data(), bytes.length()) == 0;
  };
  bool isRxd(int size, uint8_t* b) const {
    return _type == RXD && memcmp(b, _rxdBytes->data(), size) == 0;
  };
  bool isCommand(int command) const { return _type == command; };

  void operator=(int v) { _type = v; }
  Event(int tpe, void* ptr = 0) {
    _type = tpe;
    _ptr = ptr;
  }
  bool operator==(int v) const { return _type == v; }
};

class Stm32;

class Analyze : public ProtoThread<Event>,public Stm32 {
  public :
  Analyze(Stm32* stm32):Stm32(stm32){};
  virtual bool dispatch(const Event& event);
};


class Stm32 : public Actor, public ProtoThread<Event> {
  UART& _uart;
  DigitalOut& _reset;
  DigitalOut& _boot0;
  TimerSource _timer;
  TimerSource _testTimer;
  Bytes _supportedCommands; // to be filled at analyze phase
  uint8_t _bootloaderVersion;
  uint16_t _chipId;


  Analyze _analyze(this)


 public:
  Bytes getRequest;
  Bytes getIdRequest; // 0x02+0xFD
  Bytes ackReply;
  Bytes nackReply;
  Bytes syncRequest; // 0x7F

  Sink<MqttStream, 10> ota;
  Sink<Bytes, 5> rxd;
  ValueFlow<std::string> message;
  Stm32(Thread& thr, int pinTxd, int pinRxd, int pinBoot0, int pinReset);
  void init();
  void wiring();
  static void onReceive(void*);
  void reset();
  void resetRun();
  void resetProg();
  void setBoot0(bool);
  bool timeout();
  void timeout(uint32_t delta);
  void startOta(const MqttStream&);
  void stopOta(const MqttStream&);
  void writeOta(const MqttStream&);
  bool dispatch(const Event& ev);
  void write(int, uint8_t*);
  void write(Bytes& b);
  void request(int timeout,Bytes& data);
  void stopTimer();
};

#endif /* STM32_H_ */