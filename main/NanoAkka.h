#ifndef NANOAKKA_H
#define NANOAKKA_H

#include <errno.h>

#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)
// ------------------------------------------------- Linux
#ifdef __linux__
#include <mqueue.h>

#include <string>
#include <thread>
typedef std::string NanoString;
#endif
//--------------------------------------------------  ESP8266
#if defined(ESP_OPEN_RTOS) || defined(ESP_PLATFORM) // ESP_PLATFORM for ESP8266_RTOS_SDK
#define FREERTOS
#define NO_ATOMIC
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <string>
typedef std::string NanoString;
#endif
//-------------------------------------------------- ESP32
#if defined(ESP32_IDF) || defined(ESP8266_IDF)
#include <string>
typedef std::string NanoString;
#define FREERTOS
#include <FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "esp_system.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#define PRO_CPU 0
#define APP_CPU 1
#endif
//-------------------------------------------------- ARDUINO
#ifdef ARDUINO
#define NO_ATOMIC
#include <Arduino.h>
#include <printf.h>
#include <stdarg.h>
typedef String NanoString;
#define INFO(fmt, ...)                                                      \
  {                                                                         \
    char line[256];                                                         \
    int len = snprintf(line, sizeof(line), "I %06llu | %.12s:%.3d | ",      \
                       Sys::millis(), __FILE__, __LINE__);                  \
    snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__); \
    Serial.println(line);                                                   \
    Serial.flush();                                                         \
  }
#define WARN(fmt, ...)                                                      \
  {                                                                         \
    char line[256];                                                         \
    int len = snprintf(line, sizeof(line), "W %06llu | %.12s:%.3d | ",      \
                       Sys::millis(), __FILE__, __LINE__);                  \
    snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__); \
    Serial.println(line);                                                   \
    Serial.flush();                                                         \
  }
#else
#include <Log.h>
#endif
//------------------------------------------------------------------------------------
#ifndef NO_ATOMIC
#include <atomic>
#endif
#undef min
#undef max
#include <Sys.h>

#include <functional>
#include <unordered_map>
#include <vector>

typedef struct
{
  uint32_t bufferOverflow = 0;
  uint32_t bufferPushBusy = 0;
  uint32_t bufferPopBusy = 0;
  uint32_t threadQueueOverflow = 0;
  uint32_t bufferPushCasFailed = 0;
  uint32_t bufferPopCasFailed = 0;
  uint32_t bufferCasRetries = 0;
} NanoStats;
extern NanoStats stats;

//______________________________________________________________________
// INTERFACES nanoAkka
//

template <class T>
class AbstractQueue
{
public:
  virtual int pop(T &t) = 0;
  virtual int push(const T &t) = 0; // const to be able to do something like
                                    // push({"topic","message"});
};

class Named
{
  std::string _name = "no-name";

public:
  Named(const char *name) { _name = name == 0 ? "NULL" : name; }
  const char *name() { return _name.c_str(); }
};
class Invoker
{
public:
  virtual void invoke() = 0;
};

template <class T>
class Subscriber
{
public:
  virtual void on(const T &t) = 0;
  virtual ~Subscriber(){};
};
#include <bits/atomic_word.h>

template <class T>
class SubscriberFunction : public Subscriber<T>
{
  std::function<void(const T &t)> _func;

public:
  SubscriberFunction(std::function<void(const T &t)> func) { _func = func; }
  void on(const T &t) { _func(t); }
};

template <class T>
class Publisher
{
public:
  virtual void subscribe(Subscriber<T> *listener) = 0;
  void operator>>(Subscriber<T> &listener) { subscribe(&listener); }
  void operator>>(Subscriber<T> *listener) { subscribe(listener); }
  void operator>>(std::function<void(const T &t)> func)
  {
    subscribe(new SubscriberFunction<T>(func));
  }
};

class Requestable
{
public:
  virtual void request() = 0;
};
//___________________________________________________________________________
// lockfree buffer, isr ready
//
#define BUSY (1 << 15) // busy read or write ptr

#if defined(ESP_OPEN_RTOS) || defined(ESP8266_IDF) || defined(ESP_PLATFORM)
// Set Interrupt Level
// level (0-15),
// level 15 will disable ALL interrupts,
// level 0 will enable ALL interrupts
//
#define xt_rsil(level)                               \
  (__extension__({                                   \
    uint32_t state;                                  \
    __asm__ __volatile__("rsil %0," STRINGIFY(level) \
                         : "=a"(state));             \
    state;                                           \
  }))
#define xt_wsr_ps(state)                               \
  __asm__ __volatile__("wsr %0,ps; isync" ::"a"(state) \
                       : "memory")
#define interrupts() xt_rsil(0)
#define noInterrupts() xt_rsil(15)
#endif
//#pragma GCC diagnostic ignored "-Warray-bounds"

#ifdef NO_ATOMIC
template <class T, int SIZE>
class ArrayQueue : public AbstractQueue<T>
{
  T _array[SIZE];
  int _readPtr;
  int _writePtr;
  inline int next(int idx) { return (idx + 1) % SIZE; }

public:
  ArrayQueue() { _readPtr = _writePtr = 0; }
  int push(const T &t)
  {
    //    INFO("push %X", this);
    noInterrupts();
    int expected = _writePtr;
    int desired = next(expected);
    if (desired == _readPtr)
    {
      stats.bufferOverflow++;
      interrupts();
      return ENOBUFS;
    }
    _writePtr = desired;
    _array[desired] = std::move(t);
    interrupts();
    return 0;
  }

  int pop(T &t)
  {
    //    INFO("pop %X", this);
    noInterrupts();
    int expected = _readPtr;
    int desired = next(expected);
    if (expected == _writePtr)
    {
      interrupts();
      return ENOBUFS;
    }
    _readPtr = desired;
    t = std::move(_array[desired]);
    interrupts();
    return 0;
  }
};
#else
template <class T, int SIZE>
class ArrayQueue : public AbstractQueue<T>
{
  T _array[SIZE];
  std::atomic<int> _readPtr;
  std::atomic<int> _writePtr;
  inline int next(int idx) { return (idx + 1) % SIZE; }

public:
  ArrayQueue() { _readPtr = _writePtr = 0; }

  int push(const T &t)
  {
    int cnt = 0;
    int expected = 0;
    int desired = 0;
    while (cnt++ < 5)
    {
      expected = _writePtr;
      if (expected & BUSY)
      {
        stats.bufferPushBusy++;
        WARN("BUSY");
        return ENODATA;
      }
      desired = next(expected);
      if (desired == _readPtr % SIZE)
      {
        return ENOBUFS;
      }
      desired |= BUSY;
      if (_writePtr.compare_exchange_strong(expected, desired,
                                            std::memory_order_seq_cst,
                                            std::memory_order_seq_cst))
      {
        expected = desired;
        desired &= ~BUSY;
        _array[desired] = t;
        //        _array[desired] = std::move(t);
        while (_writePtr.compare_exchange_strong(
                   expected, desired, std::memory_order_seq_cst,
                   std::memory_order_seq_cst) == false)
        {
          WARN("writePtr<%s,%d> remove busy failed %u:%u:%u", S(T), SIZE,
               expected, _writePtr.load(), desired);
          stats.bufferCasRetries++;
#ifdef FREERTOS
          vTaskDelay(1);
#endif
        }
        return 0;
      }
      else
      {
        stats.bufferCasRetries++;
#ifdef FREERTOS
        vTaskDelay(1);
#endif
      }
    }
    WARN("writePtr<%s,%d> update failed %u:%u:%u", S(T), SIZE, expected,
         _writePtr.load(), desired);
    stats.bufferPushCasFailed++;
    return -1;
  }

  int pop(T &t)
  {
    int cnt = 0;
    int expected = 0;
    int desired = 0;
    while (cnt++ < 5)
    {
      expected = _readPtr.load();
      if (expected & BUSY)
      {
        stats.bufferPopBusy++;
        WARN("BUSY");
        return ENODATA;
      }
      int desired = next(expected);
      if (expected == _writePtr % SIZE)
      {
        //				WARN("EMPTY");
        return ENOBUFS;
      }

      desired |= BUSY;
      if (_readPtr.compare_exchange_strong(expected, desired,
                                           std::memory_order_seq_cst,
                                           std::memory_order_seq_cst))
      {
        expected = desired;
        desired &= ~BUSY;
        //       t = std::move(_array[desired]);
        t = _array[desired];
        while (_readPtr.compare_exchange_strong(
                   expected, desired, std::memory_order_seq_cst,
                   std::memory_order_seq_cst) == false)
        {
          stats.bufferCasRetries++;
#ifdef FREERTOS
          vTaskDelay(1);
#endif
          WARN("readPtr<%s,%d> remove busy failed %u:%u:%u", S(T), SIZE,
               expected, _readPtr.load(), desired);
        }
        return 0;
      }
      else
      {
        stats.bufferCasRetries++;
#ifdef FREERTOS
        vTaskDelay(1);
#endif
      }
    }
    WARN("readPtr<%s,%d> update failed %u:%u:%u", S(T), SIZE, expected,
         _readPtr.load(), desired);
    stats.bufferPopCasFailed++;
    return -1;
  }
};
#endif

// STREAMS
class TimerSource;
//____________________________________________________________________ THREAD __
class Thread : public Named
{
#ifdef __linux__
  int _pipeFd[2];
  int _writePipe = 0;
  int _readPipe = 0;
#elif defined(FREERTOS)
  QueueHandle_t _workQueue = 0;
#else
  ArrayQueue<Invoker *, 10> _workQueue;
#endif
  uint32_t queueOverflow = 0;
  void createQueue();
  std::vector<TimerSource *> _timers;
  static int _id;

public:
  Thread(const char *name = "noname") : Named(name)
  {
    createQueue();
  }
  Thread() : Named("noName")
  {
    // _name = "thread-%d" + _id++;
    createQueue();
  }
  void start();
  int enqueue(Invoker *invoker);
  int enqueueFromIsr(Invoker *invoker);
  void run();
  void loop();
  void addTimer(TimerSource *ts) { _timers.push_back(ts); }
};

//_____________________________________________________________________ SOURCE _
//
template <class T>
class Source : public Publisher<T>, public Requestable
{
  std::vector<Subscriber<T> *> _listeners;
  T _last;

public:
  void subscribe(Subscriber<T> *listener) { _listeners.push_back(listener); }
  void emit(const T &t)
  {
    _last = t;
    for (Subscriber<T> *listener : _listeners)
    {
      listener->on(t);
    }
  }
  ~Source() { WARN(" Source destructor. Really ? "); }
  void last(T &last) { last = _last; }
};
//____________________________________________________________ LAMBDASOURCE
//
template <class T>
class LambdaSource : public Source<T>
{
  std::function<T()> _handler;

public:
  LambdaSource(std::function<T()> handler) : _handler(handler){};
  void request() { this->emit(_handler()); }
};
//__________________________________________________________________________
//
template <class T>
class RefSource : public Source<T>
{
  T &_t;
  bool _pass = true;

public:
  RefSource(T &t) : _t(t){};
  void request() { this->emit(_t); }
  void operator=(T t)
  {
    _t = t;
    if (_pass)
      this->emit(_t);
  }
  void pass(bool p) { _pass = p; }
  T &operator()() { return _t; }
};
//__________________________________________________________________________
//
template <class T>
class ValueSource : public Source<T>
{
  T _t;
  bool _pass = true;

public:
  ValueSource(){};
  ValueSource(T t) { _t = t; }
  void request() { this->emit(_t); }
  void operator=(T t)
  {
    _t = t;
    if (_pass)
      this->emit(_t);
  }
  T &operator()() { return _t; }
  void pass(bool p) { _pass = p; }
};
//__________________________________________________________________________`
//
// TimerSource
// id : the timer id send with the timer expiration
// interval : time after which the timer expires
// repeat : repetitive timer
//
// run : sink bool to stop or run timer
//	start : restart timer from now+interval
//_______________________________________________________________ TimerSource
//
class TimerMsg
{
public:
  const char *name;
};

class TimerSource : public Source<TimerMsg>, public Named
{
  uint32_t _interval = UINT32_MAX;
  bool _repeat = false;
  uint64_t _expireTime = UINT64_MAX;

  void setNewExpireTime()
  {
    uint64_t now = Sys::millis();
    _expireTime += _interval;
    if (_expireTime < now && _repeat )
      _expireTime = now + _interval;
  }

public:
  TimerSource(Thread &thr, uint32_t interval, bool repeat, const char *name = "unknownTimer1") : Named(name)
  {
    _interval = interval;
    _repeat = repeat;
    if (repeat)
      start();
    thr.addTimer(this);
  }
  TimerSource(Thread &thr, const char *name = "unknownTimer2") : TimerSource(thr, UINT32_MAX, false, name)
  {
    thr.addTimer(this);
  }

  TimerSource(const char *name = "unknownTimer3") : Named(name) { _expireTime = Sys::now() + _interval; };
  ~TimerSource() { WARN(" timer destructor. Really ? "); }

  void attach(Thread &thr) { thr.addTimer(this); }
  void reset() { start(); }
  void start() { _expireTime = Sys::millis() + _interval; }
  void start(uint32_t interval)
  {
    _interval = interval;
    start();
  }
  void stop() { _expireTime = UINT64_MAX; }
  void interval(uint32_t i) { _interval = i; }
  void request()
  {
    if (Sys::millis() >= _expireTime)
    {
      if (_repeat)
        setNewExpireTime();
      else
        _expireTime = Sys::millis() + UINT32_MAX;
      TimerMsg tm = {name()};
      this->emit(tm);
    }
  }
  void repeat(bool r) { _repeat = r; };
  uint64_t expireTime() { return _expireTime; }
  inline uint32_t interval() { return _interval; }
};
//____________________________________  SINK ______________________
template <class T, int S>
class Sink : public Subscriber<T>, public Invoker, public Named
{
  ArrayQueue<T, S> _t;
  std::function<void(const T &)> _func;
  Thread *_thread = 0;
  T _lastValue;

public:
  Sink(const char *name = "unknown") : Named(name)
  {
    _func = [&](const T &t) { WARN(" no handler attached to this sink "); };
  }
  ~Sink() { WARN(" Sink destructor. Really ? "); }
  Sink(std::function<void(const T &)> handler, const char *name = "unknownSink") : Named(name), _func(handler){};

  void on(const T &t)
  {
    if (_thread)
    {
      while (_t.push(t) != 0)
      {
        stats.bufferOverflow++;
        T t1;
        _t.pop(t1); // drop oldest from queue
        INFO("dropped oldest item from %s ", name());
      }
      _thread->enqueue(this);
    }
    else
    {
      _func(t);
    }
  }
  /*
  virtual void request() {
    invoke();
  }*/
  void invoke()
  {
    if (_t.pop(_lastValue))
    {
      WARN(" no data in queue[%d] ", S); // second subscriber
                                         //          _func(_lastValue);
    }
    else
    {
      _func(_lastValue);
    }
  }

  void async(Thread &thread, std::function<void(const T &)> func)
  {
    _func = func;
    _thread = &thread;
  }
  void sync(std::function<void(const T &)> func)
  {
    _thread = 0;
    _func = func;
  }
  T operator()() { return _lastValue; }
};

//_________________________________________________ Flow ________________
//
template <class IN, class OUT>
class Flow : public Subscriber<IN>, public Source<OUT>
{
public:
  void operator==(Flow<OUT, IN> &flow)
  {
    this->subscribe(&flow);
    flow.subscribe(this);
  };
};
// -------------------------------------------------------- Cache
template <class T>
class Cache : public Flow<T, T>, public Subscriber<TimerMsg>
{
  Thread &_thread;
  uint32_t _min, _max;
  bool _unsendValue = false;
  uint64_t _lastSend;
  T _t;
  TimerSource _timerSource;

public:
  Cache(Thread &thread, uint32_t min, uint32_t max, bool request = false)
      : _thread(thread), _min(min), _max(max), _timerSource(thread)
  {
    _timerSource.interval(min);
    _timerSource.start();
    _timerSource.subscribe(this);
  }
  void on(const T &t)
  {
    _t = t;
    uint64_t now = Sys::millis();
    _unsendValue = true;
    if (_lastSend + _min < now)
    {
      this->emit(t);
      _unsendValue = false;
      _lastSend = now;
    }
  }
  void on(const TimerMsg &tm)
  {
    uint64_t now = Sys::millis();

    if (_unsendValue)
    {
      this->emit(_t);
      _unsendValue = false;
      _lastSend = now;
    }
    if (now > _lastSend + _max)
    {
      this->emit(_t);
      _unsendValue = false;
      _lastSend = now;
    }
    _timerSource.reset();
  }
  void request() { this->emit(_t); }

  static Cache<T> &nw(Thread &t, uint32_t min, uint32_t max)
  {
    auto cache = new Cache<T>(t, min, max);
    return *cache;
  }
};
//_____________________________________________________________________________
//
template <class T, int S>
class QueueFlow : public Flow<T, T>, public Invoker
{
  ArrayQueue<T, S> _queue;
  std::function<void(const T &)> _func;
  Thread *_thread = 0;

public:
  void on(const T &t)
  {
    if (_thread)
    {
      while (_queue.push(t) != 0)
      {
        stats.bufferOverflow++;
        T t1;
        _queue.pop(t1);
      }
      _thread->enqueue(this);
    }
    else
    {
      this->emit(t);
    }
  }
  void request() { invoke(); }
  void invoke()
  {
    T value;
    if (_queue.pop(value))
    {
      WARN(" no data ");
    }
    else
    {
      this->emit(value);
    }
  }

  void async(Thread &thread) { _thread = &thread; }
  void sync(std::function<void(const T &)> func) { _thread = 0; }
};

//________________________________________________________________
//

template <class IN, class OUT>
class LambdaFlow : public Flow<IN, OUT>
{
  std::function<int(OUT &, const IN &)> _func;

public:
  LambdaFlow()
  {
    _func = [](OUT &out, const IN &in) {
      WARN("no handler for this flow");
      return ENODATA;
    };
  };
  LambdaFlow(std::function<int(OUT &, const IN &)> func) : _func(func){};
  void lambda(std::function<int(OUT &, const IN &)> func) { _func = func; }
  virtual void on(const IN &in)
  {
    OUT out;
    if (_func(out, in) == 0)
      this->emit(out);
  }
  void request(){};
  static LambdaFlow<IN, OUT> &nw(std::function<int(OUT &, const IN &)> func)
  {
    auto lf = new LambdaFlow(func);
    return *lf;
  }
};

//________________________________________________________________
//
template <class IN, class OUT>
Source<OUT> &operator>>(Publisher<OUT> &publisher, Flow<IN, OUT> &flow)
{
  publisher.subscribe(&flow);
  return flow;
}

//________________________________________________________________
//
template <class T>
class ValueFlow : public Flow<T, T>
{
  T _t;
  bool _pass = false;

public:
  ValueFlow(){};
  ValueFlow(T t) { _t = std::move(t); }
  void request() { this->emit(_t); }
  void operator=(T t)
  {
    _t = std::move(t);
    if (_pass)
      this->emit(_t);
  }
  T &operator()() { return _t; }
  void on(const T &in)
  {
    _t = std::move(in);
    this->emit(_t);
  }
  void pass(bool p) { _pass = p; }
};
//______________________________________ Actor __________________________
//
class Actor
{
  Thread &_thread;

public:
  Actor(Thread &thread) : _thread(thread) {}
  Thread &thread() { return _thread; }
};

#endif // NANOAKKA_H
