#ifndef NANOAKKA_H
#define NANOAKKA_H

#include <errno.h>
#include <atomic>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)
//--------------------------------------------------  ESP8266
#ifdef ESP_OPEN_RTOS
#define FREERTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
typedef std::string NanoString;
#endif
//-------------------------------------------------- ESP32
#ifdef ESP32_IDF
typedef std::string NanoString;
#define FREERTOS
#include <FreeRTOS.h>
#include "esp_system.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>
#define PRO_CPU 0
#define APP_CPU 1
#endif
//-------------------------------------------------- ARDUINO
#ifdef ARDUINO
#include <Arduino.h>
#include <printf.h>
#include <stdarg.h>
typedef String NanoString;
#define INFO(fmt, ...)                                                          \
	{                                                                           \
		char line[256];                                                         \
		int len = snprintf(line, sizeof(line), "I %06lld | %.12s:%.3d | ",      \
		                   Sys::millis(), __FILE__, __LINE__);                  \
		snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__); \
		Serial.println(line);                                                   \
	}
#define WARN(fmt, ...)                                                          \
	{                                                                           \
		char line[256];                                                         \
		int len = snprintf(line, sizeof(line), "W %06lld | %.12s:%.3d | ",      \
		                   Sys::millis(), __FILE__, __LINE__);                  \
		snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__); \
		Serial.println(line);                                                   \
	}
#else
#include <Log.h>
#endif
//------------------------------------------------------------------------------------

#include <Sys.h>

typedef struct {
	uint32_t bufferOverflow = 0;
	uint32_t bufferPushBusy = 0;
	uint32_t bufferPopBusy = 0;
	uint32_t threadQueueOverflow = 0;
	uint32_t bufferPushCasFailed=0;
	uint32_t bufferPopCasFailed=0;
	uint32_t bufferCasRetries=0;
} NanoStats;
extern NanoStats stats;

//______________________________________________________________________ INTERFACES nanoAkka
//

template <class T>
class AbstractQueue {
	public:
		virtual int pop(T &t) = 0;
		virtual int push(const T &t) = 0; // const to be able to do something like
		// push({"topic","message"});
};

class Invoker {
	public:
		virtual void invoke() = 0;
};

template <class T>
class Subscriber {
	public:
		virtual void on(const T &t) = 0;
		virtual ~Subscriber() {};
};
#include <bits/atomic_word.h>

template <class T>
class SubscriberFunction : public Subscriber<T> {
		std::function<void(const T &t)> _func;

	public:
		SubscriberFunction(std::function<void(const T &t)> func) { _func = func; }
		void on(const T &t) { _func(t); }
};

template <class T>
class Publisher {
	public:
		virtual void subscribe(Subscriber<T> *listener) = 0;
		void operator>>(Subscriber<T> &listener) { subscribe(&listener); }
		void operator>>(Subscriber<T> *listener) { subscribe(listener); }
		void operator>>(std::function<void(const T &t)> func) { subscribe(new SubscriberFunction<T>(func)); }
};

class Requestable {
	public:
		virtual void request() = 0;
};
//___________________________________________________________________________ lockfree buffer, isr ready
//
#define BUSY (1 << 15) // busy read or write ptr

#ifdef ESP_OPEN_RTOS
// Set Interrupt Level
// level (0-15),
// level 15 will disable ALL interrupts,
// level 0 will enable ALL interrupts
//
#define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," STRINGIFY(level) : "=a" (state)); state; }))
#define xt_wsr_ps(state) __asm__ __volatile__("wsr %0,ps; isync" ::"a"(state) \
        : "memory")
#define interrupts() xt_rsil(0)
#define noInterrupts() xt_rsil(15)
#endif
//#pragma GCC diagnostic ignored "-Warray-bounds"

template <class T, int SIZE>
class ArrayQueue : public AbstractQueue<T> {
		T _array[SIZE];
		std::atomic<int> _readPtr;
		std::atomic<int> _writePtr;
		inline int next(int idx) {
			return (idx + 1) % SIZE;
		}

	public:
		ArrayQueue() { _readPtr = _writePtr = 0; }
		int push(const T &t) {

#if defined(ESP_OPEN_RTOS) // || defined(ARDUINO)
			noInterrupts();
			int expected = _writePtr;
			int desired = next(expected);
			if (desired == _readPtr) {
				stats.bufferOverflow++;
				interrupts();
				return ENOBUFS;
			}
			_writePtr = desired;
			_array[desired] = std::move(t);
			interrupts();
			return 0;
#else

			int cnt=0;
			int expected=0;
			int desired=0;
			while(cnt++<5) {
				expected = _writePtr;
				if (expected & BUSY) {
					stats.bufferPushBusy++;
					WARN("BUSY");
					return ENODATA;
				}
				desired = next(expected);
				if (desired == _readPtr % SIZE ) {
					stats.bufferOverflow++;
					return ENOBUFS;
				}
				desired |= BUSY;
				if (_writePtr.compare_exchange_strong(expected, desired,
				                                      std::memory_order_seq_cst,
				                                      std::memory_order_seq_cst)) {
					expected = desired;
					desired &= ~BUSY;
					_array[desired] = std::move(t);
					while (_writePtr.compare_exchange_strong(expected, desired,
					        std::memory_order_seq_cst,
					        std::memory_order_seq_cst)==false) {
						WARN("writePtr<%s,%d> remove busy failed %u:%u:%u",S(T), SIZE,expected, _writePtr.load(), desired);
						stats.bufferCasRetries++;
						vTaskDelay(1);
					}
					return 0;

				} else {
					stats.bufferCasRetries++;
					vTaskDelay(1);
				}
			}
			WARN("writePtr<%s,%d> update failed %u:%u:%u",S(T),SIZE,expected,_writePtr.load(),desired);
			stats.bufferPushCasFailed++;
			return -1;
#endif
		}

		int pop(T &t) {

#if defined(ESP_OPEN_RTOS) //|| defined(ARDUINO)
			noInterrupts();
			int expected = _readPtr;
			int desired = next(expected);
			if (expected == _writePtr) {
				interrupts();
				return ENOBUFS;
			}
			_readPtr = desired;
			t = std::move(_array[desired]);
			interrupts();
			return 0;
#else
			int cnt=0;
			int expected=0;
			int desired=0;
			while(cnt++<5) {
				expected = _readPtr.load();
				if (expected & BUSY) {
					stats.bufferPopBusy++;
					WARN("BUSY");
					return ENODATA;
				}
				int desired = next(expected);
				if (expected == _writePtr%SIZE) {
					//				WARN("EMPTY");
					return ENOBUFS;
				}

				desired |= BUSY;
				if (_readPtr.compare_exchange_strong(expected, desired,
				                                     std::memory_order_seq_cst,
				                                     std::memory_order_seq_cst)) {
					expected = desired;
					desired &= ~BUSY;
					t = std::move(_array[desired]);
					while (_readPtr.compare_exchange_strong(expected, desired,
					                                        std::memory_order_seq_cst,
					                                        std::memory_order_seq_cst)==false) {
						stats.bufferCasRetries++;
						vTaskDelay(1);
						WARN("readPtr<%s,%d> remove busy failed %u:%u:%u",S(T),SIZE, expected, _readPtr.load(), desired);
					}
					return 0;

				} else {
					stats.bufferCasRetries++;
					vTaskDelay(1);
				}
			}
			WARN("readPtr<%s,%d> update failed %u:%u:%u",S(T),SIZE,expected,_readPtr.load(),desired);
			stats.bufferPopCasFailed++;
			return -1;
#endif
		}
};
// STREAMS
class TimerSource;

class Thread {
#ifdef FREERTOS
		QueueHandle_t _workQueue = 0;
#else
		ArrayQueue<Invoker *, 10> _workQueue;
#endif
		uint32_t queueOverflow = 0;
		void createQueue();
		std::vector<TimerSource *> _timers;
		static int _id;
		NanoString _name;

	public:
		Thread(const char *name) {
			_name = name;
			createQueue();
		}
		Thread() {
			_name = "thread-%d" + _id++;
			createQueue();
		}
		void start();
		int enqueue(Invoker *invoker);
		int enqueueFromIsr(Invoker *invoker);
		void run();
		void loop();
		void addTimer(TimerSource *ts) { _timers.push_back(ts); }
};

//__________________________________________________________________________`
//
template <class T>
class Source : public Publisher<T>, public Requestable {
		std::vector<Subscriber<T> *> _listeners;

	public:
		void subscribe(Subscriber<T> *listener) { _listeners.push_back(listener); }
		void emit(const T &t) {
			for (Subscriber<T> *l : _listeners) {
				l->on(t);
			}
		}
};
//__________________________________________________________________________`
//
template <class T>
class LambdaSource : public Source<T> {
		std::function<T()> _handler;

	public:
		LambdaSource(std::function<T()> handler) : _handler(handler) {};
		void request() { this->emit(_handler()); }
};
//__________________________________________________________________________
//
//__________________________________________________________________________
//
template <class T>
class ValueSource : public Source<T> {
		T _t;
		bool _pass = true;

	public:
		ValueSource() {};
		ValueSource(T t) { _t = t; }
		void request() { this->emit(_t); }
		void operator=(T t) {
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
class TimerMsg {
	public:
		uint32_t id;
};

class TimerSource : public Source<TimerMsg> {
		uint32_t _interval = UINT32_MAX;
		bool _repeat = false;
		uint64_t _expireTime = UINT64_MAX;
		uint32_t _id = 0;
		void setNewExpireTime() {
			uint64_t now = Sys::millis();
			_expireTime += _interval;
			if (_expireTime < now)
				_expireTime = now + _interval;
		}

	public:
		TimerSource(Thread &thr, int id, uint32_t interval, bool repeat) {
			_id = id;
			_interval = interval;
			_repeat = repeat;
			if (repeat)
				start();
			thr.addTimer(this);
		}
		TimerSource(Thread &thr) : TimerSource(thr, 0, UINT32_MAX, false) {
			thr.addTimer(this);
		}

		TimerSource() { _expireTime = Sys::now() + _interval; };

		void attach(Thread &thr) { thr.addTimer(this); }
		void reset() { start(); }
		void start() { _expireTime = Sys::millis() + _interval; }
		void stop() { _expireTime = UINT64_MAX; }
		void interval(uint32_t i) { _interval = i; }
		void request() {
			if (Sys::millis() >= _expireTime) {
				if (_repeat)
					setNewExpireTime();
				else
					_expireTime = Sys::millis() + UINT32_MAX;
				TimerMsg tm = {_id};
				this->emit(tm);
			}
		}
		uint64_t expireTime() { return _expireTime; }
		inline uint32_t interval() { return _interval; }
};
//-______________________________________________________ Sink
//______________________
template <class T, int S>
class Sink : public Subscriber<T>, public Invoker {
		ArrayQueue<T, S> _t;
		std::function<void(const T &)> _func;
		Thread *_thread = 0;
		int next(int index) { return ++index % S; }
		T _lastValue;

	public:
		Sink() {
			_func = [&](const T &t) { INFO(" no handler attached to this sink "); };
		}
		Sink(std::function<void(const T &)> handler) : _func(handler) {};

		void on(const T &t) {
			if (_thread) {
				if (_t.push(t)) {
					//					WARN(" sink full ");
				} else {
					_thread->enqueue(this);
				}
			} else {
				_func(t);
			}
		}

		virtual void request() { invoke(); }
		void invoke() {
			if (_t.pop(_lastValue)) {
				WARN(" no data ");
			} else {
				_func(_lastValue);
			}
		}

		void async(Thread &thread, std::function<void(const T &)> func) {
			_func = func;
			_thread = &thread;
		}
		void sync(std::function<void(const T &)> func) {
			_thread = 0;
			_func = func;
		}
		T operator()() { return _lastValue; }
};

template <class T, int S>
class QueueSource : public Sink<T, S>, public Source<T> {
		ArrayQueue<T, S> _queue;

	public:
		void on(const T &in) { _queue.push(in); }
		void request() {
			T t;
			if (_queue.pop(t) == 0)
				this->emit(t);
		}
};
//_________________________________________________ Flow ________________
//
template <class IN, class OUT>
class Flow : public Subscriber<IN>, public Source<OUT> {
	public:
		void operator==(Flow<OUT, IN> &flow) {
			this->subscribe(&flow);
			flow.subscribe(this);
		};
};

template <class IN, class OUT>
class LambdaFlow : public Flow<IN, OUT> {
		std::function<int(OUT &, const IN &)> _func;

	public:
		LambdaFlow() {
			_func = [](OUT &out, const IN &in) {WARN("no handler for this flow"); return -1; };
		};
		LambdaFlow(std::function<int(OUT &, const IN &)> func) : _func(func) {};
		void lambda(std::function<int(OUT &, const IN &)> func) { _func = func; }
		virtual void on(const IN &in) {
			OUT out;
			if (_func(out, in)) {
				//				WARN(" conversion failed ");
				return;
			}
			this->emit(out);
		}
		void request() {};
};

template <class IN, class OUT>
Source<OUT> &operator>>(Publisher<OUT> &publisher, Flow<IN, OUT> &flow) {
	publisher.subscribe(&flow);
	return flow;
}

//________________________________________________________________
//
template <class T>
class ValueFlow : public Flow<T, T> {
		T _t;
		bool _pass = false;

	public:
		ValueFlow() {};
		ValueFlow(T t) { _t = std::move(t); }
		void request() { this->emit(_t); }
		void operator=(T t) {
			_t = std::move(t);
			if (_pass)
				this->emit(_t);
		}
		T &operator()() { return _t; }
		void on(const T &in) {
			_t = std::move(in);
			this->emit(_t);
		}
		void pass(bool p) { _pass = p; }
};
//______________________________________ Actor __________________________
//
class Actor {
		Thread &_thread;

	public:
		Actor(Thread &thread) : _thread(thread) {}
		Thread &thread() { return _thread; }
};

#endif // NANOAKKA_H
