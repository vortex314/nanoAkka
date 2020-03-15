#ifndef NANOAKKA_H
#define NANOAKKA_H

#include <atomic>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

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
#else

#endif

#ifdef ARDUINO
#include <Arduino.h>
#include <printf.h>
#include <stdarg.h>
typedef String NanoString;
#define INFO(fmt, ...)                                                         \
	{                                                                            \
		char line[256];                                                            \
		int len = snprintf(line, sizeof(line), "I %06lld | %.12s:%.3d | ",         \
		                   Sys::millis(), __FILE__, __LINE__);                     \
		snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__);    \
		Serial.println(line);                                                      \
	}
#define WARN(fmt, ...)                                                         \
	{                                                                            \
		char line[256];                                                            \
		int len = snprintf(line, sizeof(line), "W %06lld | %.12s:%.3d | ",         \
		                   Sys::millis(), __FILE__, __LINE__);                     \
		snprintf((char *)(line + len), sizeof(line) - len, fmt, ##__VA_ARGS__);    \
		Serial.println(line);                                                      \
	}
#else
#include <Log.h>
#endif
#include <Sys.h>

// INTERFACES nanoAkka

template <class T> class AbstractQueue {
	public:
		virtual int pop(T &t) = 0;
		virtual int push(const T &t) = 0; // const to be able to do something like
		// push({"topic","message"});
};

class Invoker {
	public:
		virtual void invoke() = 0;
};

template <class T> class Subscriber {
	public:
		virtual void on(const T &t) = 0;
		virtual ~Subscriber() {};
};

template <class T> class Publisher {
	public:
		virtual void subscribe(Subscriber<T> *listener) = 0;
};

class Requestable {
	public:
		virtual void request() = 0;
};
#define BUSY (1 << 31)
// template <class IN,class OUT>
template <class T, int SIZE> class ArrayQueue : public AbstractQueue<T> {
		T _array[SIZE];
		std::atomic<uint32_t> _readPtr;
		std::atomic<uint32_t> _writePtr;
		inline uint32_t next(uint32_t idx) {
			return (idx+1)%SIZE;
		}

	public:
		ArrayQueue() { _readPtr = _writePtr = 0; }
		int push(const T &t) {
			uint32_t expected = _writePtr;
			uint32_t desired = next(expected);
			if (desired == _readPtr) {
//				printf(__FILE__ ":%d FULL\n",__LINE__); // aborts in isr
				return ENOBUFS;
			}
			if (expected & BUSY) {
//				printf(__FILE__ ":%d BUSY\n",__LINE__);
				return ENODATA;
			}
			desired |= BUSY;
			if (_writePtr.compare_exchange_weak(expected, desired,
			                                    std::memory_order_relaxed)) {
				expected = desired;
				_array[desired % SIZE] = std::move(t);
				desired &= ~BUSY;
				if (!_writePtr.compare_exchange_weak(expected, desired,
				                                     std::memory_order_relaxed)) {
					WARN("remove busy failed");
				}
				return 0;
			}
			WARN("writePtr update failed");
			return -1;
		}

		int pop(T &t) {
			uint32_t expected = _readPtr.load();
			uint32_t desired = next(expected);
			if (expected == _writePtr) {
				WARN("EMPTY");
				return ENOBUFS;
			}
			if (expected & BUSY) {
				WARN("BUSY");
				return ENODATA;
			}
			desired |= BUSY;
			if (_readPtr.compare_exchange_weak(expected, desired,
			                                   std::memory_order_relaxed)) {
				expected = desired;
				t = std::move(_array[desired % SIZE]);
				desired &= ~BUSY;
				if (!_readPtr.compare_exchange_weak(expected, desired,
				                                    std::memory_order_relaxed)) {
					WARN("remove busy failed");
				}
				return 0;
			}
			WARN("readPtr update failed");
			return -1;
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
		void addTimer(TimerSource *ts) { _timers.push_back(ts); }
};

template <class IN, class OUT> class Flow;

template <class T> class Source : public Publisher<T>, public Requestable {
		std::vector<Subscriber<T> *> _listeners;

	public:
		void subscribe(Subscriber<T> *listener) { _listeners.push_back(listener); }
		void emit(const T &t) {
			for (Subscriber<T> *l : _listeners) {
				l->on(t);
			}
		}
		void operator>>(Subscriber<T> &listener) { subscribe(&listener); }
		void operator>>(Subscriber<T> *listener) { subscribe(listener); }

		template <class IN, class OUT> Source<OUT> &operator>>(Flow<IN, OUT> &flow) {
			subscribe(&flow);
			return flow;
		}

		/*	template <class IN,class OUT>
		                Source<OUT>& operator>>(SinkSource<IN,OUT>& ss)
		                {
		                        subscribe(&ss);
		                        return ss;
		                }*/
		template <class IN, class OUT>
		Source<OUT> &operator>>(std::function<OUT &(IN &)> func) {
			Source<OUT> source = *(new Source<OUT>());
			return source;
		}
};

template <class T> class LambdaSource : public Source<T> {
		std::function<T()> _handler;

	public:
		LambdaSource(std::function<T()> handler) : _handler(handler) {};
		void request() { this->emit(_handler()); }
};

template <class T> class ValueSource : public Source<T> {
		T _t;
	public:
		ValueSource() {};
		ValueSource(T t) { _t = t; }
		void request() { this->emit(_t); }
		void operator=(T t) {_t = t; this->emit(_t);}
		T &operator()() { return _t; }
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
//______________________________________ TimerSource
//____________________________________
class TimerMsg {
	public:
		uint32_t id;
};

class TimerSource : public Source<TimerMsg> {
		uint32_t _interval = UINT32_MAX;
		bool _repeat = false;
		uint64_t _expireTime = UINT64_MAX;
		uint32_t _id = 0;

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
		void start() { _expireTime = Sys::millis() + _interval; }
		void stop() { _expireTime = UINT64_MAX; }
		void interval(uint32_t i) { _interval = i; }
		void request() {
			if (Sys::millis() >= _expireTime) {
				if (_repeat)
					_expireTime = Sys::millis() + _interval;
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
template <class T, int S> class Sink : public Subscriber<T>, public Invoker {
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
template <class IN, class OUT>
class Flow : public Subscriber<IN>, public Source<OUT> {
	public:
		void on(const IN &in) {
			OUT out;
			if ( convert(out,in) ) {
//				WARN(" conversion failed ");
				return;
			}
			this->emit(out);
			// emit doesn't work as such without this->
			// https://stackoverflow.com/questions/9941987/there-are-no-arguments-that-depend-on-a-template-parameter
		}
		void request() {}
		virtual int convert(OUT & out,const IN &in) = 0;
};

template <class T>
class ValueFlow : public Subscriber<T>, public Source<T> {
		T _t;
	public:
		ValueFlow() {};
		ValueFlow(T t) { _t = t; }
		void request() { this->emit(_t); }
		void operator=(T t) {_t = t; }
		T &operator()() { return _t; }
		void on(const T &in) {
			_t = in;
		}
		void operator==(ValueFlow& vf) { vf >> *this; *this >> vf; }
};

//______________________________________ Actor __________________________
class Actor {
		Thread &_thread;

	public:
		Actor(Thread &thread) : _thread(thread) {}
		Thread &thread() { return _thread; }
};

#endif // NANOAKKA_H
