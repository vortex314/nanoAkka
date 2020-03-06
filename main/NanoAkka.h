#ifndef NANOAKKA_H
#define NANOAKKA_H

#include <functional>
#include <string>
#include <vector>
#include <atomic>


#define FREERTOS
#include <FreeRTOS.h>
#include "freertos/task.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>

#ifdef ESP32_IDF
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#define PRO_CPU 0
#define APP_CPU 1
#endif

#include <Sys.h>
#include <Log.h>

// INTERFACES nanoAkka

class Invoker
{
public :
	virtual void invoke()=0;
};

template <class T>
class Subscriber
{
public:
	virtual void on(const T& t)=0;
	virtual ~Subscriber() {};
};

template <class T>
class Publisher
{
public:
	virtual void subscribe(Subscriber<T>* listener)=0;
};

class Requestable
{
	virtual void request()=0;
};

//template <class IN,class OUT>
//class Flow : public Subscriber<IN>,public Publisher<OUT> {};

// UTIL

template <class T,int SIZE>
class CircularBuffer
{
	T _buffer[SIZE];
	T _lastValue;
	std::atomic_int _readPtr;
	std::atomic_int _writePtr;
public:
	CircularBuffer()
	{
		clear();
	}

	int atomicReadIdx()
	{
		int expected;
		do
			{
				expected=nextIndex(_readPtr);
			}
		while ( !_readPtr.compare_exchange_weak(expected,nextIndex(_readPtr),std::memory_order_release,std::memory_order_relaxed));
		return expected;
	}
	int atomicWriteIdx()
	{
		int expected;
		do
			{
				expected=nextIndex(_writePtr);
			}
		while ( !_writePtr.compare_exchange_weak(expected,nextIndex(_writePtr),std::memory_order_release,std::memory_order_relaxed));
		return expected;
	}


	void clear()
	{
		_readPtr=0;
		_writePtr = 1;
	}
	int nextIndex(int index)
	{
		return (index+1)%SIZE;
	}
	int write(const T& t)
	{
		if ( nextIndex(_writePtr) == _readPtr) return -1;
		int writePtr = atomicWriteIdx();
		if ( writePtr == _readPtr) return -1;
		_buffer[writePtr]=t;
		return 0;
	}
	int read(T& t)
	{
		if ( nextIndex(_readPtr) == _writePtr ) return -1;
		int readPtr = atomicReadIdx();
		if ( readPtr == _writePtr) return -1;
		t = _buffer[readPtr];
		return 0;
	}
	int size()
	{
		if ( _writePtr < _readPtr) return _writePtr+SIZE-_readPtr-1;
		return _writePtr-_readPtr-1;
	}
	int space()
	{
		return SIZE-size();
	}
	T lastValue()
	{
		if ( size())
			{
				read(_lastValue);
			}
		return _lastValue;
	}
};


// STREAMS
class TimerSource;

class Thread
{
#ifdef FREERTOS
	QueueHandle_t _workQueue = 0;
#endif
	std::vector<TimerSource *> _timers;
	static int _id;
	std::string _name;
public :
	Thread()
	{
		string_format(_name,"thread-%d",_id++);
		_workQueue = xQueueCreate(20, sizeof(Invoker *));
	}
	void start()
	{
		xTaskCreatePinnedToCore([](void* task)
		{
			((Thread*)task)->run();
		}, _name.c_str(), 20000, this, 17, NULL, PRO_CPU);
	}
	int enqueue(Invoker* invoker)
	{
		if (_workQueue)
			if (xQueueSend(_workQueue, &invoker, (TickType_t)0) != pdTRUE)
				{
					WARN(" queue overflow ");
					return ENOBUFS;
				}
		return 0;
	};
	int enqueueFromIsr(Invoker* invoker)
	{
		if (_workQueue)
			{
				if (xQueueSendFromISR(_workQueue, &invoker, (TickType_t)0) != pdTRUE)
					{
						//  WARN("queue overflow"); // cannot log here concurency issue
						return ENOBUFS;
					}
			}
		return 0;
	};
	void run();


	void addTimer(TimerSource *ts)
	{
		_timers.push_back(ts);
	}

};

template <class IN,class OUT>
class Flow;


template <class T>
class Source : public Publisher<T>,public Requestable
{
	std::vector<Subscriber<T>*> _listeners;
public :
	void subscribe(Subscriber<T>* listener)
	{
		_listeners.push_back(listener);
	}
	void emit(const T& t)
	{
		for( Subscriber<T>* l:_listeners)
			{
				l->on(t);
			}
	}
	void operator>>(Subscriber<T>& listener)
	{
		subscribe(&listener);
	}
	void operator>>(Subscriber<T>* listener)
	{
		subscribe(listener);
	}

	template <class IN,class OUT>
	Source<OUT>& operator>>(Flow<IN,OUT>& flow)
	{
		subscribe(&flow);
		return flow;
	}

	/*	template <class IN,class OUT>
			Source<OUT>& operator>>(SinkSource<IN,OUT>& ss)
			{
				subscribe(&ss);
				return ss;
			}*/
	template <class IN,class OUT>
	Source<OUT>& operator>>(std::function<OUT&(IN&)> func)
	{
		Source<OUT> source = *(new Source<OUT>());
		return source;
	}

};

template <class T>
class LambdaSource  : public Source<T>
{
	std::function<T()> _handler;

public:
	LambdaSource(std::function<T()> handler) : _handler(handler) {};
	void request()
	{
		this->emit(_handler());
	}
};

template <class T>
class ValueSource : public Source<T>
{
	T _t;
public:
	ValueSource() {};
	ValueSource(T t)
	{
		_t=t;
	}
	void request()
	{
		this->emit(_t);
	}
	void operator=(T t)
	{
		_t=t;
		this->emit(_t);
	}
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
//__________________________________________________________________________
class TimerMsg
{
public:
	uint32_t id;
};

class TimerSource : public Source<TimerMsg>
{
	uint32_t _interval;
	bool _repeat;
	uint64_t _expireTime;
	uint32_t _id;

public:
	bool running = true;
	TimerSource(Thread& thr,int id, uint32_t interval, bool repeat)
	{
		_id = id;
		_interval = interval;
		_repeat = repeat;
		if ( repeat) start();
		thr.addTimer(this);
	}
	void attach(Thread& thr)
	{
		thr.addTimer(this);
	}
	void start()
	{
		running = true;
		_expireTime = Sys::millis() + _interval;
	}
	void stop()
	{
		running = false;
	}
	void interval(uint32_t i)
	{
		_interval = i;
	}
	void request()
	{
		//       INFO("[%X] %d request() ",this,_id);
		if (running)
			{
				if (Sys::millis() >= _expireTime)
					{
						_expireTime += _interval;
						TimerMsg tm= {_id};
						this->emit(tm);
					}
			}
		else
			{
				WARN(" timer not running ");
			}
	}
	uint64_t expireTime()
	{
		return _expireTime;
	}
	inline uint32_t interval()
	{
		return _interval;
	}
	void subscribeOn(Thread &thread)
	{
		thread.addTimer(this);
	}
	void observeOn(Thread &thread)
	{
		thread.addTimer(this);
	}
};


template < class T,int S>
class Sink : public Subscriber<T>, public Invoker
{
	CircularBuffer<T,S> _t;
	std::function<void(const T&)> _func;
	Thread* _thread=0;
	int next(int index)
	{
		return ++index % S;
	}
public:
	Sink()
	{
		_func=[](const T& t) {};
	}
	Sink(std::function<void(const T& )> handler) : _func(handler) {};

	void on(const T& t)
	{
		if ( _thread )
			{
				_t.write(t);
				_thread->enqueue(this);
			}
		else
			{
				_func(t);
			}
	}

	virtual void request()
	{
		invoke();
	}
	void invoke()
	{
		if ( _t.size())
			{
				T t;
				_t.read(t);
				_func(t);
			}
	}
	void async ( Thread& thread,std::function<void(const T&)> func)
	{
		_func=func;
		_thread=&thread;
	}
	void sync(std::function<void(const T&)> func)
	{
		_thread=0;
		_func=func;
	}

	T operator()()
	{
		return _t.lastValue();
	}
};


template <class IN,class OUT>
class SinkSource : public Sink<IN,2>,public Source<OUT>
{
public :
	virtual OUT& convert(IN& in)
	{
		return *(new OUT());
	}
	void on(IN& in)
	{
		this->emit(convert(in));
	}
};
//______________________________________ Actor __________________________
class Actor
{
	Thread& _thread;
public:
	Actor(Thread& thread) : _thread(thread) {}
	Thread& thread()
	{
		return _thread;
	}
};

#endif // NANOAKKA_H
