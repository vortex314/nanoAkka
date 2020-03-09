#ifndef NANOAKKA_H
#define NANOAKKA_H

#include <functional>
#include <string>
#include <vector>
#include <atomic>
#include <unordered_map>


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

class Map
{
	std::unordered_map<uint32_t,std::string> _symbols;
	std::string fmt;

public:
	void add(void* address,const char* name)
	{
		_symbols.emplace((uint32_t)address,name);
	}
	void add(void* base,void* member,const char* name)
	{
		std::string n = find(base);
		n+=".";
		n+=name;
		_symbols.emplace((uint32_t)member,n);
	}
	const char*  find(void* address)
	{
		auto f =   _symbols.find((uint32_t)address);
		if ( f==_symbols.end())
			{
				string_format(fmt,"%X",address);
				return fmt.c_str();
			}
		return f->second.c_str();
	}
	const char* operator()(void* address)
	{
		return find(address);
	}
};
extern Map symbols;

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

typedef void* voidPtr;

class PointerQueue
{
	voidPtr* _buffer;
	int _size;
	std::atomic_uint32_t _readPtr;
	std::atomic_uint32_t _writePtr;
public :
	PointerQueue(int size)
	{
		_size=size;
		_buffer = (voidPtr*)malloc(_size*(sizeof(voidPtr)));
		_readPtr=0;
		_writePtr=0;
	}

	int read(voidPtr* ptr)
	{
		uint64_t readPtr = _readPtr.fetch_add(1, std::memory_order_relaxed);
		if ( readPtr >= _writePtr )
			{
				_readPtr.fetch_sub(1,std::memory_order_relaxed);
				return -1;
			}
		*ptr = _buffer[readPtr & _size];
		return 0;
	}

	int write(voidPtr ptr)
	{
		uint64_t writePtr = _writePtr.fetch_add(1,std::memory_order_relaxed);
		if ( writePtr >= _readPtr + _size)
			{
				_writePtr.fetch_sub(1,std::memory_order_relaxed);
				return -1;
			}
		_buffer[writePtr & _size]=ptr;
		return 0;
	}
	int size( )
	{
		return _writePtr-_readPtr;
	}
	int space()
	{
		return _size-size();
	}

};

template <class T,int SIZE>
class CircularBuffer
{
	PointerQueue _queue;
public:
	CircularBuffer() : _queue(2)
	{
	}

	int write(const T& t)
	{
		T* pt =  new T;
		*pt =  t;
		if ( _queue.write(pt) )
			{
				delete pt;
				return -1;
			}
		return 0;
	}
	int read(T& t)
	{
		T* pt;
		if ( _queue.read((voidPtr*)&pt) ) return -1;
		t = std::move(*pt);
		delete pt;
		return 0;
	}
	int size()
	{
		return _queue.size();
	}
	int space()
	{
		return _queue.space();
	}

};


// STREAMS
class TimerSource;

class Thread
{
#ifdef FREERTOS
	QueueHandle_t _workQueue = 0;
	void createQueue() ;
#endif
	std::vector<TimerSource *> _timers;
	static int _id;
	std::string _name;
public :
	Thread(const char *name)
	{
		INFO("Thread '%s' ctor.",name);
		_name=name;
		createQueue();
	}
	Thread()
	{
		string_format(_name,"thread-%d",_id++);
		INFO("Thread '%s' ctor.",_name.c_str());
		createQueue();
	}
	void start();
	int enqueue(Invoker* invoker);
	int enqueueFromIsr(Invoker* invoker) ;
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
	T& operator()()
	{
		return _t;
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
	uint32_t _interval=UINT32_MAX;
	bool _repeat=false;
	uint64_t _expireTime=UINT64_MAX;
	uint32_t _id=0;

public:
	TimerSource(Thread& thr,int id, uint32_t interval, bool repeat)
	{
		INFO("Timer[%X] %d,%u ",this,id,interval);
		_id = id;
		_interval = interval;
		_repeat = repeat;
		if ( repeat) start();
		thr.addTimer(this);
	}
	TimerSource(Thread& thr) : TimerSource(thr,0,UINT32_MAX,false)
	{
		thr.addTimer(this);
	}

	TimerSource()
	{
		_expireTime=Sys::now()+_interval;
	};

	void attach(Thread& thr)
	{
		thr.addTimer(this);
	}
	void start()
	{
		_expireTime = Sys::millis() + _interval;
	}
	void stop()
	{
		_expireTime=UINT64_MAX;
	}
	void interval(uint32_t i)
	{
		_interval = i;
	}
	void request()
	{
		if (Sys::millis() >= _expireTime)
			{
				INFO("Timer[%X] '%s' %d triggered.",this,symbols(this),_interval);
				if ( _repeat )
					_expireTime += _interval;
				else
					_expireTime += UINT32_MAX;
				TimerMsg tm= {_id};
				this->emit(tm);
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
		_func=[&](const T& t)
		{
			INFO(" no handler attached to this sink [%X]",this);
		};
	}
	Sink(std::function<void(const T& )> handler) : _func(handler) {};

	void on(const T& t)
	{
		if ( _thread )
			{
				if ( _t.write(t) )
					{
						ERROR(" sink full %s ",symbols(this));
					}
				else
					{
//					INFO(" async call");
						_thread->enqueue(this);
					}
//				INFO("enqueue %s",symbols(this));
			}
		else
			{
				INFO(" sync call %s ",symbols(this));
				_func(t);
			}
	}

	virtual void request()
	{
		invoke();
	}
	void invoke()
	{
		T t;
		if ( _t.read(t) )
			{
				WARN(" no data ");
			}
		else
			{
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


template <class T,int S>
class QueueSource : public Sink<T,S>,public Source<T>
{
	CircularBuffer<T,S> _queue;
public :
	void on(const T& in)
	{
		if (_queue.space()) _queue.write(in);
	}
	void request()
	{
		if ( _queue.size())
			{
				T t;
				_queue.read(t);
				this->emit(t);
			}
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
