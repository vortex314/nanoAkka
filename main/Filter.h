#include <NanoAkka.h>
//__________________________________________________________________________`
//
// MedianFilter : calculates median of N samples
// it will only emit after enough samples
// x : number of samples
//
//__________________________________________________________________________
#include <MedianFilter.h>

template <class T, int x> class Median : public Flow<T, T> {
		MedianFilter<T, x> _mf;

	public:
		Median() {};
		void on(const T &value) {
			_mf.addSample(value);
			if (_mf.isReady()) {
				this->emit(_mf.getMedian());
			}
		};
		void request() { this->emit(_mf.getMedian()); }
};
//__________________________________________________________________________`
//
// Throttle : limits the number of emits per second
// it compares the new and old value, if it didn't change it's not forwarded
// delta : after which time the value is forwarded
//
//__________________________________________________________________________
template <class T> class Throttle : public Flow<T, T> {
		uint32_t _delta;
		uint64_t _nextEmit;
		T _lastValue;

	public:
		Throttle(uint32_t delta) {
			_delta = delta;
			_nextEmit = Sys::millis() + _delta;
		}
		void on(const T &value) {
			uint64_t now = Sys::millis();
			if (now > _nextEmit) {
				this->emit(value);
				_nextEmit = now + _delta;
			}
			_lastValue = value;
		}
		void request() { this->emit(_lastValue); };
};
//__________________________________________________________________________`
//
// emits defaultValue if no new value arrived within timeout
// defaultValue : value pushed
// timeout : after which time the value is forwarded, if no other arrive
//
//__________________________________________________________________________
template <class T> class TimeoutFlow : public Flow<T, T> {
		T _defaultValue;

	public:
		TimerSource timer;
		TimeoutFlow(Thread& thr,uint32_t timeout, T defaultValue)
			: _defaultValue(defaultValue), timer(thr,1, timeout, true) {
			timer >> ([&](TimerMsg tm) { this->emit(_defaultValue); });
		}
		void on(const T &t) {
			this->emit(t);
			timer.start();
		};
		void request() { this->emit(_defaultValue); };
};
