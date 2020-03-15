#ifndef REMOTE_H
#define REMOTE_H
#include <NanoAkka.h>
#include <Hardware.h>

class ButtonStruct {
	public:
		DigitalIn& digitalIn;
		ValueSource<bool>& valueSource;
		ButtonStruct(DigitalIn& dIn,ValueSource<bool>& vs) : digitalIn(dIn),valueSource(vs) {
		}
} ;

class Remote : public Actor {
		DigitalOut& _ledLeft;
		DigitalOut& _ledRight;
		DigitalIn& _buttonLeft;
		DigitalIn& _buttonRight;
		ADC& _adcLeft;
		ADC& _adcRight;
		TimerSource _measureTimer;
		ButtonStruct _leftButton,_rightButton;
	public:
		Sink<bool,2> ledLeft,ledRight;
		ValueSource<bool> buttonLeft,buttonRight;
		ValueSource<int> potLeft,potRight;
		Remote(Thread& thread);
		~Remote();
		void init();
		static void buttonChange(void* pButton);
};

#endif // REMOTE_H
