#include "NanoAkka.h"

Map symbols;

/*
 _____ _                        _
|_   _| |__  _ __ ___  __ _  __| |
  | | | '_ \| '__/ _ \/ _` |/ _` |
  | | | | | | | |  __/ (_| | (_| |
  |_| |_| |_|_|  \___|\__,_|\__,_|
*/
int Thread::_id=0;

void Thread::createQueue() {
	_workQueue = xQueueCreate(20, sizeof(Invoker *));
	if ( _workQueue== NULL) WARN("Queue creation failed ");
}

void Thread::start() {
	xTaskCreate([](void* task) {
		((Thread*)task)->run();
	}, _name.c_str(), 20000, this, 17, NULL);
	/*
		xTaskCreatePinnedToCore([](void* task) {
			((Thread*)task)->run();
		}, _name.c_str(), 20000, this, 17, NULL, PRO_CPU);*/
}

int Thread::enqueue(Invoker* invoker) {
//	INFO("Thread '%s' >>> '%s'",_name.c_str(),symbols(invoker));
	if (_workQueue)
		if (xQueueSend(_workQueue, &invoker, (TickType_t)0) != pdTRUE) {
			WARN("Thread '%s' queue overflow [%X]",_name.c_str(),invoker);
			return ENOBUFS;
		}
	return 0;
};
int Thread::enqueueFromIsr(Invoker* invoker) {
	if (_workQueue) {
		if (xQueueSendFromISR(_workQueue, &invoker, (TickType_t)0) != pdTRUE) {
			//  WARN("queue overflow"); // cannot log here concurency issue
			return ENOBUFS;
		}
	}
	return 0;
};

void Thread::run() {
	INFO("Thread '%s' started ",_name.c_str());
	while(true) {
		uint64_t now = Sys::millis();
		uint64_t expTime = now + 5000;
		TimerSource *expiredTimer = 0;
		// find next expired timer if any within 5 sec
		for (auto timer : _timers) {
			if (timer->expireTime() < expTime) {
				expTime = timer->expireTime();
				expiredTimer = timer;
			}
		}

		if (expiredTimer && (expTime <= now)) {
			if (expiredTimer) {
				INFO("Timer[%X] already expired",expiredTimer);
				expiredTimer->request();
			}
		} else {
			Invoker *prq;
			uint32_t waitTime = pdMS_TO_TICKS(expTime - now) + 1;
			if (waitTime < 0)
				waitTime = 0;
			uint32_t queueCounter = 0;
			if (xQueueReceive(_workQueue, &prq, (TickType_t)waitTime) == pdTRUE) {
//				INFO("Thread '%s' <<< '%s'",_name.c_str(),symbols(prq));
				prq->invoke();
				waitTime == pdMS_TO_TICKS(expTime - Sys::millis());
				if (waitTime > 0)
					waitTime = 0;
			}
			if (expiredTimer) {
				expiredTimer->request();
			}
		}
	}
}
