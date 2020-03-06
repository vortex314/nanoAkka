#include "NanoAkka.h"

int Thread::_id=0;

void Thread::run()
{
	uint64_t now = Sys::millis();
	uint64_t expTime = now + 5000;
	TimerSource *expiredTimer = 0;
	// find next expired timer if any within 5 sec
	for (auto timer : _timers)
		{
			if (timer->expireTime() < expTime)
				{
					expTime = timer->expireTime();
					expiredTimer = timer;
				}
		}

	if (expiredTimer && (expTime <= now))
		{
			if (expiredTimer)
				expiredTimer->request();
		}
	else
		{
			Invoker *prq;
			uint32_t waitTime = pdMS_TO_TICKS(expTime - now) + 1;
			if (waitTime < 0)
				waitTime = 0;
			uint32_t queueCounter = 0;
			while (xQueueReceive(_workQueue, &prq, (TickType_t)waitTime) == pdTRUE)
				{
					prq->invoke();
					waitTime == pdMS_TO_TICKS(expTime - Sys::millis());
					if (waitTime > 0)
						waitTime = 0;
				}
			if (expiredTimer)
				{
					expiredTimer->request();
				}
		}
}
