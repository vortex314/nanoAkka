#include "NanoAkka.h"

NanoStats stats;
/*
 _____ _                        _
|_   _| |__  _ __ ___  __ _  __| |
  | | | '_ \| '__/ _ \/ _` |/ _` |
  | | | | | | | |  __/ (_| | (_| |
  |_| |_| |_|_|  \___|\__,_|\__,_|
*/
int Thread::_id=0;

void Thread::createQueue()
{
    _workQueue = xQueueCreate(20, sizeof(Invoker *));
    if ( _workQueue== NULL) WARN("Queue creation failed ");
}

void Thread::start()
{
    xTaskCreate([](void* task) {
        ((Thread*)task)->run();
    }, _name.c_str(), 20000, this, tskIDLE_PRIORITY, NULL);
    /*
    	xTaskCreatePinnedToCore([](void* task) {
    		((Thread*)task)->run();
    	}, _name.c_str(), 20000, this, 17, NULL, PRO_CPU);*/
}

int Thread::enqueue(Invoker* invoker)
{
//	INFO("Thread '%s' >>> '%s'",_name.c_str(),symbols(invoker));
    if (_workQueue)
        if (xQueueSend(_workQueue, &invoker, (TickType_t)0) != pdTRUE) {
            stats.threadQueueOverflow++;
            WARN("Thread '%s' queue overflow [%X]",_name.c_str(),invoker);
            return ENOBUFS;
        }
    return 0;
};
int Thread::enqueueFromIsr(Invoker* invoker)
{
    if (_workQueue) {
        if (xQueueSendFromISR(_workQueue, &invoker, (TickType_t)0) != pdTRUE) {
            //  WARN("queue overflow"); // cannot log here concurency issue
            stats.threadQueueOverflow++;
            return ENOBUFS;
        }
    }
    return 0;
};

void Thread::run()
{
    INFO("Thread '%s' started ",_name.c_str());
    uint32_t noWaits=0;
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
        int32_t waitTime = (expTime-now); // ESP_OPEN_RTOS seems to double sleep time ?

//		INFO(" waitTime : %d ",waitTime);
        if ( noWaits % 1000 == 999 ) WARN(" noWaits : %d in thread %s waitTime %d ",noWaits,_name.c_str(),waitTime);
        if ( waitTime > 0 ) {
            Invoker *prq;
            TickType_t tickWaits  = pdMS_TO_TICKS(waitTime) ;
            if ( tickWaits==0) noWaits++;
            if (xQueueReceive(_workQueue, &prq, tickWaits) == pdTRUE) {
                uint64_t start=Sys::millis();
                prq->invoke();
                uint32_t delta=Sys::millis()-start;
                if ( delta > 50 ) WARN("Invoker [%X] slow %d msec invoker on thread '%s'.",prq,delta,_name.c_str());
            } else {
                noWaits=0;
            }
        } else {
            noWaits++;
            if (expiredTimer) {
                if ( -waitTime>100 ) INFO("Timer[%X] already expired by %u msec on thread '%s'.",expiredTimer,-waitTime,_name.c_str());
                uint64_t start=Sys::millis();
                expiredTimer->request();
                uint32_t deltaExec=Sys::millis()-start;
                if ( deltaExec > 50 ) WARN("Timer [%X] request slow %d msec on thread '%s'",expiredTimer,deltaExec,_name.c_str());
            }
        }
    }
}
