/*
 * LogIsr.h
 *
 *  Created on: Apr 23, 2019
 *      Author: lieven
 */

#ifndef MAIN_LOGISR_H_
#define MAIN_LOGISR_H_

#include <NanoAkka.h>
#include <CircBuf.h>
#include <Log.h>
#include <stdlib.h>
#include <stdio.h>

class LogIsr: public Actor
{
    CircBuf _buffer;
    char _strBuffer[256];
    static LogIsr* _me;
public:
    TimerSource logTimer;
    Sink<std::string,20> logs;
    static void log(const char* str);
    static void log(const char* file, uint32_t line, const char* fmt, ...);
    static bool _debug;
    LogIsr(Thread& thr);
    ~LogIsr();
    void init();
};

#define INFO_ISR(fmt, ...)   LogIsr::log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);
#define ERROR_ISR(fmt, ...) LogIsr::log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);
#define WARN_ISR(fmt, ...)  LogIsr::log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);
#define FATAL_ISR(fmt, ...) LogIsr.log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);

#define DEBUG_ISR(fmt, ...)                                                        \
    if (LogIsr::_debug) LogIsr::log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);
#define TRACE_ISR(fmt, ...)                                                        \
    if (LogIsr::_debug) LogIsr::log( __SHORT_FILE__, __LINE__,  fmt, ##__VA_ARGS__);

#endif /* MAIN_LOGISR_H_ */
