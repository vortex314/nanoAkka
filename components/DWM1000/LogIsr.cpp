/*
 * LogIsr.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: lieven
 */

#include "LogIsr.h"

LogIsr* LogIsr::_me = 0;
bool LogIsr::_debug = false;

void LogIsr::log(const char* str)
{
    if (_me == 0) return;
    for (int i = 0; i <= strlen(str); i++)
        _me->_buffer.write(str[i]);
}

void LogIsr::log(const char* file, uint32_t line, const char* fmt, ...)
{
    if (_me == 0) return;
    int offset =
        snprintf(_me->_strBuffer, sizeof(_me->_strBuffer) - 1, "ISR >> %s:%d ", file, line);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(&_me->_strBuffer[offset], sizeof(_me->_strBuffer) - offset
                      - 1, fmt, ap);
    _me->_strBuffer[sizeof(_strBuffer) - 1] = 0;
    va_end(ap);
    log(_me->_strBuffer);
}

LogIsr::LogIsr(Thread& thr)
    : Actor(thr),_buffer(256),
      logTimer(thr,50,true)
{
    _me = this;
}

LogIsr::~LogIsr()
{
}

void LogIsr::init()
{
    logTimer >>( [&](const TimerMsg& tm) {
        std::string line;
        while( _buffer.hasData() ) {
            char ch=_buffer.read();
            if ( ch == 0 ) {
                logger.serialLog((char*)line.c_str(),line.length());
                line.clear();
            } else {
                line+=ch;
            }
        }
        if ( line.length()!=0)
            logger.serialLog((char*)line.c_str(),line.length());
    });


}
