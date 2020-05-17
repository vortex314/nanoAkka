#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
# ex. : touch main.cpp
#       make flash term DEFINE="-DPROGRAMMER=1 -DHOSTNAME=prog"
#
DEFINE ?= -DAAAAA=BBBBBB
PROJECT_NAME := nanoAkka
TTY ?= USB0
DEFINE ?= -DAAAAA=BBBBBB
SERIAL_PORT ?= /dev/tty$(TTY)
ESPPORT = $(SERIAL_PORT)
SERIAL_BAUD = 115200
ESPBAUD = 921600
IDF_PATH ?= /home/lieven/esp/esp-idf
WORKSPACE := /home/lieven/workspace
DEFINES := -DWIFI_SSID=${SSID} -DWIFI_PASS=${PSWD}  -DESP32_IDF=1 $(DEFINE) -DMQTT_HOST=limero.ddns.net -DMQTT_PORT=1883 

CPPFLAGS +=  $(DEFINES)  -I../Common 
CPPFLAGS +=  -I$(WORKSPACE)/ArduinoJson/src -I $(IDF_PATH)/components/freertos/include/freertos 

CXXFLAGS +=  $(DEFINES)  -I../Common 
CXXFLAGS +=  -I$(WORKSPACE)/ArduinoJson/src -I $(IDF_PATH)/components/freertos/include/freertos 
CXXFLAGS +=  -fno-rtti -ffunction-sections -fdata-sections -std=c++11 -fno-exceptions -lstdc++

EXTRA_COMPONENT_DIRS = mqtt esp-mqtt 
# LDFLAGS += -Wl,-latomic 
# -Wl,--gc-sections

include $(IDF_PATH)/make/project.mk

COMPASS :
	touch main/main.cpp
	make DEFINE=" -DCOMPASS=2 -DHOSTNAME=compass"

STEPPER :
	touch main/main.cpp
	make DEFINE=" -DSTEPPER=1 -DCOMPASS=2 -DHOSTNAME=drive"
	
STEPPER_SERVO :
	touch main/main.cpp
	make DEFINE=" -DSTEPPER_SERVO=1  -DHOSTNAME=drive"
	
HWTIMER :
	touch main/main.cpp
	make DEFINE=" -DHWTIMER=1  -DHOSTNAME=timer"

GPIO :
	touch main/main.cpp
	make DEFINE=" -DGPIO_TEST -DHOSTNAME=gpio"

GPS_WIFI:
	touch main/main.cpp
	make DEFINE=" -DGPS=2 -DUS=1 -DHOSTNAME=gps -DMQTT_HOST=limero.ddns.net " 
	
GPS_SERIAL:
	touch main/main.cpp
	make DEFINE=" -DGPS=2 -DUS=1 -DHOSTNAME=gps -DMQTT_SERIAL " 


REMOTE :
	touch main/main.cpp
	make DEFINE="-DREMOTE=1 -DHOSTNAME=remote -DMQTT_HOST=limero.ddns.net" 
	
DRIVE :
	touch main/main.cpp
	make DEFINE="-DMOTOR=1 -DSTEPPER_SERVO=2 -DHOSTNAME=drive"

MOTOR_WIFI :
	touch main/main.cpp
	make DEFINE="-DMOTOR=1 -DHOSTNAME=drive"
	
MOTOR_SERIAL :
	touch main/main.cpp
	make DEFINE="-DMOTOR=1 -DMQTT_SERIAL -DHOSTNAME=drive"
	
SERVO_WIFI :
	touch main/main.cpp
	make DEFINE="-DSERVO=1 -DHOSTNAME=drive"
	
DRIVE_WIFI :
	touch main/main.cpp
	make DEFINE="-DMOTOR=2 -DSERVO=1 -DHOSTNAME=drive" 
	
DRIVE_SERIAL :
	touch main/main.cpp
	make DEFINE="-DMOTOR=2 -DSERVO=1 -DHOSTNAME=drive -DMQTT_SERIAL" 
	
DRIVE_SERIAL2 :
	touch main/main.cpp
	make DEFINE="-DMOTOR=2 -DHOSTNAME=drive2 -DMQTT_SERIAL"
	
TAG_WIFI :
	touch main/main.cpp
	make DEFINE="-DDWM1000_TAG=2 -DHOSTNAME=tag"

TAG :
	touch main/main.cpp
	make DEFINE="-DDWM1000_TAG=2 -DHOSTNAME=tag"
	
DIGITAL_COMPASS :
	touch main/main.cpp
	make DEFINE="-DDIGITAL_COMPASS=1"
	
SERIAL :
	touch main/main.cpp
	make DEFINE="-DMQTT_SERIAL" 

term:
	rm -f $(TTY)_minicom.log
	minicom -D $(SERIAL_PORT) -b $(SERIAL_BAUD) -C $(TTY)_minicom.log
