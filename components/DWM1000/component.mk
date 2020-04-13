#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
# COMPONENT_ADD_INCLUDEDIRS=.
# CXXFLAGS +="-DESP32_IDF=1"
# Component makefile for extras/DWM1000

# expected anyone using bmp driver includes it as 'DWM1000/MQTT*.h'
INC_DIRS += $(DWM1000_ROOT) ../Common $(DWM1000_ROOT)/../../main $(DWM1000_ROOT)/../../../microAkka/src 

# args for passing into compile rule generation
DWM1000_SRC_DIR =  $(DWM1000_ROOT)
DWM1000_CFLAGS= -DESP_OPEN_RTOS 
DWM1000_CXXFLAGS= -ffunction-sections -fdata-sections  -fno-threadsafe-statics -std=c++11 -fno-rtti -lstdc++ -fno-exceptions -DPSWD=${PSWD} -DSSID=${SSID} -DESP_OPEN_RTOS
DWM1000_INC_DIR= ../etl/src ../../main

# $(eval $(call component_compile_rules,DWM1000))

COMPONENT_ADD_INCLUDEDIRS := . 