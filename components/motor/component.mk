#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
# COMPONENT_ADD_INCLUDEDIRS=.
# CXXFLAGS +="-DESP32_IDF=1"
# Component makefile for extras/Common

#ESP32 COMPONENT.MK
COMPONENT_SRCDIRS := .
COMPONENT_ADD_INCLUDEDIRS := . ../main ../../ArduinoJson ../../Common

