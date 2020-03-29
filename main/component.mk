COMPONENT_SRCDIRS := . 
COMPONENT_EXTRA_INCLUDES :=   ../../ArduinoJson/src
#CXXFLAGS =  -fno-rtti -std=gnu++11  -mlongcalls -fdata-sections -ffunction-sections

COMPONENT_ADD_INCLUDEDIRS := . ../../Common .

COMPONENT_ADD_LDFLAGS += -Wl,--gc-sections -fno-rtti

COMMENT := -fno-rtti