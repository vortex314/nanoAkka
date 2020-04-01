# nanoAkka
## TL;DR
- Running on ESP32, ESP8266, LM4F120 and probably on any Arduino
- Very lightweight : mostly a 500 lines header
#### Publish data on change
```C++
ValueSource<int> x;
x >> mqtt.toTopic<int>("src/x");
x=1; // This emits a MQTT message topic:src/x, message=1
```
#### Subscribe /Publish to MQTT data
```C++
servo.KI == mqtt.topic<float>("servo/KI");
//publishing on MQTT will change the value, changing the value will publish
// topic : src/host/servo/KI : reports the changes
// publish to topic dst/host/servo/KI will change the value
```
#### Activate MQTT when Wifi is connected and blink a LED slow, report signal strength of WiFi
```C++
Thread mqttThread("mqtt");
Wifi wifi(mqttThread);
MqttWifi mqtt(mqttThread);
wifi.init();
mqtt.init();
wifi.connected >> mqtt.wifiConnected;
mqtt.connected >> led.blinkSlow;
wifi.rssi >> mqtt.toTopic<int>("wifi/rssi");
```
#### Report current boot time to MQTT
```C++
LambdaSource<uint64_t> systemUptime([](){return Sys::millis();});
systemUptime >> mqtt.toTopic<uint64_t>("system/upTime");
poller(systemUptime)(systemHeap)(systemHostname)(systemBuild)(systemAlive);
```

## History
In microAkka I tried to implement the LightBend Akka framework almost literately , with the same verbs and concepts of : Actor , ActorRef, Dispatcher, .. But using messagetype detection in C++ required too much overhead in code and slowness. 

Later on I worked with Akka streams and reactive programming concepts, which led to the project esp32-streams. There I still struggled with the combination of streams and threads. 

So here is nanoAkka ! I called it nano because it's way smaller than microAkka or esp32-streams. It combines concepts of Akka, Akka Streams and sometimes feels like I'm wiring chips together to build a system. The code becomes very condense, much easier to read to reason about. 
<!--stackedit_data:
eyJoaXN0b3J5IjpbMjEyOTkwMDY3OV19
-->