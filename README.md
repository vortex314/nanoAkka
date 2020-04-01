# nanoAkka
## TL;DR
Some code 
````

````
## History
In microAkka I tried to implement the LightBend Akka framework almost literately , with the same verbs and concepts of : Actor , ActorRef, Dispatcher, .. But using messagetype detection in C++ required too much overhead in code and slowness. 

Later on I worked with Akka streams and reactive programming concepts, which led to the project esp32-streams. There I still struggled with the combination of streams and threads. 

So here is nanoAkka ! I called it nano because it's way smaller than microAkka or esp32-streams. It combines concepts of Akka, Akka Streams and sometimes feels like I'm wiring chips together to build a system. The code becomes very condense, much easier to read to reason about. 
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTI2ODQyNjkxMl19
-->