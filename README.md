thing
=====

Thing is an occupancy sensor designed to exist within the Internet of Things

The development of the Thing system is part of Ash's honours thesis: [github repo](https://github.com/atyndall/honours).

Read more about the composition of the Thing system in section 2.3 of [Ash's thesis](https://github.com/atyndall/honours/blob/master/thesis/thesis.pdf?raw=true)

The code in the "node" section runs on the Arduino platform, interfacing to a [MLX90620](http://www.melexis.com/Infrared-Thermometer-Sensors/Infrared-Thermometer-Sensors/MLX90620-776.aspx).

The code in the "coordinator" section runs on the Raspberry Pi platform, interfacing with multiple "nodes" over a WPAN, and the broader internet via Ethernet or WiFi.