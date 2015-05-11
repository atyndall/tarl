TArL: Thermal Array Library
=====

TArL is a library for interfacing and analysing the features of low-resolution thermal imagery.

The development of the TArL library is part of Ash's honours thesis: [github repo](https://github.com/atyndall/honours).

Read more about the composition of TArL in chapter 3 of [Ash's thesis](https://github.com/atyndall/honours/blob/master/thesis/thesis.pdf?raw=true)

The code in "mlx90620_driver" runs on the Arduino platform, interfacing to a [MLX90620](http://www.melexis.com/Infrared-Thermometer-Sensors/Infrared-Thermometer-Sensors/MLX90620-776.aspx).

The code in the "processing" section runs on the Raspberry Pi platform, interfacing with the MLx90620 over serial.
