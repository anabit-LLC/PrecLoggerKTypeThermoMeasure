# PrecLoggerKTypeThermoMeasure
This Arduino sketch is meant to work with Anabit's Precision Logger product which features a 
32 bit ADC with a MUX that allows you to route the ADC to 10 single ended channels or 5 differential 
channels. It also includes features like bipolar measurements, built-in current sources (for resistance 
measurements), programmable gain amp, built-in temperature sensor, and more. It is a great soluiton to make
DC or low frequency measurements from precision sensors and transducers

Product link: 

This example sketch demonstrates how to make a precision K-Type thermocouple measurement. It also utilizes the 
ADC's on board temperatue sensor to serve as the cold jucntion compensation measurement. This sketch shows the 
ADC's ability to accurately resolve and measure micro volt level signals. This sketch uses the awesome 
library that Molorius put together which can be found on Github using the link below. There are more than 
one libary on Github for this ADC so be sure you find the correct one (or just use the link below). The ADS126X
ADC / precision datalogger IC has a lot of different features and settings so it can be a little intimidating to
get started with. But the goal of this example sketch is to make it as painless as possible. To communicate with
the precision logger using this sketch you just need SPI communication pins including chip select. There are 
other pins such as "start' and "ready" that help control ADC conversion timing, but this sketch uses delays to 
handle timing and adjusts delays based on the settings. This sketch uses a structure to hold settings for the 
ADC. Two versions of the structure are used, one for the k-type thermocouple measurement and another for the 
internal temperature sensor measurement. You can change the default thermocouple pins in the structure, but 
we recommend to get started that you leave all others settings to default for your first getting started 
measurements

Link to ADS126X.h library on Github: https://github.com/Molorius/ADS126X

Please report any issue with the sketch to the Anabit forum: https://anabit.co/community/forum/analog-to-digital-converters-adcs

Example code developed by Your Anabit LLC © 2025
Licensed under the Apache License, Version 2.0.
