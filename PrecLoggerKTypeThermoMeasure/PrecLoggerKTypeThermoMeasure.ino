/********************************************************************************************************
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
********************************************************************************************************/

#include <ADS126X.h>
#include "KTypeTC.h"
#include <SPI.h>

//ADC settings
#define ADC1_BITS (float)2147483647.0 //this is for 31 bits with the 32nd bit being signed
#define ADC2_BITS (float)8388608.0 //this is for 31 bits with the 32nd bit being signed
#define VREF_2048 (float)2.048
#define VREF_INT (float)2.5 //internal ref voltage
#define PGA_BYPASSED   0b101 //created this value that is not in the library to allow me to know if we want amp bypassed
//Volt ref
#define INT_2_5_VREF 0
#define EXT_2048_VREF 1

ADS126X adc; // start the class
uint8_t cSelect = 10; // Arduino pin conected to CS on ADS126X

//Structure that handles settings for ADC 1, these settings are configured for making a thermocouple measurement
struct PrecisionLoggerSettings{ 
    uint8_t posMeasPin = ADS126X_AIN5; //sets positive measurement channel for thermocouple
    uint8_t negMeasPin = ADS126X_AIN4; //sets negative measurement channel for thermocouple
    uint8_t sampleRateSetting = ADS126X_RATE_2_5;    // Sample rate in samples per second
    uint8_t filterType = ADS126X_SINC2; //See library file ADS126X_definitions.h for constnat to set filter type
    uint8_t progGainAmp = ADS126X_GAIN_32; //determines amp gain and allows you to bypass amp (default)
    uint8_t vRef = INT_2_5_VREF; //set internal voltage reference we plan to use
    bool choppingEnabled = true;           // Chopping Mode A (true = enabled)
    bool iDACRotation = false; //only applicable when using internal current sources (IDAC)
    float convTime = 0;
} precLoggerConfig;

//struct to hold internal temp sensor measurement, we will have two structs with ADC settings 
//one holds the settings for the thermo couple measurement and the other the settings for the 
//internal temperature sensor settings (cold junction compensation measurement)
PrecisionLoggerSettings tempSensorConfig;


void setup() {
  //we will set the ADC settings for the internal temperature sensor measurement in the setup function
  //We have to make a temperature measurement of the PCB to do cold junction compensation for the thermocouple measurement calculation
  tempSensorConfig.posMeasPin = ADS126X_TEMP; //set mux to internal temp sensor
  tempSensorConfig.negMeasPin = ADS126X_TEMP; //set mux to internal temp sensor
  tempSensorConfig.sampleRateSetting = ADS126X_RATE_2_5;
  tempSensorConfig.filterType = ADS126X_SINC2; //See library file ADS126X_definitions.h for constnat to set filter type
  tempSensorConfig.progGainAmp = ADS126X_GAIN_1; //for internal temp sensor PGA must be on with gain of 1 as per datasheet
  tempSensorConfig.vRef = INT_2_5_VREF; //set refernce used for measurement
  tempSensorConfig.choppingEnabled = false; //chop mode can not be used for internal temp sensor measurements as per datasheet
  tempSensorConfig.iDACRotation = false; //only applicable when using internal current sources (IDAC)
  tempSensorConfig.convTime = 0; //we will calculate this 

  //lets get ADC conversion time for thermo couple measurement
  precLoggerConfig.convTime = calculateADS1262Timing(precLoggerConfig); //based on filter, sample rate, and chop mode calculated measurement conversion time
  tempSensorConfig.convTime = calculateADS1262Timing(tempSensorConfig); //based on filter, sample rate, and chop mode calculated measurement conversion time

  adc.begin(cSelect); // start ADC library and SPI comm
  //adc.begin();
  
  Serial.begin(115200); //start serial communication
  delay(2000); //delay to get serial monitor up and running
  
  Serial.println("We are making a K-Type Thermocouple measurement,which requires two measurements."); 
  Serial.println("One measurement is of the K-type thermocouple sensor.");
  Serial.println("The second measurement is of the ADC's internal temperature sensor to get the cold reference temperature.");
  Serial.print("ADC conversion time for the thermocouple measurement is: "); Serial.println(precLoggerConfig.convTime);
  Serial.print("ADC conversion time for the internal temperature sensor measurement is: "); Serial.println(tempSensorConfig.convTime);
  
}

void loop() {
  //first we will get thermocouple measurement
  initLogger(precLoggerConfig); //init ADC settings with thermocouple measurement config
  discardFirstReadingDelayConTime(precLoggerConfig.convTime,precLoggerConfig.posMeasPin,precLoggerConfig.negMeasPin); //stsrt ADC, delay, and burn first reading
  delay(precLoggerConfig.convTime); //delay to let next conversion complete
  double measVolt = makeLoggerMeasurement(precLoggerConfig); //make a voltage measurement, scaled on gain settings
  Serial.print("Measured thermocouple voltage: "); Serial.println(measVolt,6); // send voltage through serial
  //next we get the cold reference temperature measurement
  initLogger(tempSensorConfig); //reset ADC settings for temp sensor measurement
  discardFirstReadingDelayConTime(tempSensorConfig.convTime,tempSensorConfig.posMeasPin, tempSensorConfig.negMeasPin); //burn first ADC measurement
  delay(tempSensorConfig.convTime); //delay to let next conversion complete
  double tempSensor = makeLoggerMeasurement(tempSensorConfig); //make a voltage measurement of temp sensor
  Serial.print("Measured internal temperture sensor voltage: "); Serial.println(tempSensor,6); // send voltage through serial
  tempSensor = convertVolt2IntTemperatureVal(tempSensor); //convert volt measurement to temperature
  Serial.print("Temperture internal sensor value: "); Serial.println(tempSensor,2); // send voltage through serial
  //finally we use our two measurements to calculate the measured K-Type thermocouple temperature
  double measMilliVolt = measVolt*1000; //convert voltage measurement to millivolts
  double tHot = KTypeTC::temperature(measMilliVolt,tempSensor); 

    if (isnan(tHot))
        Serial.println(F("Measured temperature value is out of range"));
    else
        Serial.print(F("Measured K-Type Thermocouple temperature: ")), Serial.println(tHot, 2);
}

//This function starts ADC1 (stops it first), delays for conversion time, 
//and clears buffer by disgarding first reading
//Input argument is ADC conversion time
void discardFirstReadingDelayConTime(float cTime, uint8_t posChan, uint8_t negChan) {
  adc.stopADC1(); //stop ADC 1
  adc.startADC1(); // start conversion on ADC1
  delay(cTime); //delay conversion time
  adc.readADC1(posChan,negChan); //burn first reading to clear buffer
}

//converts ADC reading to voltage, voltage is returned as float
//input arguments are measured ADC value, ADC resolution, ADC ref voltage, volt measurement range
double getADCtoVoltage(int32_t aVal,double bits, uint8_t rVoltSetting) {
  double vRef;
  if(rVoltSetting == INT_2_5_VREF) vRef = VREF_INT; //use internal ref value
  else vRef = VREF_2048;
  return (((double)aVal / bits) * vRef);
}


//function to scale voltage value based on the PGA setting and gain
//input argument is measured voltage value and PGA gain we are in
//returns voltage value based on gain we are in
double scaleVoltMeas4Gain(double vVal, uint8_t gain) {
  double gainFactor;    
  if(gain == ADS126X_GAIN_1 || gain == PGA_BYPASSED) gainFactor = 1; 
  else if(gain == ADS126X_GAIN_2) gainFactor = 2;
  else if(gain == ADS126X_GAIN_4) gainFactor = 4; 
  else if(gain == ADS126X_GAIN_8) gainFactor = 8; 
  else if(gain == ADS126X_GAIN_16) gainFactor = 16; 
  else gainFactor = 32; 
  return vVal/gainFactor; 
}

//settings for ADC1 PGA, allows you to enable or disable and sets gain
//input argument true for enable or false for disable.
//second argument sets gain. If disabled, gain setting is ignored
void setADC1PGA(uint8_t gain) {
  if(gain == PGA_BYPASSED) adc.bypassPGA(); //do not use PGA
  else {
    adc.enablePGA(); //use PGA
    adc.setGain(gain); //set gain of PGA
  }
}

//set voltage reference, input argument is vref you want to use
void setsUpADCVoltRef(uint8_t voltRef) {
  adc.enableInternalReference(); //turn on internal reference, needed for IDACs and temperature sensor
  if(voltRef == INT_2_5_VREF) adc.setReference(ADS126X_REF_NEG_INT, ADS126X_REF_POS_INT); //use internal ref
  else adc.setReference(ADS126X_REF_NEG_AIN3, ADS126X_REF_POS_AIN0); //Set this up based on DIP switch settings, AIN3 tied to analog ground and AIN0 to 2.048V ref
}

//this function sets up the ADC settings, some setting are based on the measurement mode we are in
//input arguments are filter type, sample rate, and mode meter is in
void initLogger(PrecisionLoggerSettings loggerSettings) {
  setsUpADCVoltRef(loggerSettings.vRef); //turn on internal ref and select between int and ext
  setADC1PGA(loggerSettings.progGainAmp); //set default range of meter
  delay(10); //give current source and ref time to settle
 // adc.calibrateSelfOffsetADC1();
  if(loggerSettings.choppingEnabled && loggerSettings.iDACRotation) adc.setChopMode(ADS126X_CHOP_3); //11 enables both chop mode and IDAC rotation
  else if(loggerSettings.choppingEnabled)  adc.setChopMode(ADS126X_CHOP_1); //01 enable chop mode, IDAC is off
  else if(loggerSettings.iDACRotation)  adc.setChopMode(ADS126X_CHOP_2); //10 enable IDAC rotation, not chop mode
  else adc.setChopMode(ADS126X_CHOP_0); //00 disable chop mode and IDAC rotation
  adc.setFilter(loggerSettings.filterType); //set ADC1 filter type ADS126X_SINC4
  adc.setRate(loggerSettings.sampleRateSetting); //sets ADC sample rate ADS126X_RATE_2_5
  adc.clearResetBit(); //clear reset bit if it is set
}

//this function takes a voltage measurement from the internal temperature
//and converts it to a temperature measurement in C. This function uses the
//algorithm found in the datasheet page 35:
//Temperature (°C) = [(Temperature Reading (µV) – 122,400) / 420 µV/°C] + 25°C 
//volt values are in micro volts
//note that datasheet says self heating of ADC IC can lead to 0.7 degree error compared to PCB
double convertVolt2IntTemperatureVal(double tVolt) {
  tVolt = tVolt*1000000; //convert volts to micro volts
  return (((tVolt - 122400)/420) + 25);
}

//makes an ADC1 measurement and converts it to a voltage and scales based on PGA gain settings
//input arguments are the positive and negative pins used for measurement
//returns voltage value as double
double makeLoggerMeasurement(PrecisionLoggerSettings loggerSettings) {
  int32_t aDCReading;
  double valMeas;
  uint8_t rTemp;
  aDCReading = adc.readADC1(loggerSettings.posMeasPin,loggerSettings.negMeasPin); // read the voltage
  valMeas = getADCtoVoltage(aDCReading,ADC1_BITS,loggerSettings.vRef); //get voltage from ADC reading
  valMeas = scaleVoltMeas4Gain(valMeas, loggerSettings.progGainAmp); //adjust voltage based on PGA gain setting
  return valMeas;
}

// Function to compute conversion and settling time in milliseconds of ADC1
//input arguments include filter type (reg), data rate (reg), is chop mode enabled
//returns conversion time in milliseconds 
float calculateADS1262Timing(PrecisionLoggerSettings loggerSettings) {
  uint8_t settlingFactor;
  float sampleRate;

  // Identify settling factor for each filter
  switch (loggerSettings.filterType) {
    case ADS126X_FIR:
      settlingFactor = 3;
      break;
    case ADS126X_SINC1:
      settlingFactor = 1;
      break;
    case ADS126X_SINC2:
      settlingFactor = 2;
      break;
    case ADS126X_SINC3:
      settlingFactor = 3;
      break;
    case ADS126X_SINC4:
      settlingFactor = 4;
      break;
    default:
      settlingFactor = 4; //invalid filter type so set to highest settling factor
      break;
  }

   //get sample rate value
  switch (loggerSettings.sampleRateSetting) {
    case ADS126X_RATE_2_5:
      sampleRate = 2.5;
      break;
    case ADS126X_RATE_5:
      sampleRate = 5.0;
      break;
    case ADS126X_RATE_10:
      sampleRate = 10.0;
      break;
    case ADS126X_RATE_16_6:
      sampleRate = 16.6;
      break;
    case ADS126X_RATE_20:
      sampleRate = 20.0;
      break;
    case ADS126X_RATE_50:
      sampleRate = 50.0;
      break;
    case ADS126X_RATE_60:
      sampleRate = 60.0;
      break;
    case ADS126X_RATE_100:
      sampleRate = 100.0;
      break;
    case ADS126X_RATE_400:
      sampleRate = 400.0;
      break;
    case ADS126X_RATE_1200:
      sampleRate = 1200.0;
      break;
    case ADS126X_RATE_2400:
      sampleRate = 2400.0;
      break;
    case ADS126X_RATE_4800:
      sampleRate = 4800.0;
      break;
    case ADS126X_RATE_7200:
      sampleRate = 7200.0;
      break;
    case ADS126X_RATE_14400:
      sampleRate = 14400.0;
      break;
    case ADS126X_RATE_19200:
      sampleRate = 19200.0;
      break;
    case ADS126X_RATE_38400:
      sampleRate = 38400.0;
      break;
    default:
      sampleRate = 2.5; //invalid filter type so set to highest settling factor
      break;
  }

  float Tconv_ms = 1000.0 / sampleRate;  // Base conversion period in ms

  // Calculate settling time
  //there is a ~400usec delay at any filter setting beyond the conversion rate
  Tconv_ms = (Tconv_ms * settlingFactor) + 1; //add one because most filters have a ~400usec delay

  // Double conversion period if chopping mode is enabled
  if (loggerSettings.choppingEnabled) {
    Tconv_ms *= 2.0;  // Chopping mode A doubles conversion time
  } 
  //double conversion period if IDAC rotation mode is enabled
  if (loggerSettings.iDACRotation) {
    Tconv_ms *= 2.0;  // Chopping mode A doubles conversion time
  } 

  return Tconv_ms; //return delay time between measurements
}

