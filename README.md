# AutoGreenhouse

Automatic greenhouse monitoring, including basic control. Arduino-based.

Monitor: 
  - External Temp & Relative Humidity
  - External rain detector
  - Internal Temp & Relative Humidity
  - Light detector for day/night
  - Dew point detector
  - Soil humidity detector
  - Atmospheric pressure
  
Log:
  - microSD card module for data logging
  - WiFi module for uploading to IoT (Thingspeak) and date gathering
  - LCD 16x2 for visual display of situation

Control (Via 220V Relais):
  - Heating
  - Cooling (open upper window or switch on fan)
  - Reduce humidity (open window or switch on fan)
  - Watering 
  - Blackout (limit daylight duration for plants which need to be forced)
  
Remote updating:
  - Added read-in of values from Thingspeak channel description to update controls' trigger values. 
  - Format is: [semicolon]Variable[space]Value[space]Variable2[space]...ValueN[space][semicolon]
  - Sequence of variables cannot be altered from following example:
  ";HeatOn 0.5 HeatOff 3 LenDay 41400000 LenBO 10800000 OHeatOn 35 OHeatOff 28 H2Oon 15 H2Ooff 30 RHOn 90 RHOff 60 RHminT 5 DewMax 95 ;"
  
Libraries to download:
- BMP180 pressure sensor lib, from: https://github.com/sparkfun/BMP180_Breakout_Arduino_Library/archive/master.zip 
          https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-/installing-the-arduino-library
- LiquidCrystal_I2C (V1.2 is ok) from fmalpartida: https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
-         http://arduino-info.wikispaces.com/LCD-Blue-I2C
- HTU21D, from:   https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library/archive/master.zip
          https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide

Warning/Debug needed: 
- Only ~290bytes free for local variables, MAY BE UNSTABLE (even if most variables are preassigned and should not fragment). 96% flash used.
- Linux not compiling (libraries conflict?)

More explanations coming
