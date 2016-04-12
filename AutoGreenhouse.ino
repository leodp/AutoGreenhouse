// Weather logger greenhouse control - 20160310 Leodp@yahoo.com
// License: GNU 3.0

// Working Setup: -------------------------------
// Digit 10 11 12 13 -> CS SCK MOSI MISO (SD card)
// Analog A4 A5 -> SCL SDA (LCD I2C + 2x T-RH sensors)
// GND, Vin    To power 3.3V supply for WiFi card (may need lot of current)

// LCD 2x16, + IIC module, to monitor situation
// 2x HTU21D Temp & Humid sensors
// 1x SD card SPI module, to save data (set values and actual values)
// Mux/Demux Digit 09 for multiplexing the IIC line (T/RH sensors have the same IIC address)
// 1x WiFi card to send data to Thingspeak

// SFE_BMP180 pressure module
#include <SFE_BMP180.h>  //pressure sensor
SFE_BMP180 PRESSURE;
#define ALTITUDE 345.0   // Local altitude above sea, in meters (not used, needed if you want to recover the atmospheric pressure at sea level)

// LCD and communication
#include <Wire.h>
#include <LiquidCrystal_I2C.h>                                  // arduino-info.wikispaces.com/LCD-Blue-I2C
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address to 0x20 for a 20 chars 2 line display and the pins on the I2C chip used for LCD

// Temp Humidity sensor
#include "HTU21D.h"                        // https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
#define HTU21D_ADDRESS 0x27                // Unshifted 7-bit I2C address for the sensor
#define TRIGGER_TEMP_MEASURE_NOHOLD 0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD 0xF5
HTU21D myHumidity;                         //Create an instance of the object relative humidity
HTU21D myTemp;                             //Create an instance of the object  Temperature

// SD Card                  // http://arduino-info.wikispaces.com/SD-Cards    https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial  http://garagelab.com/profiles/blogs/tutorial-how-to-use-sd-card-with-arduino
#include <SD.h>             // SD card attached to SPI bus as follows:  MOSI - pin 11   MISO - pin 12    CLK - pin 13   CS - pin 10
#define file_name "GH1.txt" // log file name
File myFile;


// ESP8266 - WiFi card
#define SSID "your_ssid"                                      // edit with your wifi ssid here
#define PASS "your_pwd"                                       // edit with your wifi key here
#define myWriteAPIKeyWB  "yourChWriteKey"                     // edit with your thingspeak channel info
#define myReadAPIKeyWB   "yourChReadKey"                      // edit with your thingspeak channel info  IF I DEFINE A READ KEY I CAN KEEP THE CHANNEL PRIVATE
#define myChannelID      "yourThingspeak_chID"                // edit with your thingspeak channel info

#define DST_IP "184.106.153.149"                              // Thingspeak API IP address
#define IPWiFiCARD "192"                                      // if it is  connected, IP=192.168... (you may need to edit here, check your AP settings)
#define SERIALspeed 115200                                    // Speed for ESP8266, in different firmwares may be different
char cmdBuf[124] = {0};                                       // for the AT command. Check that Thingspeak description length is shorted than this buffer len
char cmd2Buf[30] = {0};                                       // for the AT command
bool ConnOK = false;
//bool WiFiMOD = true;

//define I/O lines & connections
#define chipSelectSD  10     // SD card CS
#define relay_I2cMultiplex 9 // Switch to enable a second T/RH sensor on I2C

// Digital lines for controls
#define relay_Heat 8
#define relay_Water 7
#define relay_Overheat 6     // possible PWM on motor
#define relay_BOut 5         // possible PWM on motor
#define relay_WiFiReset 4    // Hard reset necessary for the dumb ESP8266
#define relay_LowRH 3        // possible PWM on motor
#define relay_Humidify 2 

// Analog lines for sensing
#define anaCh_dew 0          // channel to sense dew point
#define anaCh_soil 1         // channel to sense soil humidity
#define anaCh_sun 2          // channel to sense day/night (Photodiode)
#define anaCh_rain 3         // channel to sense rain

// General variables
char buf[7] = "";                        // temporary buffer
unsigned char i,  j;                     // generic index
char k;                                  // generic index
double Ti, To;                           // temperatures: inside, outside
char RHi = 0;                            // Relative Hunidities inside, outside
char RHo = 0;
double pressure;                         // pressure, temp(edit:reuse Ti)  double p0,a; //not used, for equiv sea-level pressure

unsigned char sun = 0;            // Day
unsigned char rain = 0;           // rain sensor
unsigned char dew = 0;            // dew point sensor
unsigned char soil = 0;           // Soil humidity sensor
//unsigned int SDflag = 0;        // file open flag
unsigned char dayflag = 0;        // to detect the day
double timeDiffMeas = 0;          // variable to measure the time passed on the Black out tent
unsigned char dayStateMachine = 0;// for controlling blackout opening/closing routine

byte CtrlByte  = B00000000;       // http://playground.arduino.cc/Code/BitMath some booleans for recording the status of the controls: sun,rain,humidify,blackOut,heating,overheated,watering,reduceRH


// VALUES TO SET AUTOMATIC CONTROLS ------------------------  EDIT HERE TO SUIT YOUR NEEDS ----------------------

double Day  = 11.50;                  // Time of illumination/allowed daylength (in h, first value indicate the effective hours)
double BO =    3;                     // Time to keep Blackout. During the night it can be opened for air exchange. Increase according to your needs
#define DayJitt   5                      // Number of day-begin detections. (To avoid jitter, due for example to short light pulses, i.e. lights on for 1min during night visits)
                                         // No PWM on BOut motor, as 1 and 0 control the two motor directions (open/close)
double Ht1 = 0.5;                        // Too low Temp: start heating. Need float precision on temp warning close to 0°C
double Ht0 = 3;                          // Safe Temp, stop heating

char OHt1 = 40;                          // Open window in case of overheating: max acceptable temp before plants suffer
char OHt0 = 30;                          // Close window to keep this minimum Temp safeguarded
#define overheatPWM   128                // PWM controls for the motors. Set to 255 for always on/always off

char H2O1 = 15;                          // Turn on auto watering (value from soil humidity sensor)
char H2O0 = 30;                          // Turn off auto watering
unsigned char H2OT = 30;                 // Auto watering switch-on time. Set to 0 to switch on auto-watering based only on soil humidity. In minutes. Max 255

char RH1 = 15;                           // Too low Temp: start heating. Need float precision on temp warning close to 0°C
char RH0 = 35;                           // Safe Temp, stop heating

char ORH1    = 95;                       // Max humidity allowed: take precautions to reduce humidity (open windows, ventilate with external air)
char ORH0   = 75;                        // Safe humidity level (reduced molds/mildew risk): switch off ventilation.
char ORHT  = 5;                          // If temperature falls below this value then stop ventilation to avoid undercooling, even if RH is high
char ODew = 94;                          // If the dew sensor is indicating condensation, you may want to start ventilation. Increase above 100 to disable
#define lowRHPWM   128                   // PWM controls for the motors. Set to 255 for always on/always off


//  SETUP  /////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  lcd.begin(16, 2); // initialize the lcd for 16 chars 2 lines and turn on backlight

  pinMode(relay_WiFiReset, OUTPUT);
  pinMode(relay_I2cMultiplex, OUTPUT);
  pinMode(chipSelectSD, OUTPUT);
  pinMode(relay_Humidify, OUTPUT);
  pinMode(relay_Heat, OUTPUT);
  pinMode(relay_BOut, OUTPUT);
  pinMode(relay_Overheat, OUTPUT);
  pinMode(relay_Water, OUTPUT);
  pinMode(relay_LowRH, OUTPUT);
  pinMode(anaCh_sun, INPUT);
  pinMode(anaCh_rain, INPUT);
  pinMode(anaCh_dew, INPUT);
  pinMode(anaCh_soil, INPUT);
  
  digitalWrite(relay_I2cMultiplex, LOW);  // Init temp sensor # switch
  digitalWrite(chipSelectSD, HIGH);       // SD Card ACTIVE
  digitalWrite(relay_Humidify, LOW);      // LOW=0 = not operative = do not humidify the greenhouse (control for tropical environment)
  digitalWrite(relay_Heat, LOW);          // LOW=0 = not operative = do not heat the greenhouse
  digitalWrite(relay_BOut, LOW);          // LOW=0 = not operative = OPEN WINDOW
  digitalWrite(relay_Overheat, LOW);      // LOW=0 = not operative = no blackout
  digitalWrite(relay_Water, LOW);         // LOW=0 = not operative = no watering
  digitalWrite(relay_LowRH, LOW);         // LOW=0 = not operative = no dehumidify
  


  PRESSURE.begin();
  //  if (PRESSURE.begin()) //debug in alternative to the above command
  //   { LcdDbg("BMP180 init 1");}
  //  else
  //   { LcdDbg("BMP180 init 0\n\n");// Problema
  //    while(1); // Pause forever.
  //   }

  Serial.begin(SERIALspeed);           // Start serial communication with WIFi module.
  Serial.setTimeout(5000);             // Set time to wait for response strings to be found. Keep high to avoid WiFi problems (i.e. long connect/reply times)
  digitalWrite(relay_WiFiReset, LOW);  // Hard reset of WiFi ESP8266 works better than anything to avoid "Busy p..." errors
  delay(500);
  digitalWrite(relay_WiFiReset, HIGH);
  Serial.println(F("AT+RST"));         // After a hard reset a soft reset helps to complete the cleanup
  if (Serial.find("dy"))
  { Serial.println(F("AT+CIPMUX=0"));//set the connection mode, no mux        //Serial.println("AT+GMR");        // print firmware version number
    //Serial.readString();
    for (i = 0; i < 2; i++)
    { connectWiFi();                 //  connect to the wifi
      if (ConnOK) 
      { break; }// WiFi connection success, stop connect loop
    }
  }
  else
  { Serial.println(F("No WiFi module"));
    //WiFiMOD = false;
  }

  SD.begin(chipSelectSD);                                // Start writing to file in microSD card
  myFile = SD.open(file_name, FILE_WRITE);
  //delayFnct();
  //if (!myFile) {
  //  Serial.println("file open failed!");
  //  myFile.close();
  //  return; }
  myFile.println(F("Ti\tRHi\tTo\tRHo\tmBar\tRain\tHSoil\tDew\tSn_Rn_BO_Ht_<Ht_H2O_RH_<RH\tt\tDate")); // Header: T & RH in, T & RH out, pressure, Rain, Soil, Dew, Dailight, Blackout, Heating, Too hot, watering, reduce humidity, time since start, date from network
  myFile.close();
}


// Main Loop  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  //  if necessary reconnect to the wifi  (connection may be lost due to a weak or temporary WiFi signal)
  Serial.println(F("AT+CIFSR"));        // does not always work as expected (detect connection), but helps to reconnect if necessary
  delayFnct();
  if (Serial.find(IPWiFiCARD))
  { ConnOK = true;   }                  // It's connected, no need to do anything Serial.println(F("Connected!"));}
  else
  {
    ConnOK = false;
    Serial.println(F("AT+CWQAP"));         // Be sure to disconnect. Helps to avoid keep-disconnected
    Serial.readString();
    //delayFnct();
    digitalWrite(relay_WiFiReset, LOW); // Hard reset of WiFi ESP8266 works better than anything to clear "Busy p..." lock-ups
    delay(500);
    digitalWrite(relay_WiFiReset, HIGH);
    Serial.println(F("AT+RST"));
    Serial.readString();
    delayFnct();
    Serial.println(F("AT+CIPMUX=0"));   // Set the connection mode, no mux
    connectWiFi();
  }

  // ACQUIRE SENSORS DATA---------------------------------------------------------------
 
  // ACQUIRE T & RH
  digitalWrite(relay_I2cMultiplex, 0);  // Switch to the internal temp/RH sensor
  delayFnct();                          // Need some delay to avoid "998" readout
  do { Ti =    myTemp.readTemperature();// Read internal temp and RH
       RHi =  myHumidity.readHumidity();
     } while(RHi==998 || Ti==998);      // Sometimes the readout is wrong: in case repeat the measurement. ATENTION, MAY LOCK!
  digitalWrite(relay_I2cMultiplex, 1);  // Switch to the external temp/RH sensor
  delayFnct();                          // Need some delay to avoid "998" readout
  do { To =    myTemp.readTemperature();// Read external temp and RH
       RHo =  myHumidity.readHumidity();
     } while(RHo==998 || To==998);      // Sometimes the readout is wrong: in case repeat the measurement. ATENTION, MAY LOCK!

  
  // ACQUIRE PRESSURE
  cmd2Buf[0] = PRESSURE.startTemperature();                 // Start a temperature measurement:If request is successful, the number of s to wait is returned, else 0
  if (cmd2Buf[0] != 0)
  { delay(cmd2Buf[0]);                                      // Wait for the measurement to complete. Return=0 is fail
    cmd2Buf[0] = PRESSURE.getTemperature(Ti);
    if (cmd2Buf[0] != 0)
    { cmd2Buf[0] = PRESSURE.startPressure(3);               // Start a pressure measurement:The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).If request is successful, the number of s to wait is returned, else 0
      if (cmd2Buf[0] != 0)
      { delay(cmd2Buf[0]);                                  // Wait for the measurement to complete:
        cmd2Buf[0] = PRESSURE.getPressure(pressure, Ti);     // The function requires the previous temperature measurement (T) for calibration. Return=0 is failure
        //  if (cmd2Buf[0] != 0)
        //  { p0 = PRESSURE.sealevel(pressure,ALTITUDE);    // Sealevel pressure is commonly used in weather reports:
        //    a = PRESSURE.altitude(pressure,p0); }         // Inverse: altitude from pressure: P = absolute pressure in mb, p0 = baseline pressure in mb. a in meters
        //  else { LcdDbg("Ep1\n"); }                       // Do something in case of error
      } //else LcdDbg(F("Ep2\n"));                            // Here and following: print error messages on LCD for debug
    }   //else LcdDbg(F("ET1\n"));
  }     //else LcdDbg(F("ET2\n"));


  // ACQUIRE Arduino analog input channels
  rain = ( (int) (1023 - analogRead(anaCh_rain))/10.23 );         // Rescale. A high value is indicating rain (max 100)
  sun =  ( (int) (1023 - analogRead(anaCh_sun))/10.23  ) ;        // Rescale. A high value is indicating sunlight (max 100)
  dew =  ( (int) (analogRead(anaCh_dew))/10.23 );                 // Rescale. A high value is indicating condensation (max 100)
  soil = ( (int) (1023 - analogRead(anaCh_soil))/10.23 );         // Rescale. Value is indicating how much water is present in the soil (max 100)
  CtrlByte &=  63 ;
  CtrlByte |= ( ((sun/50)<<7)+ ((rain/50)<<6)  );// Log digital sun/rain
  

  // CONTROLS IN THE GREENHOUSE ---------------------------------------------------------------------------------------------------------------------

  // Black-out control, to force plants via light (growing/flowering/fruiting/dormiency/...)--------------------------------------
  if (sun && !dayStateMachine) {
    dayflag += 1; // Count to avoid jitter on stray light, allows for max DayJitt wrong light detections
  }
  if (dayflag == DayJitt && dayStateMachine == 0)                             // Start recording the day length, up to when we have reached the allowed illumination time
  { timeDiffMeas = millis()/1000;
    dayStateMachine = 1;
  }
  if ((millis()/1000 - timeDiffMeas)/60/60 > Day && dayStateMachine == 1)     // If we have had enough day turn on the blackout system. May be connected also to a small air-circulation fan, too keep low T & RH
  { CtrlByte |= B00100000;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 2;
  }
  if ((millis()/1000 - timeDiffMeas)/60/60 > BO + Day && dayStateMachine == 2) // After a while open again the blackout tent, for easing ventilation (it should be already night outside)
  { CtrlByte &= B11011111;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 0;
    dayflag = 0;
  }


  // Heat Control--------------------------------------
  if (Ti < Ht1)
  { CtrlByte |= B00010000;}
  if (Ti > Ht0)         // Heating On and Off have different values, to allow for hysteresis
  { CtrlByte &= B11101111;}
  digitalWrite(relay_Heat, (CtrlByte>>4)&1);


  // Overheat Control--------------------------------------
  if (Ti > OHt1)                // Overheating indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00001000;}
  if (Ti < OHt0)
  { CtrlByte &= B11110111;}
  analogWrite(relay_Overheat, ((CtrlByte>>3)&1)*overheatPWM);


  // Watering Control--------------------------------------
  if (H2OT==0)
    {  if (soil < H2O1)                 // Watering On and Off have different values, to allow for hysteresis
          { CtrlByte |= B00000100;}
       if (soil > H2O0)
          { CtrlByte &= B11111011;}
    }
   else 
    {  if( dayStateMachine == 1)                     // if START DAY:timeDiffMeas
        {  CtrlByte |= B00000100; }                  // then switch on
       if ((millis()/1000 - timeDiffMeas)/60 > H2OT) // If watering has been on for long enough (in minutes)
        {  CtrlByte &= B11111011; }                  // switch off
    }
  digitalWrite(relay_Water, (CtrlByte>>2)&1);
  //Serial.println(soil);
  //Serial.println(H2O1);
  //Serial.println(H2O0);
  

  // Humidification Control--------------------------------------
  if (RHi < RH1)              // Humidifying indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00000010;}
  if (RHi > RH0)
  { CtrlByte &= B11111101;}
  digitalWrite(relay_Humidify, (CtrlByte>>1)&1);

  
  // Control to lower the air humidity -----------------------------------                                   // http://planetcalc.com/2167/    RH/T proportional to absolute H2O content (T in Kelvin)
  if (   (      ( RHi > ORH1  &&(RHi*((int)To+273)>RHo*((int)Ti+273)) )      || dew > ODew   )  && To > ORHT)// If [(internal_RH too high AND internal_H2O<external_H2O) OR condensation happening] AND external temperature not too low
    //if ( (RHi>ORH1  || dew>ODew) && To>ORHT)                                                               // If [(internal_RH too high AND internal_H2O<external_H2O) OR condensation happening] AND external temperature not too low
  { CtrlByte |= B00000001;}                                                                                  // Switching on can decrease internal RH
  if (   (      ( RHi < ORH0                                          )      && dew < ODew   )  || To < ORHT)// If RH inside is low enough or the external absolute humidity is higher =>turn off
  { CtrlByte &= B11111110;}
  analogWrite(relay_LowRH, (CtrlByte&1)*lowRHPWM);


  // WRITE/UPDATE/UPLOAD routines -------------------------------------------------------------------------------------------------------

  //Serial.println(F("UPDATING"));
  update_LCD   ();

  if (ConnOK) 
    { update_WiFi ();}// Thingspeak update routine

  //update_Serial();  // COMMENTED OUT, IT MAY INTERFERE WITH WiFi card which is also on RX-TX Serial lines. Un-comment on for debug

  // UPDATE SD card
  myFile = SD.open(file_name, FILE_WRITE);
  //if (!myFile) {SDflag=1;}  
  update_SD   ();
  myFile.close();


  delay(100000);// lengthen the loop, act around 0.5 per minute.
}   // end of main loop


// ROUTINES-FUNCTIONS  ////////////////////////////////////////////////////////////////////////////////////////

void update_SD() {
  //Serial.println(F("UPDATINGSD"));
  myFile.print(Ti);
  myFile.print(F("\t"));
  myFile.print((int)RHi);
  myFile.print(F("\t"));
  myFile.print(To);
  myFile.print(F("\t"));
  myFile.print((int)RHo);
  myFile.print(F("\t"));
  myFile.print(pressure);
  myFile.print(F("\t"));
  myFile.print((int)rain);
  myFile.print(F("\t"));
  myFile.print((int)soil);
  myFile.print(F("\t"));
  myFile.print((int)dew);
  myFile.print(F("\t"));
  myFile.print((int)CtrlByte);
  myFile.print(F("\t"));
  myFile.print(millis()/1000);
  myFile.print(F("\t"));
  myFile.println(cmd2Buf);
  //myFile.flush();
  //Serial.println(F("UPDATED_SD"));
}

void update_LCD() {                 // LCD display routine
  lcd.clear();
  lcd.setCursor(0, 0);              // Start at character 0 on line 0
  lcd.print(F("I____'__\%"));       // LCD preparation of indications: I(internal) '(degrees C) %(RH%)
  lcd.setCursor(0, 1);              // Start at character 0 on line 1
  lcd.print(F("O____'__\%r  s"));   // LCD preparation of indications:  O(outside) '(degrees C) %(RH%) d(dewmax) s(soil)
  lcd.setCursor(1, 0);
  lcd.print(Ti, 1);
  lcd.setCursor(1, 1);
  lcd.print(To, 1);
  lcd.setCursor(6, 0);
  lcd.print((int)RHi);
  lcd.setCursor(6, 1);
  lcd.print((int)RHo);
  lcd.setCursor(9, 0);
  lcd.print(pressure, 1);
  lcd.setCursor(10, 1);
  lcd.print((int)rain);
  lcd.setCursor(13, 1);
  lcd.print((int)soil);
  lcd.setCursor(15, 0);
  lcd.print( CtrlByte>>4   , HEX); // Digital value, to sum up important parameters
  lcd.setCursor(15, 1);
  lcd.print( CtrlByte&15 , HEX); // Digital value, to sum up important parameters
  
}


void LcdDbg (String dbgmsg) // not used (for debug purposes, as Serial is used already by WiFi)
{ lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println(dbgmsg);
}


void connectWiFi()
{ cmdBuf[0] = {0};
  Serial.readString();
  delayFnct();
  Serial.println(F("AT+CWMODE=1"));
  Serial.readString();
  delayFnct();
  strcat(cmdBuf, "AT+CWJAP=\"");
  strcat(cmdBuf, SSID);
  strcat(cmdBuf, "\",\"");
  strcat(cmdBuf, PASS);
  strcat(cmdBuf, "\"");
  Serial.println(cmdBuf);
  delay(4000);                    // Give time to connect to the WiFi
  cmdBuf[0] = {0};

  if(Serial.find("OK"))       
  { //Serial.println(F("WF1"));     // WiFi connection OK
    ConnOK = true;
  }
  else
  { //Serial.println(F("EWiFi"));   //WiFi ERROR
    ConnOK = false;
  }
  Serial.readString();
  delayFnct();
}


void  update_WiFi ()
{ cmdBuf[0] = {0};                           // 1) Update the values on thingspeak with measured data
  strcat(cmdBuf, "AT+CIPSTART=\"TCP\",\"");  // api.thingspeak.com
  strcat(cmdBuf, DST_IP);                   
  strcat(cmdBuf, "\",80");
  Serial.println(cmdBuf);
  ///delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  Serial.readString();                      // Prepare GET string
  cmdBuf[0] = {0};
  cmd2Buf[0] = {0};
  strcat(cmdBuf, myWriteAPIKeyWB);
  strcat(cmdBuf, "&field1=");
  strcat(cmdBuf, dtostrf( Ti , 0, 2, buf));
  strcat(cmdBuf, "&field2=");
  strcat(cmdBuf, dtostrf( RHi , 0, 0, buf));
  strcat(cmdBuf, "&field3=");
  strcat(cmdBuf, dtostrf( To , 0, 2, buf));
  strcat(cmdBuf, "&field4=");
  strcat(cmdBuf, dtostrf( RHo , 0, 0, buf));
  strcat(cmdBuf, "&field5=");
  strcat(cmdBuf, dtostrf( pressure , 0, 2, buf));
  strcat(cmdBuf, "&field6=");
  strcat(cmdBuf, dtostrf( dew , 0, 0, buf));
  strcat(cmdBuf, "&field7=");
  strcat(cmdBuf, dtostrf( soil , 0, 0, buf));
  strcat(cmdBuf, "&field8=");
  strcat(cmdBuf, dtostrf( (int)CtrlByte , 0, 0, buf)); // digital flags to log
  //strtmp=String(CtrlByte,BIN);
  //strtmp.toCharArray(buf, 8);
  //strcat(cmdBuf, buf ); // digital flags to log
  //Serial.println(buf);
  strcat(cmdBuf, "\r\n\r\n");
  // CIPSEND
  strcat(cmd2Buf, "AT+CIPSEND=");
  strcat(cmd2Buf, dtostrf( strlen(cmdBuf)+20 , 0, 0, buf));  //add 20 for the first GET part. (Separated to shorten buffer and save RAM)
  Serial.println(cmd2Buf);
  ///delayFnct();
  if (Serial.find(">"))
    {   Serial.print(F("GET /update?api_key="));
        Serial.print(cmdBuf);  }
  else
    { //Serial.print(cmdBuf);
      Serial.println(F("AT+CIPCLOSE")); }
  Serial.readString();
  delayFnct();


  cmdBuf[0] = {0};                          // 2) Start AT communication with Thingspeak to read channel description (contains controls values to be updated remotely)
  strcat(cmdBuf, "AT+CIPSTART=\"TCP\",\""); 
  strcat(cmdBuf, DST_IP);                   
  strcat(cmdBuf, "\",80");
  Serial.println(cmdBuf);
  ///delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  Serial.readString();               
  Serial.println(F("AT+CIPSEND=49"));      // Prepare GET string
  ///delayFnct();  
  if (Serial.find(">"))
  { Serial.print(F("GET /channels/91010.json                     \r\n\r\n")); 
    //Serial.print(F("GET /channels/91010.json?key="));   // these 3 lines instead of the above if you want to keep the channel private
    //Serial.print(myReadAPIKeyWB);
    //Serial.print(F("\r\n\r\n")); 
    ///delayFnct();
    cmdBuf[0] = {0};
    cmd2Buf[0] = {0};
    Serial.readBytesUntil(';', cmdBuf, 124);//read quickly 'till first variable (avoids overflow)
    cmdBuf[0] = {0};
    i = Serial.readBytesUntil(';', cmdBuf, 124); //read quickly all variables (avoids overflow)
    ///cmdBuf[i] = {0};
    //Serial.print(F("PARAMS:"));
    //Serial.println(cmdBuf);
    j = 0;
    k = -1;
    for (i = 0; i <= strlen(cmdBuf); i++)
    { cmd2Buf[j] = cmdBuf[i];
      if (cmdBuf[i] == ' ') // Found separator
      { cmd2Buf[j] = {0};
        k = k + 1;
        j = 0;
        switch (k)
        { case 1:                    /// ATTENTION: NO CHECK THAT VALUES ARE ACCEPTABLE, ENTER CORRECT VALUES IN THINGSPEAK DESCRIPTION!!!!
            Day = atol(cmd2Buf);
            break;
          case 3:
            BO = atol(cmd2Buf);
            break;
          case 5:
            Ht1 = atol(cmd2Buf);
            break;
          case 7:
            Ht0 = atol(cmd2Buf);
            break;
          case 9:
            OHt1 = atoi(cmd2Buf);
            break;
          case 11:
            OHt0 = atoi(cmd2Buf);
            break;
          case 13:
            H2O1 = atoi(cmd2Buf);
            break;
          case 15:
            H2O0 = atoi(cmd2Buf);
            break;
          case 17:
            H2OT = atoi(cmd2Buf);
            break;
          case 19:
            RH1 = atoi(cmd2Buf);
            break;
          case 21:
            RH0 = atoi(cmd2Buf);
            break;
          case 23:
            ORH1 = atoi(cmd2Buf);
            break;
          case 25:
            ORH0 = atoi(cmd2Buf);
            break;
          case 27:
            ORHT = atoi(cmd2Buf);
            break;
          case 29:
            ODew = atoi(cmd2Buf);
            break;
          default:
            break;
        }       // switch case on recognized "word"
      }         // if loop on recognized separator, "word" completed
      else
      {j=j+1;}
    }           // for loop on string length
  }             // if loop on communication from WiFi 
  else
  { Serial.println(F("AT+CIPCLOSE")); } //Serial.println("DESCRIPTION NOT FOUND");
  Serial.readString();      // close all communication queues
  delayFnct();

//    Serial.println(Day);
//    Serial.println(BO);
//    Serial.print(F("Ht1:"));
//    Serial.println(Ht1);
//    Serial.print(F("Ht0:"));
//    Serial.println(Ht0);
//    Serial.println(OHt1);
//    Serial.println(OHt0);
//    Serial.println(H2O1);
//    Serial.println(H2O0);
//    Serial.println(RH1);
//    Serial.println(RH0);
//    Serial.println(ORH1);
//    Serial.println(ORH0);
//    Serial.println(ORHT);
//    Serial.println(ODew);

  cmdBuf[0] = {0};                          // 3) Finish by getting the date from server
  ///cmd2Buf[0] = {0};                         // will save here the date string
  strcat(cmdBuf, "AT+CIPSTART=\"TCP\",\"");  // api.thingspeak.com
  strcat(cmdBuf, DST_IP);                   
  strcat(cmdBuf, "\",80");
  Serial.println(cmdBuf);
  //delayFnct();
  //if (Serial.find("OK")) {
  //  delay(10); //LcdDbg("TCPOK");
  //}
  Serial.readString();
  delayFnct();
  // Send GET string
  Serial.println(F("AT+CIPSEND=12"));
  //delayFnct();  
  if (Serial.find(">"))
  { Serial.print(F("GET date\r\n\r\n"));
    ///delayFnct();
  }
  cmd2Buf[0] = {0};                         // will save here the date string
  if (Serial.find("Date: "))                // get the date line from the http header (sends a fake/wrong GET request)
  { //delayFnct();
    if (Serial.available() > 0) 
      { Serial.readBytes(cmd2Buf, 29);   }  // read the incoming byte(s)
  }
  //cmd2Buf[29] = {0};                      // will save here the date string
  Serial.readString();
  delayFnct();
  Serial.print(F("DATE:"));
  Serial.println(cmd2Buf);
  Serial.readString();
  delayFnct();
}


void update_Serial()           // Not use for interference with WiFi module.
{ Serial.print(F("Ti:"));
  Serial.print(Ti);            //eventually dtostrf(Ti,2,2,temp); dtostrf(value, width_before_comma, precision, output);
  Serial.print(F("\tRHi:"));
  Serial.print(RHi);
  Serial.print(F("\tTo:"));
  Serial.print(To);
  Serial.print(F("\tRHo:"));
  Serial.print(RHo);
  Serial.print(F("\tPress:"));
  Serial.print(pressure);
  //Serial.print(F("\tp0:"));  //if p0 enabled you may want to log the equivalent pressure the at-sea-level
  //Serial.print(p0);
  Serial.print(F("\tRain:"));
  Serial.print(rain);
  Serial.print(F("\tSoilH:"));
  Serial.print(soil);
  Serial.print(F("\tDew:"));
  Serial.print(dew);
  Serial.print(F("\tDay:"));
  Serial.print(sun);
  Serial.print(F("\tCtrlByte:"));
  Serial.print(CtrlByte);
  Serial.print(F("\tT:"));
  Serial.print(millis()/1000);
  Serial.print(F("\tDate:"));
  Serial.println(cmd2Buf);
  Serial.readString(); //clean the buffer and the communication
  delayFnct();
}

void delayFnct()  //just to save memory
{ delay(120);}
