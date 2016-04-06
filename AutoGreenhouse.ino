// Weather logger greenhouse control - 20160310 Leodp@yahoo.com
// License: GNU 3.0

// Working Setup: -------------------------------
// (possibility of 6x relais (220V) for driving, digital pins 03-08)
// Digit 09 -> I2C multiplex switch, for multiplexing the 2 T-RH sensors
// Digit 10 11 12 13 -> CS SCK MOSI MISO (SD card)

// Analog A4 A5 -> SCL SDA (LCD I2C + 2x T-RH sensors)

// GND, 3.3V   T-HR sensors
// GND, 5.0V   LCD, SD, 2xRelais (220V)
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
#define file_name "GH0.txt" // log file name
File myFile;


// ESP8266 - WiFi card
#define SSID "your_ssid"                                      // edit with your wifi ssid here
#define PASS "your_pwd"                                       // edit with your wifi key here
#define myWriteAPIKeyWB  "yourChWriteKey"                     // edit with your thingspeak channel info
#define myReadAPIKeyWB   "yourChReadKey"                      // edit with your thingspeak channel info  IF I DEFINE A READ KEY I CAN KEEP THE CHANNEL PRIVATE
#define myChannelID      "yourThingspeak_chID"                // edit with your thingspeak channel info

#define DST_IP "184.106.153.149"                              // Thingspeak API IP address
#define IPWiFiCARD "192."                                     // if it is  connected, IP=192.168... (you may need to edit here, check your AP settings)
#define SERIALspeed 115200                                    // Speed for ESP8266, in different firmwares may be different
char cmdBuf[145] = {0};                                       // for the AT command. Check that Thingspeak description length is shorted than this buffer len
char cmd2Buf[20] = {0};                                       // for the AT command
bool ConnOK = false;
//bool WiFiMOD = true;
//String  strtmp;

//define I/O lines & connections
#define chipSelectSD  10     // SD card CS
#define relay_I2cMultiplex 9 // Switch to enable a second T/RH sensor on I2C
//#define digCh_in 2           // channel for the "input switch" (not used, was for debug purposes)

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
char buf[9] = "";                       // temporary buffer
int i, j, k;                             // generic index
unsigned long time_begin, time_elapsed;  // time count
float Ti, To;                            // temperatures: inside, outside
int RHi = 0;                             // Relative Hunidities inside, outside
int RHo = 0;
double T, pressure;                      // pressure, temp  double p0,a; //not used, for equiv sea-level pressure

unsigned int sun = 0;           // Day
unsigned int rain = 0;          // rain sensor
unsigned int dew = 0;           // dew point sensor
unsigned int soil = 0;          // Soil humidity sensor
//unsigned int SDflag = 0;      // file open flag
unsigned int dayflag = 0;       // to detect the day
double timeDiffMeas = 0;        // variable to measure the time passed on the Black out tent
int dayStateMachine = 0;        // for controlling blackout opening/closing routine

byte CtrlByte  = B00000000;       // http://playground.arduino.cc/Code/BitMath some booleans for recording the status of the controls: sun,rain,humidify,blackOut,heating,overheated,watering,reduceRH


// VALUES TO SET AUTOMATIC CONTROLS ------------------------  EDIT HERE TO SUIT YOUR NEEDS ----------------------

int long LenDay  = 11.50 * 60 * 60 * 1000; // Time of illumination/allowed daylength (in ms, first value indicate the effective hours)
int long LenBO =    3 * 60 * 60 * 1000;    // Time to keep Blackout. During the night it can be opened for air exchange. Increase according to your needs
int DayJitt  = 5;                          // Number of day-begin detections. (To avoid jitter, due for example to short light pulses, i.e. lights on for 1min during night visits)
                                           // No PWM on BOut motor, as 1 and 0 control the two motor directions (open/close)
float HtOn = 0.5;                        // Too low Temp: start heating. Need float precision on temp warning close to 0°C
float HtOff = 3;                         // Safe Temp, stop heating

float OHtOn = 40;                          // Open window in case of overheating: max acceptable temp before plants suffer
float OHtOff = 30;                         // Close window to keep this minimum Temp safeguarded
unsigned int overheatPWM = 128;            // PWM controls for the motors. Set to 255 for always on/always off

int H2Oon = 15;                            // Turn on auto watering (value from soil humidity sensor)
int H2Ooff = 30;                           // Turn off auto watering

int RHon = 15;                     // Too low Temp: start heating. Need float precision on temp warning close to 0°C
int RHoff = 35;                    // Safe Temp, stop heating

int ORHon    = 95;                          // Max humidity allowed: take precautions to reduce humidity (open windows, ventilate with external air)
int ORHoff   = 75;                          // Safe humidity level (reduced molds/mildew risk): switch off ventilation.
int ORHT  = 5;                           // If temperature falls below this value then stop ventilation to avoid undercooling, even if RH is high
int DewMax = 94;                           // If the dew sensor is indicating condensation, you may want to start ventilation. Increase above 100 to disable
unsigned int lowRHPWM = 128;               // PWM controls for the motors. Set to 255 for always on/always off


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
  //pinMode(digCh_in, OUTPUT);            // Not used. Was for testing interrupts. To use external switch without external pull-up resistance (there's already an internal one)
  
  digitalWrite(relay_I2cMultiplex, LOW);  // Init temp sensor # switch
  digitalWrite(chipSelectSD, HIGH);       // SD Card ACTIVE
  digitalWrite(relay_Humidify, LOW);      // LOW=0 = not operative = do not humidify the greenhouse (control for tropical environment)
  digitalWrite(relay_Heat, LOW);          // LOW=0 = not operative = do not heat the greenhouse
  digitalWrite(relay_BOut, LOW);          // LOW=0 = not operative = OPEN WINDOW
  digitalWrite(relay_Overheat, LOW);      // LOW=0 = not operative = no blackout
  digitalWrite(relay_Water, LOW);         // LOW=0 = not operative = no watering
  digitalWrite(relay_LowRH, LOW);         // LOW=0 = not operative = no dehumidify
  //digitalWrite(digCh_in, HIGH);         // Not used anymore. To use the internal pull-up resistance. External switch does not need it
  
  //attachInterrupt(digitalPinToInterrupt(digCh_in), finito, RISING);  // Not used anymore. function to stop recording. (Simple programming exercise, not needed)


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
  if (Serial.find("ready"))
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
  myFile.println(F("Ti\tRHi\tTo\tRHo\tmBar\tRain\tHSoil\tDew\tSn_Rn_BO_Ht_<Ht_H2O_RH_<RH\tt_ms\tDate")); // Header: T & RH in, T & RH out, pressure, Rain, Soil, Dew, Dailight, Blackout, Heating, Too hot, watering, reduce humidity, time since start, date from network
  myFile.close();

  time_begin = millis();                                 // Start logging time
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


  time_elapsed = millis() - time_begin; // Log time since restart, to associate with measures



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
  cmd2Buf[0] = PRESSURE.startTemperature();                 // Start a temperature measurement:If request is successful, the number of ms to wait is returned, else 0
  if (cmd2Buf[0] != 0)
  { delay(cmd2Buf[0]);                                      // Wait for the measurement to complete. Return=0 is fail
    cmd2Buf[0] = PRESSURE.getTemperature(T);
    if (cmd2Buf[0] != 0)
    { cmd2Buf[0] = PRESSURE.startPressure(3);               // Start a pressure measurement:The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).If request is successful, the number of ms to wait is returned, else 0
      if (cmd2Buf[0] != 0)
      { delay(cmd2Buf[0]);                                  // Wait for the measurement to complete:
        cmd2Buf[0] = PRESSURE.getPressure(pressure, T);     // The function requires the previous temperature measurement (T) for calibration. Return=0 is failure
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
  { timeDiffMeas = time_elapsed;
    dayStateMachine = 1;
  }
  if ((time_elapsed - timeDiffMeas) > LenDay && dayStateMachine == 1)         // If we have had enough day turn on the blackout system. May be connected also to a small air-circulation fan, too keep low T & RH
  { CtrlByte |= B00100000;//BOUTbool = HIGH;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 2;
  }
  if ((time_elapsed - timeDiffMeas) > LenBO + LenDay && dayStateMachine == 2) // After a while open again the blackout tent, for easing ventilation (it should be already night outside)
  { CtrlByte &= B11011111;//BOUTbool = LOW;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 0;
    dayflag = 0;
  }


  // Heat Control--------------------------------------
  if (Ti < HtOn)
  { CtrlByte |= B00010000;}//heatbool = HIGH;
  if (Ti > HtOff)         // Heating On and Off have different values, to allow for hysteresis
  { CtrlByte &= B11101111;}//heatbool  LOW;
  digitalWrite(relay_Heat, (CtrlByte>>4)&1);


  // Overheat Control--------------------------------------
  if (Ti > OHtOn)                // Overheating indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00001000;}//overheatbool = HIGH;}
  if (Ti < OHtOff)
  { CtrlByte &= B11110111;}//overheatbool = LOW; }
  analogWrite(relay_Overheat, ((CtrlByte>>3)&1)*overheatPWM);


  // Watering Control--------------------------------------
  if (soil < H2Oon)                 // Watering On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00000100;}//waterbool = HIGH; }
  if (soil > H2Ooff)
  { CtrlByte &= B11111011;}//waterbool = LOW;  }
  digitalWrite(relay_Water, (CtrlByte>>2)&1);
  //Serial.println(soil);
  //Serial.println(H2Oon);
  //Serial.println(H2Ooff);
  

  // Humidification Control--------------------------------------
  if (RHi < RHon)              // Humidifying indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00000010;}//humidifybool = HIGH;}
  if (RHi > RHoff)
  { CtrlByte &= B11111101;}//humidifybool = LOW; }
  digitalWrite(relay_Humidify, (CtrlByte>>1)&1);

  
  // Control to lower the air humidity --------------------------------------                                   // http://planetcalc.com/2167/    RH/T proportional to absolute H2O content (T in Kelvin)
  if (   (      ( RHi > ORHon &&(RHi*((int)To+273)>RHo*((int)Ti+273)) )      || dew > DewMax   )  && To > ORHT) // If [(internal_RH too high AND internal_H2O<external_H2O) OR condensation happening] AND external temperature not too low
    //if ( (RHi>ORHon  || dew>DewMax) && To>ORHT)                                                               // If [(internal_RH too high AND internal_H2O<external_H2O) OR condensation happening] AND external temperature not too low
  { CtrlByte |= B00000001;}//lowRHbool = HIGH; }                                                                // Switching on can decrease internal RH
  if ( RHi < ORHoff    ||    RHi*((int)To+273)<RHo*((int)Ti+273)    ||    To < ORHT)                            // If RH inside is low enough or the external absolute humidity is higher =>turn off
  { CtrlByte &= B11111110;}//lowRHbool = LOW;  }
  analogWrite(relay_LowRH, (CtrlByte&1)*lowRHPWM);



  // WRITE/UPDATE/UPLOAD routines -------------------------------------------------------------------------------------------------------

  //Serial.println(F("UPDATING"));
  update_LCD   ();

  if (ConnOK) 
    { update_WiFi (); // Thingspeak update routine
    }

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
  Serial.println(F("UPDATINGSD"));
  myFile.print(Ti);
  myFile.print(F("\t"));
  myFile.print(RHi);
  myFile.print(F("\t"));
  myFile.print(To);
  myFile.print(F("\t"));
  myFile.print(RHo);
  myFile.print(F("\t"));
  myFile.print(pressure);
  myFile.print(F("\t"));
  myFile.print(rain);
  myFile.print(F("\t"));
  myFile.print(soil);
  myFile.print(F("\t"));
  myFile.print(dew);
  myFile.print(F("\t"));
  myFile.print(CtrlByte);
  myFile.print(F("\t"));
  myFile.print(time_elapsed);
  myFile.print(F("\t"));
  myFile.println(cmd2Buf);
  myFile.flush();
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
  lcd.print(RHi);
  lcd.setCursor(6, 1);
  lcd.print(RHo);
  lcd.setCursor(9, 0);
  lcd.print(pressure, 1);
  lcd.setCursor(10, 1);
  lcd.print(rain);
  lcd.setCursor(13, 1);
  lcd.print(soil);
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
  delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  Serial.readString();                      // Prepare GET string
  cmdBuf[0] = {0};
  cmd2Buf[0] = {0};
  strcat(cmdBuf, "GET /update?api_key=");
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
  strcat(cmdBuf, dtostrf( CtrlByte , 0, 0, buf)); // digital flags to log
  //strtmp=String(CtrlByte,BIN);
  //strtmp.toCharArray(buf, 8);
  //strcat(cmdBuf, buf ); // digital flags to log
  //Serial.println(buf);
  strcat(cmdBuf, "\r\n\r\n");
  // CIPSEND
  strcat(cmd2Buf, "AT+CIPSEND=");
  strcat(cmd2Buf, dtostrf( strlen(cmdBuf) , 0, 0, buf));
  Serial.println(cmd2Buf);
  delayFnct();
  if (Serial.find(">"))
    { Serial.print(cmdBuf);  }
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
  delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  Serial.readString();               
  Serial.println(F("AT+CIPSEND=49"));      // Prepare GET string
  delayFnct();  
  if (Serial.find(">"))
  { Serial.print(F("GET /channels/91010.json                     \r\n\r\n")); 
    //Serial.print(F("GET /channels/91010.json?key="));   // these 3 lines instead of the above if you want to keep the channel private
    //Serial.print(myReadAPIKeyWB);
    //Serial.print(F("\r\n\r\n")); 
    delayFnct();
    cmdBuf[0] = {0};
    cmd2Buf[0] = {0};
    Serial.readBytesUntil(';', cmdBuf, 140);//read quickly 'till first variable (avoids overflow)
    cmdBuf[0] = {0};
    i = Serial.readBytesUntil(';', cmdBuf, 140); //read quickly all variables (avoids overflow)
    cmdBuf[i] = {0};
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
            LenDay = atol(cmd2Buf);
            break;
          case 3:
            LenBO = atol(cmd2Buf);
            break;
          case 5:
            HtOn = atof(cmd2Buf);
            break;
          case 7:
            HtOff = atof(cmd2Buf);
            break;
          case 9:
            OHtOn = atof(cmd2Buf);
            break;
          case 11:
            OHtOff = atof(cmd2Buf);
            break;
          case 13:
            H2Oon = atoi(cmd2Buf);
            break;
          case 15:
            H2Ooff = atoi(cmd2Buf);
            break;
          case 17:
            RHon = atoi(cmd2Buf);
            break;
          case 19:
            RHoff = atoi(cmd2Buf);
            break;
          case 21:
            ORHon = atoi(cmd2Buf);
            break;
          case 23:
            ORHoff = atoi(cmd2Buf);
            break;
          case 25:
            ORHT = atoi(cmd2Buf);
            break;
          case 27:
            DewMax = atoi(cmd2Buf);
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

//    Serial.println(LenDay);
//    Serial.println(LenBO);
//    Serial.print(F("HtOn "));
//    Serial.println(HtOn);
//    Serial.print(F("HtOff "));
//    Serial.println(HtOff);
//    Serial.println(OHtOn);
//    Serial.println(OHtOff);
//    Serial.println(H2Oon);
//    Serial.println(H2Ooff);
//    Serial.println(RHon);
//    Serial.println(RHoff);
//    Serial.println(ORHon);
//    Serial.println(ORHoff);
//    Serial.println(ORHT);
//    Serial.println(DewMax);

  cmdBuf[0] = {0};                          // 3) Finish by getting the date from server
  cmd2Buf[0] = {0};                         // will save here the date string
  Serial.print(F("AT+CIPSTART=\"TCP\",\""));
  Serial.print(DST_IP);
  Serial.println(F("\",80"));
  //delayFnct();
  if (Serial.find("OK")) {
    delay(10); //LcdDbg("TCPOK");
  }
  delayFnct();
  Serial.readString();
  // Send GET string
  Serial.println(F("AT+CIPSEND=12"));
  delayFnct();  
  if (Serial.find(">"))
  { Serial.print(F("GET date\r\n\r\n"));
    delayFnct();
  }
  if (Serial.find("Date: "))                // get the date line from the http header (sends a fake/wrong GET request)
  { delayFnct();
    Serial.readBytes(cmd2Buf, 29);
  }
  cmd2Buf[30] = {0};                         // will save here the date string
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
  Serial.print(F("\tDate:"));
  Serial.println(cmd2Buf);
}

void delayFnct()  //just to save memory
{
  delay(120);
}

//void finito()  //Not used anymore. Interrupt function  stops main loop if stop button pressed
//  {  exit;  }                                

