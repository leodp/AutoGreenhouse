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

#define WIFI
//#define SDCARD


#include <avr/wdt.h> // include watchdog timer
#include <SPI.h> // include watchdog timer
#include <SD.h> // include watchdog timer

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
//#define SSID "your_ssid"                                      // edit with your wifi ssid here
//#define PASS "your_pwd"                                       // edit with your wifi key here
//#define myWriteAPIKeyWB  "yourChWriteKey"                     // edit with your thingspeak channel info
//#define myReadAPIKeyWB   "yourChReadKey"                      // edit with your thingspeak channel info  IF I DEFINE A READ KEY I CAN KEEP THE CHANNEL PRIVATE
//#define myChannelID      "yourThingspeak_chID"                // edit with your thingspeak channel info

#define SSID "ldp"                                              // edit with your wifi ssid here
#define PASS "123qwe123"                                        // edit with your wifi key here
#define myWriteAPIKeyWB "LWDLPODZ0DOK1RP5"                      // edit with your thingspeak channel info
#define myReadAPIKeyWB  "GCC75CD4VAR28V7R"                    // edit with your thingspeak channel info  IF I DEFINE A READ KEY I CAN KEEP THE CHANNEL PRIVATE. ATTENTION: LOW MEMORY AVAILABLE MAY LEAD TO FRAGMENTATION
#define myChannelID      "91010"                                // edit with your thingspeak channel info

#define DST_IP "184.106.153.149"                                // Thingspeak API IP address
#define IPWiFiCARD "192"                                        // if it is  connected, IP=192.168... (you may need to edit here, check your AP settings)
#define SERIALspeed 115200                                      // Speed for ESP8266, in different firmwares may be different
char cmdBuf[54];                                                // for the AT command. Check that Thingspeak description length is shorted than this buffer len
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
unsigned char h, i, j;                   // generic index
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
unsigned long timeDiffMeas = 0;          // variable to measure the time passed on the Black out tent
unsigned char dayStateMachine = 0;// for controlling blackout opening/closing routine
byte CtrlByte  = B00000000;       // http://playground.arduino.cc/Code/BitMath some booleans for recording the status of the controls: sun,rain,humidify,blackOut,heating,overheated,watering,reduceRH


// VALUES TO SET AUTOMATIC CONTROLS ------------------------  EDIT HERE TO SUIT YOUR NEEDS ----------------------

double Day  = 11.50;             // Time of illumination/allowed daylength (in h, first value indicate the effective hours)
double BO =   6.0;               // Time to keep Blackout. During the night it can be opened for air exchange. Increase according to your needs (in latitudes far from the equator it must be longer, to be sure we are into the night when BO ends)
#define DayJitt   30             // Number of day-begin detections. (To avoid jitter, due for example to short light pulses, i.e. lights on for 1min during night visits)
                                 // No PWM on BOut motor, as 1 and 0 control the two motor directions (open/close)
double Ht1 = 0.3;                // Too low Temp: start heating. Need float precision on temp warning close to 0°C
double Ht0 = 4.0;                // Safe Temp, stop heating

char OHt1 = 38;                  // Open window in case of overheating: max acceptable temp before plants suffer
char OHt0 = 27;                  // Close window to keep this minimum Temp safeguarded
#define overheatPWM   128        // PWM controls for the motors. Set to 255 for always on/always off

char H2O1 = 15;                  // Turn on auto watering (value from soil humidity sensor)
char H2O0 = 30;                  // Turn off auto watering
unsigned char H2OT = 30;         // Auto watering switch-on time. Set to 0 to switch on auto-watering based only on soil humidity. In minutes. Max 255

char RH1 = 15;                   // Too low Temp: start heating. Need float precision on temp warning close to 0°C
char RH0 = 35;                   // Safe Temp, stop heating

char ORH1    = 60;               // Max humidity allowed: take precautions to reduce humidity (open windows, ventilate with external air)
char ORH0   = 55;                // Safe humidity level (reduced molds/mildew risk): switch off ventilation.
char ORHT  = 5;                  // If temperature falls below this value then stop ventilation to avoid undercooling, even if RH is high
char ODew = 94;                  // If the dew sensor is indicating condensation, you may want to start ventilation. Increase above 100 to disable
#define lowRHPWM   128           // PWM controls for the motors. Set to 255 for always on/always off

// Next line to be entered in Thingspeak channel description
// ;11.5 3.8 6 10 20 15 15 25 30 58 65 80 70 5 85 ;LenDay LenBO Ht1 Ht0 OHt1 OHt0 H2O1 H2O0 H2OT RH1 RH0 ORH1 ORH0 ORHT ODew

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

  h=0;
  
  PRESSURE.begin();
  //  if (PRESSURE.begin()) //debug in alternative to the above command
  //   { LcdDbg("BMP180 init 1");}
  //  else
  //   { LcdDbg("BMP180 init 0\n\n");// Problema
  //    while(1); // Pause forever.
  //   }

  Serial.begin(SERIALspeed);           // Start serial communication with WIFi module.
  Serial.setTimeout(5000);             // Set time to wait for response strings to be found. Keep high to avoid WiFi problems (i.e. long connect/reply times)

  #ifdef WIFI  
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
  #endif

  #ifdef SDCARD
  SD.begin(chipSelectSD);                                // Start writing to file in microSD card
  myFile = SD.open(file_name, FILE_WRITE);
  //delayFnct();
  //if (!myFile) {
  //  Serial.println("file open failed!");
  //  myFile.close();
  //  return; }
  myFile.println(F("Ti\tRHi\tTo\tRHo\tmBar\tRain\tHSoil\tDew\tSn_Rn_BO_Ht_<Ht_H2O_RH_<RH\tTime[s]\tDate")); // Header: T & RH in, T & RH out, pressure, Rain, Soil, Dew, Dailight, Blackout, Heating, Too hot, watering, reduce humidity, time since start, date from network
  myFile.close();
  #endif
  wdt_enable(WDTO_8S);  // enable watchdog timer 8 seconds
}


// Main Loop  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  h=h+1;

  wdt_reset();
  #ifdef WIFI
  if (h==2)  //Execute WiFi & SDCARD logging only once every n loop (but LCD & Serial get updated more often)
   {
    //  if necessary reconnect to the wifi  (connection may be lost due to a weak or temporary WiFi signal)
    Serial.println(F("AT+CIFSR"));        // does not always work as expected (detect connection), but helps to reconnect if necessary
    delayFnct();
    if (Serial.find(IPWiFiCARD))
    { ConnOK = true;                     // It's connected, no need to do anything Serial.println(F("Connected!"));}
      //wdt_reset()
      ;}
    else
    {
      ConnOK = false;
      Serial.println(F("AT+CWQAP"));         // Be sure to disconnect. Helps to avoid keep-disconnected
      wdt_reset();
      Serial.readString();
      //delayFnct();
      digitalWrite(relay_WiFiReset, LOW); // Hard reset of WiFi ESP8266 works better than anything to clear "Busy p..." lock-ups
      delay(500);
      digitalWrite(relay_WiFiReset, HIGH);
      wdt_reset();
      Serial.println(F("AT+RST"));
      Serial.readString();
      wdt_reset();
      delayFnct();
      Serial.println(F("AT+CIPMUX=0"));   // Set the connection mode, no mux
      wdt_reset();
      connectWiFi();
    }
   }
  #endif 
  
  // ACQUIRE SENSORS DATA---------------------------------------------------------------
 
  // ACQUIRE PRESSURE
  wdt_reset();
  cmdBuf[0] = {0};
  cmdBuf[0] = PRESSURE.startTemperature();                 // Start a temperature measurement:If request is successful, the number of s to wait is returned, else 0
  if (cmdBuf[0] != 0)
  { delay(cmdBuf[0]);                                      // Wait for the measurement to complete. Return=0 is fail
    cmdBuf[0] = PRESSURE.getTemperature(Ti);
    if (cmdBuf[0] != 0)
    { cmdBuf[0] = PRESSURE.startPressure(3);               // Start a pressure measurement:The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).If request is successful, the number of s to wait is returned, else 0
      if (cmdBuf[0] != 0)
      { delay(cmdBuf[0]);                                  // Wait for the measurement to complete:
        cmdBuf[0] = PRESSURE.getPressure(pressure, Ti);     // The function requires the previous temperature measurement (T) for calibration. Return=0 is failure
        //  if (cmdBuf[0] != 0)
        //  { p0 = PRESSURE.sealevel(pressure,ALTITUDE);    // Sealevel pressure is commonly used in weather reports:
        //    a = PRESSURE.altitude(pressure,p0); }         // Inverse: altitude from pressure: P = absolute pressure in mb, p0 = baseline pressure in mb. a in meters
        //  else { LcdDbg("Ep1\n"); }                       // Do something in case of error
      } //else LcdDbg(F("Ep2\n"));                            // Here and following: print error messages on LCD for debug
    }   //else LcdDbg(F("ET1\n"));
  }     //else LcdDbg(F("ET2\n"));


  // ACQUIRE T & RH
  wdt_reset();
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

  

  // ACQUIRE Arduino analog input channels
  wdt_reset();
  rain = ( (int) (1023 - analogRead(anaCh_rain))/10.23 );         // Rescale. A high value is indicating rain (max 100)
  sun =  ( (int) (1023 - analogRead(anaCh_sun))/10.23  ) ;        // Rescale. A high value is indicating sunlight (max 100)
  dew =  ( (int) (analogRead(anaCh_dew))/10.23 );                 // Rescale. A high value is indicating condensation (max 100)
  soil = ( (int) (1023 - analogRead(anaCh_soil))/10.23 );         // Rescale. Value is indicating how much water is present in the soil (max 100)
  CtrlByte &=  63 ;
  CtrlByte |= ( ((sun/50)<<7)+ ((rain/50)<<6)  );// Log digital sun/rain
  

  // CONTROLS IN THE GREENHOUSE ---------------------------------------------------------------------------------------------------------------------

  wdt_reset();
  
  // Black-out control, to force plants via light (growing/flowering/fruiting/dormiency/...)--------------------------------------
  if (CtrlByte>>7 && !dayStateMachine) {
    dayflag += 1; // Count to avoid jitter on stray light, allows for max DayJitt wrong light detections
  }
  if (!(CtrlByte>>7) && !dayStateMachine) {
    dayflag=max(0,dayflag-1);
  }
  if (dayflag == DayJitt && dayStateMachine == 0)                             // Start recording the day length, up to when we have reached the allowed illumination time
  { timeDiffMeas = millis();                                                  // http://www.leonardomiliani.com/2012/come-gestire-loverflow-di-millis/
    dayStateMachine = 1;
  }
  if ( (unsigned long)(millis() - timeDiffMeas) > (unsigned long)((float)(Day*60*60*1000)) && dayStateMachine == 1)     // If we have had enough day turn on the blackout system. May be connected also to a small air-circulation fan, too keep low T & RH
  { CtrlByte |= B00100000;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 2;
  }
  if ( (unsigned long)(millis() - timeDiffMeas) > (unsigned long)((float)((BO + Day)*60*60*1000)) && dayStateMachine == 2) // After a while open again the blackout tent, for easing ventilation (it should be already night outside). BE SURE THAT IT'S DARK WHEN YOU REOPEN!!
  { CtrlByte &= B11011111;
    digitalWrite(relay_BOut, (CtrlByte>>5)&1);
    dayStateMachine = 3;
    dayflag = 0;
  }

  //Serial.print(CtrlByte>>7);  //debug
  //Serial.print(F("\t"));
  //Serial.print(dayflag);
  //Serial.print(F("\t"));
  //Serial.println(dayStateMachine);
  //Serial.print((unsigned long)(millis()));
  //Serial.print(F("\t"));
  //Serial.print((unsigned long)(millis() - timeDiffMeas));
  //Serial.print(F("\t"));
  //Serial.print(  (unsigned long) ( (float)(Day*60*60*1000) )  );
  //Serial.print(F("\t"));
  //Serial.print(  (unsigned long) ( (float)((BO + Day)*60*60*1000) )  );
  //Serial.print(F("\t"));
  //Serial.println((unsigned long)(((long)H2OT)*60*1000));
  //wdt_reset();
  //Serial.readString();
  //delayFnct();
  //wdt_reset();

  // Heat Control--------------------------------------
  if (Ti < (float)Ht1)
  { CtrlByte |= B00010000;}
  if (Ti > (float)Ht0)         // Heating On and Off have different values, to allow for hysteresis
  { CtrlByte &= B11101111;}
  digitalWrite(relay_Heat, (CtrlByte>>4)&1);


  // Overheat Control--------------------------------------
  if (Ti > (float)OHt1)                // Overheating indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00001000;}
  if (Ti < (float)OHt0)
  { CtrlByte &= B11110111;}
  analogWrite(relay_Overheat, ((CtrlByte>>3)&1)*overheatPWM);


  // Watering Control--------------------------------------
  if ((int)H2OT==0)
    {  if ((int)soil < (int)H2O1)                 // Watering On and Off have different values, to allow for hysteresis
          { CtrlByte |= B00000100;}
       if ((int)soil > (int)H2O0)
          { CtrlByte &= B11111011;}
    }
   else 
    {  if( dayStateMachine == 1)                     // if START DAY:timeDiffMeas. Watering at the first sunlights, to avoid nightwatering/snails
        {  CtrlByte |= B00000100; }                  // then switch on
       if ((unsigned long)(millis() - timeDiffMeas) > (unsigned long)(((long)H2OT)*60*1000)) // If watering has been on for long enough (in minutes)
        {  //Serial.println(F("WATER OFF"));
           CtrlByte &= B11111011; }                  // switch off
    }
  digitalWrite(relay_Water, (CtrlByte>>2)&1);


  // Humidification Control--------------------------------------
  if ((int)RHi < (int)RH1)              // Humidifying indications On and Off have different values, to allow for hysteresis
  { CtrlByte |= B00000010;}
  if ((int)RHi > (int)RH0)
  { CtrlByte &= B11111101;}
  digitalWrite(relay_Humidify, (CtrlByte>>1)&1);

  
  // Control to lower the air humidity -----------------------------------                                   // http://planetcalc.com/2167/    https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
  if (   ( (int)RHi > (int)ORH1   || (int)dew > (int)ODew   )  &&  (int)RHi > (int)RHo  && To > (float)ORHT) // If [(internal_RH too high) OR condensation happening] AND external RH not higher (raining) external temperature not too low
  { CtrlByte |= B00000001;}                                                                                  // Switching on can decrease internal RH
  if (   ( (int)RHi < (int) ORH0   && (int)dew < (int)ODew   )  &&  (int)RHi < (int)RHo  || To < (float)ORHT) // If RH inside is low enough OR the external absolute humidity is higher OR external T is too low =>turn off
  { CtrlByte &= B11111110;}
  analogWrite(relay_LowRH, (CtrlByte&1)*lowRHPWM);
  

  // WRITE/UPDATE/UPLOAD routines -------------------------------------------------------------------------------------------------------
  
  //Serial.println(F("UPDATING"));
  wdt_reset();

  if (h==2)  //Execute WiFi & SDCARD logging only once every n loop (but LCD & Serial get updated more often)
    {
    h=0;  
    #ifdef WIFI
    if (ConnOK) 
      { wdt_reset();
        update_WiFi ();}// Thingspeak update routine    PUTS DATE IN cmdBuf
    #endif 
    
    wdt_reset();
  
    // UPDATE SD card
    #ifdef SDCARD
    wdt_reset();
    myFile = SD.open(file_name, FILE_WRITE);
    //if (!myFile) {SDflag=1;}  
    update_SD   ();
    myFile.close();
    wdt_reset();
    #endif
    }
    
  wdt_reset();
  update_LCD   ();
  wdt_reset();
  update_Serial();  // COMMENTED OUT, IT MAY INTERFERE WITH WiFi card which is also on RX-TX Serial lines. Un-comment on for debug
  wdt_reset();

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
  #ifdef WIFI
  myFile.print(F("\t"));
  myFile.println(cmdBuf);
  #endif
  myFile.flush();
  //Serial.println(F("UPDATED_SD"));
}

void update_LCD() {                 // LCD display routine
  lcd.clear();
  lcd.setCursor(0, 0);              // Start at character 0 on line 0
  lcd.print(F("I____'__\%"));       // LCD preparation of indications: I(internal) '(degrees C) %(RH%)
  lcd.setCursor(0, 1);              // Start at character 0 on line 1
  lcd.print(F("O____'__\%d  s"));   // LCD preparation of indications:  O(outside) '(degrees C) %(RH%) d(dewmax) s(soil)
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
  lcd.print((int)dew);
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
{ wdt_reset();
  Serial.readString();
  delayFnct();
  wdt_reset();
  Serial.println(F("AT+CWMODE=1"));
  Serial.readString();
  delayFnct();
  wdt_reset();
  Serial.print(F("AT+CWJAP=\""));
  Serial.print(SSID);
  Serial.print(F("\",\""));
  Serial.print(PASS);
  Serial.println(F("\""));
  wdt_reset();
  delay(4500);                    // Give time to connect to the WiFi
  wdt_reset();

  if(Serial.find("OK"))       
  { //Serial.println(F("WF1"));     // WiFi connection OK
    ConnOK = true;
  }
  else
  { //Serial.println(F("EWiFi"));   //WiFi ERROR
    ConnOK = false;
  }
  wdt_reset();
  Serial.readString();
  delayFnct();
  wdt_reset();
  }


void  update_WiFi ()
{ wdt_reset();
  Serial.print(F("AT+CIPSTART=\"TCP\",\""));    // 1) Update the values on thingspeak with measured data
  Serial.print(DST_IP);                      // api.thingspeak.com
  Serial.println(F("\",80"));
  wdt_reset();
  ///delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  wdt_reset();
  Serial.readString();                      // Prepare GET string
  wdt_reset();
  cmdBuf[0] = {0};


  // CIPSEND
  Serial.println(F("AT+CIPSEND=150"));  // prepare for a message of 150 characters
  wdt_reset();
  
  i=150;
  if (Serial.find(">"))
    {   wdt_reset();
        Serial.print(F("GET /update?api_key="));
        Serial.print(myWriteAPIKeyWB);
        Serial.print(F("&field1="));
        Serial.print(dtostrf( Ti , 0, 2, buf));
        i -=strlen(dtostrf( Ti , 0, 2, buf));
        Serial.print(F("&field2="));
        Serial.print(dtostrf( RHi , 0, 2, buf));
        i -=strlen(dtostrf( RHi , 0, 2, buf));
        Serial.print(F("&field3="));
        Serial.print(dtostrf( To , 0, 2, buf));
        i -=strlen(dtostrf( To , 0, 2, buf));
        Serial.print(F("&field4="));
        Serial.print(dtostrf( RHo , 0, 2, buf));
        i -=strlen(dtostrf( RHo , 0, 2, buf));
        Serial.print(F("&field5="));
        Serial.print(dtostrf( pressure , 0, 2, buf));
        i -=strlen(dtostrf( pressure , 0, 2, buf));
        Serial.print(F("&field6="));
        Serial.print(dtostrf( dew , 0, 2, buf));
        i -=strlen(dtostrf( dew , 0, 2, buf));
        Serial.print(F("&field7="));
        Serial.print(dtostrf( soil , 0, 2, buf));
        i -=strlen(dtostrf( soil , 0, 2, buf));
        Serial.print(F("&field8="));
        Serial.print(dtostrf( (int)CtrlByte , 0, 0, buf)); // digital flags to log
        i -=strlen(dtostrf( (int)CtrlByte , 0, 0, buf));
        i=i-20-(8*8)-4; // add the  GET /update... + "&fieldn=" + newlines string lengths
        for (j=0; j<i;j++)
          {Serial.print(F(" "));} // add the necessary whitespaces to fill the 150 characters
        Serial.print(F("\r\n\r\n"));
    }
  else
    { //Serial.print(cmdBuf);
      wdt_reset();
      Serial.println(F("AT+CIPCLOSE")); }
  Serial.readString();
  delayFnct();
  wdt_reset();

  

  buf[0] = {0};
  cmdBuf[0] = {0};
  Serial.print(F("AT+CIPSTART=\"TCP\",\""));    // 2) Start AT communication with Thingspeak to read channel description (contains controls values to be updated remotely)
  Serial.print(DST_IP);                      // api.thingspeak.com
  Serial.println(F("\",80"));
  wdt_reset();
  ///delayFnct();
  if (Serial.find("OK")) 
    { delayFnct();  }
  wdt_reset();
  Serial.readString();               
  Serial.println(F("AT+CIPSEND=28"));      // Prepare GET string
  //Serial.println(F("AT+CIPSEND=49"));      // Prepare GET string
  wdt_reset();
  delayFnct();  
  if (Serial.find(">"))
  { //wdt_reset();
    Serial.print(F("GET /channels/91010.json\r\n\r\n")); 
    //Serial.print(F("GET /channels/91010.json?key="));   // these 3 lines instead of the above if you want to keep the channel private
    //Serial.print(myReadAPIKeyWB);
    //Serial.print(F("\r\n\r\n")); 
    //delayFnct();
  }
  
  wdt_reset();
  //delayFnct();
  if (Serial.find("__"))//(Serial.find("tion\":\";"))
  {wdt_reset();
      Serial.readBytes(cmdBuf, 53);//   }  // read the incoming bytes
      for (i=0;i++;i<53)
      {if (cmdBuf[i]==';')  {cmdBuf[i-1]={0};
                             break;}
      }
      //i = Serial.readBytesUntil(';', cmdBuf, 53); //read quickly all variables (avoids overflow)
      //wdt_reset();
      ///cmdBuf[i] = {0};
      //Serial.print(F("I:"));
      //Serial.println(i);
      Serial.print(F("PARAMS:"));
      Serial.println(cmdBuf);
      j = 0;
      k = -1;
      for (i = 0; i <= strlen(cmdBuf); i++)
      { buf[j] = cmdBuf[i];
        if (cmdBuf[i] == ' ') // Found separator
        { buf[j] = {0};
          k = k + 1;
          j = 0;
          switch (k)
          { case 0:                    /// ATTENTION: NO CHECK THAT VALUES ARE ACCEPTABLE, ENTER CORRECT VALUES IN THINGSPEAK DESCRIPTION!!!!
              Day =  atof(buf); break;
            case 1:
              BO =   atof(buf);  break;
            case 2:
              Ht1 =  atof(buf);  break;
            case 3:
              Ht0 =  atof(buf);  break;
            case 4:
              OHt1 = atoi(buf);  break;
            case 5:
              OHt0 = atoi(buf);  break;
            case 6:
              H2O1 = atoi(buf);  break;
            case 7:
              H2O0 = atoi(buf);  break;
            case 8:
              H2OT = atoi(buf);  break;
            case 9:
              RH1 =  atoi(buf);  break;
            case 10:
              RH0 =  atoi(buf);  break;
            case 11:
              ORH1 = atoi(buf);  break;
            case 12:
              ORH0 = atoi(buf);  break;
            case 13:
              ORHT = atoi(buf);  break;
            case 14:
              ODew = atoi(buf);  break;
            default:             
                                 break;
          }       // switch case on recognized "word"
        }         // if loop on recognized separator, "word" completed
        else
        {j=j+1;}
      }           // for loop on string length
  }              // loop when serial found ";"
  else
  { wdt_reset();
    Serial.println(F("AT+CIPCLOSE")); } //Serial.println("DESCRIPTION NOT FOUND");
  wdt_reset();
  Serial.readString();      // close all communication queues
  delayFnct();
  wdt_reset();
  
  Serial.print(F("\t"));Serial.print(F("Day:"));Serial.print((float)Day);
  Serial.print(F("\t"));Serial.print(F( "BO:"));Serial.print((float)BO);
  Serial.print(F("\t"));Serial.print(F("Ht1:"));Serial.print((float)Ht1);
  Serial.print(F("\t"));Serial.print(F("Ht0:"));Serial.print((float)Ht0);
  Serial.print(F("\t"));Serial.print((int)OHt1);
  Serial.print(F("\t"));Serial.print((int)OHt0);
  Serial.print(F("\t"));Serial.print((int)H2O1);
  Serial.print(F("\t"));Serial.print((int)H2O0);
  Serial.print(F("\t"));Serial.print((int)H2OT);
  Serial.print(F("\t"));Serial.print((int)RH1);
  Serial.print(F("\t"));Serial.print((int)RH0);
  Serial.print(F("\t"));Serial.print((int)ORH1);
  Serial.print(F("\t"));Serial.print((int)ORH0);
  Serial.print(F("\t"));Serial.print((int)ORHT);
  Serial.print(F("\t"));Serial.println((int)ODew);
  wdt_reset();
  Serial.readString();      // close all communication queues
  delayFnct();
  wdt_reset();


  
  Serial.print(F("AT+CIPSTART=\"TCP\",\""));    // 3) Finish by getting the date from server
  Serial.print(DST_IP);                         // api.thingspeak.com
  Serial.println(F("\",80"));
  wdt_reset();
  //delayFnct();
  //if (Serial.find("OK")) {
  //  delay(10); //LcdDbg("TCPOK");
  //}
  Serial.readString();
  delayFnct();
  wdt_reset();
  // Send GET string
  Serial.println(F("AT+CIPSEND=12"));
  //delayFnct();  
  if (Serial.find(">"))
  { Serial.print(F("GET date\r\n\r\n"));
    ///delayFnct();
  }
  wdt_reset();
  cmdBuf[0] = {0};                          // will save here the date string                    
  if (Serial.find("te: "))                  // get the date line from the http header (sends a fake/wrong GET request)
  { wdt_reset();
    //delayFnct();
  //  if (Serial.available() > 0) 
  //    { 
        wdt_reset();
        Serial.readBytes(cmdBuf, 29);//   }  // read the incoming bytes
  }
  cmdBuf[29] = {0};    
  wdt_reset();
  Serial.readString();
  delayFnct();
  wdt_reset();
  Serial.print(F("DATE:"));
  Serial.println(cmdBuf);
  Serial.readString();
  delayFnct();
  wdt_reset();
  }



void update_Serial()           // ATTENTION Possible interference with WiFi module.
{ wdt_reset();
  Serial.print(F("Ti:"));
  Serial.print(Ti);            //eventually dtostrf(Ti,2,2,temp); dtostrf(value, width_before_comma, precision, output);
  Serial.print(F("\tRHi:"));
  Serial.print((int)RHi);
  Serial.print(F("\tTo:"));
  Serial.print(To);
  Serial.print(F("\tRHo:"));
  Serial.print((int)RHo);
  Serial.print(F("\tPress:"));
  Serial.print(pressure);
  //Serial.print(F("\tp0:"));  //if p0 enabled you may want to log the equivalent pressure the at-sea-level
  //Serial.print(p0);
  Serial.print(F("\tRain:"));
  Serial.print((int)rain);
  Serial.print(F("\tSoilH:"));
  Serial.print((int)soil);
  Serial.print(F("\tDew:"));
  Serial.print((int)dew);
  Serial.print(F("\tDay:"));
  Serial.print((int)sun);
  Serial.print(F("\tCtrlByte:"));
  Serial.print((int)CtrlByte);
  Serial.print(F("\tTime[s]:"));
  Serial.print(millis()/1000);
  Serial.print(F("\tDate:"));
  Serial.println(cmdBuf);
  Serial.readString(); //clean the buffer and the communication
  delayFnct();
  wdt_reset();
}

void delayFnct()  //just to save memory
{ delay(120);}
