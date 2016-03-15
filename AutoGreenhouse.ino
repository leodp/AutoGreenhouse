// Weather logger greenhouse control - 20160310 Leodp@yahoo.com

// Working Setup: -------------------------------
// (possibility of 6x relais (220V) for driving, digital pins 03-08)
// Digit 09 -> I2C multiplex switch, for multiplexing the 2 T-RH sensors
// Digit 10 11 12 13 -> CS SCK MOSI MISO (SD card)

// Analog A4 A5 -> SCL SDA (LCD I2C + 2x T-HR sensors)

// GND, 3.3V   T-HR sensors
// GND, 5.0V   LCD, SD, 2xRelais (220V)

// LCD 2x16, + IIC module, to monitor situation
// 2x HTU21D Temp & Humid sensors
// 1x SD card SPI module, to save data (set values and actual values)
// Mux/Demux Digit 09 for multiplexing the IIC 
// 1x WiFi card to send data to Thingspeak


// SFE_BMP180 pressure module
#include <SFE_BMP180.h>  //pressure sensor
SFE_BMP180 PRESSURE;
#define ALTITUDE 345.0 // Altitude Graz in meters, not used


// LCD and communication
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // arduino-info.wikispaces.com/LCD-Blue-I2C
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// set the LCD address to 0x20 for a 20 chars 2 line display and the pins on the I2C chip used for LCD
  
// Temp Humidity sensor   https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
#include "HTU21D.h" 
#define HTU21D_ADDRESS 0x27 //Unshifted 7-bit I2C address for the sensor
#define TRIGGER_TEMP_MEASURE_NOHOLD 0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD 0xF5
HTU21D myHumidity; //Create an instance of the objects H & T
HTU21D myTemp;

// SD Card
#include <SD.h>
#define file_name "GH1.txt" // log files
File myFile;
// SD card attached to SPI bus as follows:  MOSI - pin 11   MISO - pin 12    CLK - pin 13   CS - pin 10
// http://arduino-info.wikispaces.com/SD-Cards
// https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial
// http://garagelab.com/profiles/blogs/tutorial-how-to-use-sd-card-with-arduino

// ESP8266 - WiFi card
//#include <SoftwareSerial.h>
#define SSID "your_ssid"       //your wifi ssid here
#define PASS "your_password";//your wifi key here
#define myWriteAPIKeyWB  "your_thingspeak_channel_write_key"
#define DST_IP "184.106.153.149"// Thingspeak
String cmd; 
String cmd2="               "; // for the AT command
bool ConnOK=false;
String date_string; //char* 
bool WiFiMOD=true;

//define I/O lines & connections
#define chipSelectSD  10   // SD card CS
#define relay_I2cMultiplex 9 // Switch to enable a second T/Rh sensor on I2C
#define digCh_in 2          // channel for the "input switch" (not used, was for debug purposes)

// Digital lines for controls
#define relay_Heat 8
#define relay_BOut 7
#define relay_Overheat 6
#define relay_Water 5
#define relay_LowRH 4
#define relay_WiFiReset 3 // Hard reset necessary for the dumb ESP8266


// Analog lines for sensing
#define anaCh_dew 0          // channel to sense dew point
#define anaCh_soil 1         // channel to sense soil humidity
#define anaCh_sun 2          // channel to sense day/night (Photodiode)
#define anaCh_rain 3         // channel to sense rain

// General variables
char buf[32]="";//temporary
unsigned int i; // generic index
unsigned long time_begin, time_elapsed;
float T0, T1;
int RH0=0;
int RH1=0;
double T,pressure; //double p0,a; //not used, for equiv sea-level pressure

int sun=0;  // Day 
int rain=0; // rain sensor
float dew=0;  // dew point sensor
float soil=0; // Soil humidity sensor
int SDflag=0; // file open flag
int dayflag=0; // to detect the day
double timeDiffMeas=0; // variable to measure the time passed on the BLOut tent
int dayStateMachine=0; // for controlling opening/closing routine
bool BOUTbool=LOW;
bool heatbool=LOW;
bool overheatbool=LOW;
bool waterbool=LOW;
bool lowRHbool=LOW;


// VALUES TO SET AUTOMATIC CONTROLS
#define HeatingOn  0.5  //Low Temp to start heating
#define HeatingOff 3    //Safe Temp, disable heating

#define LengthDay 11.30*60*60*1000 //Time of illumination, in ms
#define LengthNight 3*60*60*1000   //Time to keep Blackout. During the night it can be opened for air exchange
#define WaitDayConfirm 10          // number of program loops where I detect day has begun. (To avoid jitter, leave about >10)
 
#define OverheatOn  35    //Open window in case of overheating
#define OverheatOff 27    //Close window to keep this minimum Temp safeguarded

#define WaterOn  15  // Turn on auto watering (value from soil humidity sensor)
#define WaterOff 30  // Turn off auto watering

#define RHOn  90  // Max humidity allowed: take precautions to reduce humidity
#define RHOff 60  // Safe humidity level



//  SETUP  /////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  lcd.begin(16,2);// initialize the lcd for 16 chars 2 lines and turn on backlight
  // ------- Quick blinks of backlight  -------------
  for(i = 0; i< 2; i++) {
    lcd.backlight();
    delay(100);
    lcd.noBacklight();
    delay(100);
  }
  lcd.backlight(); // finish with backlight on  
  
  Serial.begin(115200); //speed for ESP8266
  //Serial.println(F("Weather logger")); 

  // Set time to wait for response strings to be found
  Serial.setTimeout(10000);


  pinMode(relay_WiFiReset, OUTPUT);
  pinMode(chipSelectSD, OUTPUT);
  pinMode(relay_I2cMultiplex, OUTPUT);
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
  digitalWrite(relay_Heat, LOW);
  digitalWrite(relay_BOut, LOW); // LOW=0 = not operative = OPEN WINDOW
  digitalWrite(relay_Overheat, LOW); // LOW=0 = not operative = CLOSED
  digitalWrite(relay_Water, LOW); // LOW=0 = not operative = no water
  digitalWrite(relay_LowRH, LOW); // LOW=0 = not operative = no dehumidify
  digitalWrite(chipSelectSD, HIGH);      // SD Card ACTIVE

  PRESSURE.begin();
  //  if (PRESSURE.begin()) //debug in alternative to the above command
  //   { LcdDbg("BMP180 init 1");}
  //  else
  //  {// Oops, something went wrong, this is usually a connection problem,
  //  LcdDbg("BMP180 init 0\n\n");
  //    while(1); // Pause forever.
  //  }

  Serial.begin(115200); //speed for ESP8266
  Serial.setTimeout(5000);
  digitalWrite(relay_WiFiReset, LOW);  // Hard reset of WiFi ESP8266 works better than anything
  delay(500);
  digitalWrite(relay_WiFiReset, HIGH);  
  Serial.println(F("AT+RST"));    
    if(Serial.find("ready"))
    {    //set the connection mode, no mux
      Serial.println(F("AT+CIPMUX=0"));
    //Serial.println("AT+GMR");    // print version number
    Serial.readString();
    //  connect to the wifi
    for(i=0;i<5;i++)
      { connectWiFi();
        if(ConnOK)
          {break;}
      }
    }
    else 
    {Serial.println(F("WiFi module absent"));
     WiFiMOD=false;
    }
      
  SD.begin(chipSelectSD);
  //  if (!SD.begin(chipSelectSD)) {
  //    LcdDbg("SD init 0");
  //    //return;
  //  }
  //  // open the file.
  //  LcdDbg("SD init 1");   
  myFile = SD.open(file_name, FILE_WRITE);
  //Serial.print(F("FOPEN: "));  
  //Serial.println(file_name);
  delay(100);
  //if (!myFile) {
  //  Serial.println("file open failed!");
  //  myFile.close();
  //  return; }
  myFile.print(F("T0\tRH0\tT1\tRH1\tmBar\t"));      // T & RH in, T & RH out, pressure
  myFile.print(F("Dew\tHSoil\tRain\tSun\tBOut\t ")); // Dew, Soil, Rain, Dailight, Blackout
  myFile.println(F("Heat\tHOT\tWater\t<RH\tt_ms\tDate"));     // Heating, Too hot, watering, too hµigh humidity
  delay(100);
  myFile.close();  
  
  time_begin=millis();



}


// Start Loop  ////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

      //  if necessary reconnect to the wifi
      Serial.println(F("AT+CIFSR"));    // does not always work. as expected (detect connection), but helps to reconnect if necessary
      delay(111);
      if(Serial.find("192."))// if it is  connected, IP=192.168... (check your AP settings)
      {
      ConnOK=true;
      delay(1);}//Serial.println(F("Connected!"));}
      else
      { 
        ConnOK=false;
        Serial.println(F("CWQAP")); //be sure to disconnect. Helps to avoid keep-disconnected
        Serial.readString();
        delay(111);
        digitalWrite(relay_WiFiReset, LOW);    // Hard reset of WiFi ESP8266 works better than anything
        delay(500);
        digitalWrite(relay_WiFiReset, HIGH);  
        Serial.println(F("AT+RST"));
        Serial.readString();
        delay(111);
        //set the connection mode, no mux
        Serial.println(F("AT+CIPMUX=0"));
        Serial.readString();
        delay(111);
        connectWiFi();
      }
    
    
    time_elapsed=millis()-time_begin;
    
    // ACQUIRE SENSOR DATA---------------------------------------------------------------
    // ACQUIRE T & RH
    digitalWrite(relay_I2cMultiplex, 0);
    delay(20); // Need some delay to avoid "999" readout
    T0=    myTemp.readTemperature();
    RH0=myHumidity.readHumidity();
    digitalWrite(relay_I2cMultiplex, 1);
    delay(20); // Need some delay to avoid "999" readout
    T1=    myTemp.readTemperature();
    RH1=   myHumidity.readHumidity();

    
    // ACQUIRE PRESSURE
    // Start a temperature measurement:If request is successful, the number of ms to wait is returned, else 0
    buf[0] = PRESSURE.startTemperature();
    if (buf[0] != 0)
    {delay(buf[0]); // Wait for the measurement to complete. Return=0 is fail
    buf[0] = PRESSURE.getTemperature(T);
    if (buf[0] != 0)
    { //LcdDbg(T,2);
      // Start a pressure measurement:The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned, else 0
      buf[0] = PRESSURE.startPressure(3);
      if (buf[0] != 0)
      {delay(buf[0]);// Wait for the measurement to complete:
        // The function requires the previous temperature measurement (T) for calibration. Return=0 is failure
        buf[0] = PRESSURE.getPressure(pressure,T);
        //  if (buf[0] != 0)
        //  { // Sealevel pressure is commonly used in weather reports:
        //   p0 = PRESSURE.sealevel(pressure,ALTITUDE); 
        //   // Inverse: altitude from pressure: P = absolute pressure in mb, p0 = baseline pressure in mb. a in meters
        //   a = PRESSURE.altitude(pressure,p0); }
        //   else LcdDbg("Ep1\n");
      }
      else LcdDbg(F("Ep2\n"));
    }
    else LcdDbg(F("ET1\n"));
  }
  else LcdDbg(F("ET2\n"));


    // ACQUIRE Arduino input channels
    rain=1023-analogRead(anaCh_rain);
    sun=1023-analogRead(anaCh_sun);    
    dew=(analogRead(anaCh_dew))*100.0/1024.0;
    soil=100.0-(analogRead(anaCh_soil))*100.0/1024.0;


    // CONTROLS IN THE GREENHOUSE ------------------------------------------------------------------
    // Heat Control
    if (T0<(float)HeatingOn)
    {heatbool=HIGH;}
    if (T0>(float)HeatingOff)
    {heatbool=LOW;}
    digitalWrite(relay_Heat, heatbool);
    
    // Overheat Control
    if (T0>OverheatOn)
    {overheatbool=HIGH;}
    if (T0<OverheatOff)
    {overheatbool=LOW;}
    digitalWrite(relay_Overheat, overheatbool);
 
    // Watering Control
    if (soil<WaterOn)
    {waterbool=HIGH;}
    if (soil>WaterOff)
    {waterbool=LOW;}
    digitalWrite(relay_Water, waterbool);
 
    // Lower Humidity Control        // http://planetcalc.com/2167/    RH/T proportional to absolute H2O content
    if (RH0>RHOn && RH0/T0>RH1/T1)   // If internal RH too high AND internal_H2O < external_H2=
    {lowRHbool=HIGH;}                // Switching on can decrease internal RH
    if (RH0<RHOff || RH0/T0<RH1/T1)  // If RH inside is low enough or the external absolute humidity is higher =>turn off
    {lowRHbool=LOW;}
    digitalWrite(relay_LowRH, lowRHbool);
 
    
    // Window Control
    if (sun && !dayStateMachine) {dayflag += 1;}                      //Count to avoid jitter on stray light
     if (dayflag==WaitDayConfirm && dayStateMachine==0)               //start recording the day length set
        {timeDiffMeas=time_elapsed;
        dayStateMachine=1;}
     if ((time_elapsed-timeDiffMeas)>LengthDay && dayStateMachine==1) //!BOUTbool  && dayflag)// if we have had enough day
        {BOUTbool=HIGH;
        digitalWrite(relay_BOut, BOUTbool);
        dayStateMachine=2;}
     if ((time_elapsed-timeDiffMeas)>LengthNight+LengthDay && dayStateMachine==2)//BOUTbool)// if we have had enough night open and wait again
        {BOUTbool=LOW;
        digitalWrite(relay_BOut, BOUTbool);
        dayStateMachine=0;
        dayflag=0;}

    
    // START WRITE/UPDATE/UPLOAD routines ---------------------------------------------------------------
    
    date_string="";
    if (ConnOK){update_WiFi ();}
    //update_Serial(); // COMMENTED OUT, IT INTERFERES WITH WiFi card which is on RX-TX Serial lines

    update_LCD   ();
 
    // UPDATE SD card
    myFile = SD.open(file_name, FILE_WRITE);
    delay(50);
    if (!myFile) {SDflag=1;}
    update_SD   ();
    delay(50);
    myFile.close();

    if (digitalRead(digCh_in))  
       {LcdDbg("FINITO");
       exit;} //stops loop if stoop button pressed
    
} // main loop


// ROUTINES-FUNCTIONS  ////////////////////////////////////////////////////////////////////////////////////////

void update_SD() {
    myFile.print(T0);
    myFile.print(F("\t"));
    myFile.print(RH0);
    myFile.print(F("\t"));
    myFile.print(T1);
    myFile.print(F("\t"));
    myFile.print(RH1);
    myFile.print(F("\t"));
    myFile.print(pressure);
    myFile.print(F("\t"));
    myFile.print(dew);
    myFile.print(F("\t"));
    myFile.print(soil);
    myFile.print(F("\t"));
    myFile.print(rain);
    myFile.print(F("\t"));
    myFile.print(sun);
    myFile.print(F("\t"));
    myFile.print(BOUTbool);
    myFile.print(F("\t"));
    myFile.print(heatbool);
    myFile.print(F("\t"));
    myFile.print(overheatbool);
    myFile.print(F("\t"));
    myFile.print(waterbool);
    myFile.print(F("\t"));
    myFile.print(lowRHbool);
    myFile.print(F("\t"));
    myFile.print(time_elapsed);
    myFile.print(F("\t"));
    myFile.println(date_string);


}
  
void update_LCD() {
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print(F("I____'__\%"));
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print(F("O____'__\%d  s"));
  lcd.setCursor(1,0); 
  lcd.print(T0,1);
  lcd.setCursor(1,1); 
  lcd.print(T1,1);
  lcd.setCursor(6,0);
  lcd.print(RH0);
  lcd.setCursor(6,1);
  lcd.print(RH1);
  lcd.setCursor(9,0);
  lcd.print(pressure,1);

  lcd.setCursor(10,1); 
  lcd.print(dew,0);
  
  lcd.setCursor(13,1); 
  lcd.print(soil,0);
  
  lcd.setCursor(15,0); 
  lcd.print(String( (sun>512)*8+(rain>512)*4+(dew>512)*2+BOUTbool*1   ,HEX));
  lcd.setCursor(15,1); 
  lcd.print(String( heatbool*8+overheatbool*4+waterbool*2+lowRHbool*1 ,HEX));
  }



void LcdDbg (String dbgmsg) // not used (for debug purposes, as Serial is used already by WiFi)
  {lcd.clear();
  lcd.setCursor(0,0);
  lcd.println(dbgmsg);
  }



void connectWiFi()
  {
    Serial.println(F("AT+CWMODE=1")); // TO DEBUG: first loop after connection other commands do not work
    Serial.readString();
    delay(111);
    cmd="AT+CWJAP=\"";
    cmd+=SSID;
    cmd+="\",\"";
    cmd+=PASS;
    cmd+="\"";
    Serial.println(cmd);
    delay(4000);
    
    cmd=Serial.readString();
    //Serial.print("COMM----->");
    //Serial.println(cmd);
    
    if(cmd.indexOf('K')>0)//OK
    //if(Serial.find("OK"))//OK
    { Serial.println("WF1"); ///WiFi connection OK=> BUT NOT WORKING
     ConnOK=true;
    }
    else
    {Serial.println(F("EWiFi")); ///WiFi ERROR
     ConnOK=false;
    }            
    Serial.readString();
    delay(111);
  }



void  update_WiFi ()  {  // TO DEBUG: first loop after connection commands do not work


   // Start AT communication to recover date from server 
  date_string="";
   // Start AT communication with Thingspeak
  cmd =F("AT+CIPSTART=\"TCP\",\"");
  cmd += DST_IP; // api.thingspeak.com
  cmd += "\",80";
  Serial.println(cmd);
  delay(111);
  if(Serial.find("OK")) { delay(10);}   //return;}//LcdDbg("TCPOK"); 
  delay(111);
  Serial.readString();
  // Prepare GET string
  cmd = F("GET date\r\n\r\n");
    // CIPSEND
  cmd2 =F("AT+CIPSEND=");
  cmd2 += String(cmd.length());
  Serial.println(cmd2);
  delay(111);  
  if(Serial.find(">"))    
  {Serial.print(cmd);
   delay(111);  
  }
    if (Serial.find("Date: ")) //get the date line from the http header
      {
           delay(111);
           date_string=Serial.readString();
           date_string.remove(31);
           date_string.replace("\n","");
           date_string.replace("\r","");
           Serial.print(F("DATE:"));
           Serial.println(date_string);
      }
  Serial.readString();    
  delay(111);
  

   // Start AT communication with Thingspeak
  cmd =F("AT+CIPSTART=\"TCP\",\"");
  cmd += DST_IP; // api.thingspeak.com
  cmd += "\",80";
  Serial.println(cmd);
  delay(111);
  if(Serial.find("OK")) { delay(10);}   //return;}//LcdDbg("TCPOK"); 
  delay(111);                         
  Serial.readString();

  // Prepare GET string
  // Prepare GET string
  cmd = F("GET /update?api_key=");
  cmd += myWriteAPIKeyWB;
  cmd += F("&field1=");
  cmd += dtostrf(T0, 0, 2, buf);
  cmd += F("&field2=");
  cmd += RH0;//dtostrf(RH0, 0, 0, buf);
  cmd += F("&field3=");
  cmd += dtostrf(T1, 0, 2, buf);
  cmd += F("&field4=");
  cmd += RH1;//dtostrf(RH1, 0, 0, buf);
  cmd += F("&field5=");
  cmd += dtostrf(pressure, 0, 1, buf);
  cmd += F("&field6=");
  cmd += dtostrf(dew, 0, 1, buf);;
  cmd += F("&field7=");
  cmd += dtostrf(soil, 0, 1, buf);
  cmd += F("&field8=");
  cmd += dtostrf((sun>512)*128+(rain>512)*64+(dew>512)*32+BOUTbool*16+heatbool*8+overheatbool*4+waterbool*2+lowRHbool*1, 0, 0, buf);
  cmd += F("\r\n\r\n");

    // CIPSEND
  cmd2 =F("AT+CIPSEND=");
  cmd2 += String(cmd.length());
  Serial.println(cmd2);
  delay(111);  
  if(Serial.find(">"))    
  {Serial.print(cmd); }
  else
  {Serial.println(F("AT+CIPCLOSE")); }
  Serial.readString();
  delay(111);  
}


void update_Serial() {
  //dtostrf(T0,2,2,temp); // dtostrf(value, width_before_comma, precision, output);
  Serial.print(F("T0:"));
  Serial.print(T0);
  Serial.print(F("\tRH0:"));
  Serial.print(RH0);
  Serial.print(F("\tT1:"));
  Serial.print(T1);
  Serial.print(F("\tRH1:"));
  Serial.print(RH1);
  Serial.print(F("\tPress:"));
  Serial.print(pressure);
  Serial.print(F("\tp0:"));
  //Serial.print(p0);
  //Serial.print(F("\tDew:"));
  Serial.print(dew);
  Serial.print(F("\tSoilH:"));
  Serial.print(soil);
  Serial.print(F("\tRain:"));
  Serial.print(rain);
  Serial.print(F("\tDay:"));
  Serial.print(sun);
  Serial.print(F("\tT:"));
  Serial.print(F("\tDate:"));
  Serial.println(date_string);
  //Serial.print(F("\tSDerr:"));
  //Serial.print(SDflag);
  }
