/*
   Receiver which receivers messages through a Hope RFM95 LoRa
   Module.
   It sends a feedback signal (checksum) to the receiver

*/

//FDRS

#include "fdrs_gateway_config.h"
#include <fdrs_gateway.h>


extern DataReading myCtrlData[256];
extern uint8_t myCtrl;
//end FDRS

long timeOut = 0;
int sendStatetimeOut = 80000;
int counter;
int forceSleep = 0;
boolean hasReceivedCmd = false;

enum {idle, waitingTime, sendingSensor, sleeping};        // 0: idle, 1: sendingTime, 2: waitingSensor, 3: sleeping
int GtwStatus = idle;

float M1, M2, M3, M4;                           // Moisture sensors 1 to 4 (beware M2, M3, M4 does not work if motor drivers are soldered...)

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int timeToSleep;                    /* Time ESP32 will go to sleep (in seconds) */


#define ENA_9V_PIN 32
#define LED_PIN 22

// motor driver 1
#include <ESP32MX1508.h> //https://github.com/ElectroMagus/ESP32MX1508
#define IN1_M1_PIN  13
#define IN2_M1_PIN  15
#define IN3_M1_PIN  2
#define IN4_M1_PIN  0
#define CH1 0                   // 16 Channels (0-15) are available
#define CH2 1                   // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7
#define IN1_M2_PIN  4
#define IN2_M2_PIN  16
#define IN3_M2_PIN  17
#define IN4_M2_PIN  5
// Optional Parameters
#define RES 8                   // Resolution in bits:  8 (0-255),  12 (0-4095), or 16 (0-65535)     
#define FREQ  5000              // PWM Frequency in Hz    

MX1508 C1(IN1_M1_PIN, IN2_M1_PIN, CH1, CH2);    // coil1
MX1508 C2(IN3_M1_PIN, IN4_M1_PIN, CH3, CH4);    // coil2
MX1508 C3(IN1_M2_PIN, IN2_M2_PIN, CH5, CH6);    // coil3
MX1508 C4(IN3_M2_PIN, IN4_M2_PIN, CH7, CH8);    // coil4

//RTC DS1302
#include <Ds1302.h>

#define ENA_PIN 18
#define CLK_PIN 19
#define DAT_PIN 23
// DS1302 RTC instance
Ds1302 rtc(ENA_PIN, CLK_PIN, DAT_PIN);


const static char* WeekDays[] =
{
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday",
  "Sunday"
};

#define VCC_PIN 36    //ADC pin for solar panel voltage measurement



String ssid = "";
String password = "";
boolean hasWifiCredentials = false;
boolean hasNtpTime = false;                 //UTC time not acquired from NTP

//these variable remain in RTC memory even in deep sleep or after software reset (https://github.com/espressif/esp-idf/issues/7718)(https://www.esp32.com/viewtopic.php?t=4931)
RTC_NOINIT_ATTR boolean hasRtcTime = false;   //UTC time not acquired from smartphone
//RTC_DATA_ATTR boolean hasRtcTime = false;   //will only survice to deepsleep reset... not software reset
RTC_NOINIT_ATTR int hours;
RTC_NOINIT_ATTR int seconds;
RTC_NOINIT_ATTR int tvsec;
RTC_NOINIT_ATTR int minutes;
RTC_NOINIT_ATTR int days;
RTC_NOINIT_ATTR int months;
RTC_NOINIT_ATTR int years;


//time
#include <TimeLib.h>

//Preferences
#include <Preferences.h>
Preferences preferences;



#define DEBUG_WIFI   //debug Wifi 
#define DEBUG_SLEEP
#define DEBUG_UDP    //broadcast info over UDP
#define DEBUG_PREFS  //debug preferences
#define DEBUG_VCC
//#define DEBUG


//asyncUDP
#if defined DEBUG_UDP
#include <AsyncUDP.h>
AsyncUDP broadcastUDP;
#endif

//********************
//code starts here
//********************


void display_time(void)
{
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.print(" at ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
}



void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);  //led

  //Preferences
  preferences.begin("Rezodo", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  preferences.putInt("timeToSleep", 5);                 // reset time to sleep to 10 minutes (usefull after night)
  timeToSleep = preferences.getInt("timeToSleep", 10);


  //enable deepsleep for ESP32
  //  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                 //allow timer deepsleep
  //esp_sleep_enable_touchpad_wakeup();                                           //allow to wake up with touchpads

  //FDRS
  beginFDRS();
  DBG("start REZODO Lora gateway");

  // initialize the RTC
  rtc.init();

  // test if clock is halted and set a date-time (see example 2) to start it
  if (rtc.isHalted())
  {
    DBG("RTC is halted...");
    if (hasRtcTime)
    {
      Serial.println("use time from ESP32 RTC :");
      struct timeval current_time;
      gettimeofday(&current_time, NULL);
      // Serial.printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      //Serial.printf("seconds stored : %ld\nnow seconds : %ld\n", tvsec, current_time.tv_sec);
      int sec  = seconds - tvsec + current_time.tv_sec ;
      sec = hours * 3600 + minutes * 60 + sec;
      int ss = sec % 60;
      sec = sec / 60;
      int mm = sec % 60;
      sec = sec / 60;
      int hh = sec % 24;
      int dd = days + sec / 24;
      //set time manually (hr, min, sec, day, mo, yr)
      setTime(hh, mm, ss, dd, months, years);
      display_time();
    }
  }
  else
  {
    // get the current time
    Ds1302::DateTime now;
    rtc.getDateTime(&now);
    years = now.year + 2000;
    months = now.month;
    days = now.day;
    hours = now.hour;
    minutes = now.minute ;
    seconds = now.second;
    setTime(hours, minutes, seconds, days, months, years); //set ESP32 time manually
    DBG("use time from DS1302 RTC : ");
    display_time();
    struct timeval current_time;       //get ESP32 RTC time and save it
    gettimeofday(&current_time, NULL);
    tvsec  = current_time.tv_sec ;      //seconds since reboot (now stored into RTC RAM
    hasRtcTime = true;                  //now ESP32 RTC time is also initialized
  }


  //water coils
  pinMode(ENA_9V_PIN, INPUT);  //disable stepUp booster 9V (by pulldown resistor)
  C1.motorStop();             // Soft Stop    -no argument
  C2.motorStop();             // Soft Stop    -no argument
  C3.motorStop();             // Soft Stop    -no argument
  C4.motorStop();             // Soft Stop    -no argument


  //soil moisture probes initialization
  Serial.println("measuring moisture");
  touch_pad_init();
  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  touch_pad_config(TOUCH_PAD_NUM8, 0); //T8
  // touch_pad_config(TOUCH_PAD_NUM0, 0); //T0 does not work if coil driver is soldered(will work if not soldered)
  // touch_pad_config(TOUCH_PAD_NUM2, 0); //T2 does not work if coil driver is soldered(will work if not soldered)
  // touch_pad_config(TOUCH_PAD_NUM3, 0); //T3 does not work if coil driver is soldered(will work if not soldered)

  //humidity sensors: read touch output
  uint16_t output;
  touch_pad_read(TOUCH_PAD_NUM8, &output);  //T8 or M1
  M1 = float(output);
  
  timeOut = millis();
}

void activateCoil(int num, boolean openCoil)
{
  Serial.print("activate coil ");
  Serial.print(num);
  Serial.print(" value ");
  Serial.println(openCoil);
  switch (num)
  {
    case 1:
      if (openCoil)  C1.motorGo(255);
      else C1.motorRev(255);
      delay (250);
      C1.motorStop();
      break;
    case 2:
      if (openCoil)  C2.motorGo(255);
      else C2.motorRev(255);
      delay (250);
      C2.motorStop();
      break;
    case 3:
      if (openCoil)  C3.motorGo(255);
      else C3.motorRev(255);
      delay (250);
      C3.motorStop();
      break;
    case 4:
      if (openCoil)  C4.motorGo(255);
      else C4.motorRev(255);
      delay (250);
      C4.motorStop();
      break;
    default:
      // statements
      break;
  }
}

void loop()
{
  loopFDRS();
  if (myCtrl > 0) //we have received at least a control message for this gateway
  {

    timeOut = millis();
    for (int i = 0 ; i < myCtrl; i++) //for each control message
    {
      int id = myCtrlData[i].id >> 8;                 //extract the GTW id
      int index = myCtrlData[i].id & 0xFF;            //extract the sensor index
      myCtrlData[i].id = 0x8000;                      //reset the id to unused value (eat the message)
      if (id == UNIT_MAC)
      {
        switch (index)
        {
          case 1:  //irrigation coil
          case 2:
          case 3:
          case 4:
            if (myCtrlData[i].t == 1) //activate coil
            {
              GtwStatus = waitingTime;
              hasReceivedCmd = true;        //this will disable security shutdown during deepsleep
              pinMode(ENA_9V_PIN, OUTPUT);  //enable stepUp booster 9V
              digitalWrite(ENA_9V_PIN, HIGH);
              delay(100);
              activateCoil(index, myCtrlData[i].d); //index is the coil to activate and .d is the value (0 off, 1 On)
              pinMode(ENA_9V_PIN, INPUT);  //disable stepUp booster 9V
            }
            break;
          case 0xFF:  //time synchro
            sendStatetimeOut = 15000;  //decrease the timeout value
            timeOut = millis();   //reset the timeout
            GtwStatus = sendingSensor;  //ok to send sensor values
            hours =  floor( myCtrlData[i].d / 3600);
            minutes = floor((myCtrlData[i].d - hours * 3600) / 60);
            seconds = myCtrlData[i].d - hours * 3600 - minutes * 60;
            days = 1;
            months = 6;
            years = 2023;

            //set ESP32 time manually (hr, min, sec, day, mo, yr)
            setTime(hours, minutes, seconds, days, months, years);
            Serial.print("time from GTW0: ");
            display_time();

            struct timeval current_time;        //get ESP32 RTC time and save it
            gettimeofday(&current_time, NULL);
            tvsec  = current_time.tv_sec ;      //seconds since reboot
            hasRtcTime = true;                  //now ESP32 RTC time is also initialized

            //            //set DS1302 RTC time
            Ds1302::DateTime dt;
            dt.year = 23;
            dt.month = 6;
            dt.day = 1;
            dt.hour = hours;
            dt.minute = minutes;
            dt.second = seconds;
            if (rtc.isHalted()) DBG("RTC is halted...");
            rtc.setDateTime(&dt);
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));

            sendSensorsValues();              //now answer with the sensors values
            break;
          default:
            //do nothing
            break;
        }
      }
    }
    myCtrl = 0;                                       //Ctrl message has been read, clear it
  }


  if (((millis() - timeOut) > (sendStatetimeOut - 1000)) && (hasRtcTime)) gotoSleep(); //if no RTC time then wait for Time sync message
}


void sendSensorsValues(void)
{
  timeOut = millis();
  //loadFDRS(float data, uint8_t type, uint16_t id);
  uint16_t id;
  id = (UNIT_MAC << 8) | 0;
  loadFDRS(M1, SOIL_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x200 = GTW2 sensor0
  //id = (UNIT_MAC << 8) | 1;
  //    loadFDRS(M3, SOIL_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x201 = GTW2 sensor1
  //id = (UNIT_MAC << 8) | 2;
  //    loadFDRS(M4, SOIL_T, id); //will send back these data readings using INTERNAL_ACT event. Id 0x202 = GTW2 sensor2
  sendFDRS();
  forceSleep ++;
  if (forceSleep > 3) gotoSleep();  //escape from eternal lock...
}

void gotoSleep()
{
  Serial.println("Entering DeepSleep");
  if (!hasReceivedCmd)
  {
    Serial.println("No Cmd received... will shutdown the irrigation");
    pinMode(ENA_9V_PIN, OUTPUT);  //enable stepUp booster 9V
    digitalWrite(ENA_9V_PIN, HIGH);
    delay(100);
    for (int i = 0; i < 4; i++)
    {
      activateCoil(i, false);
    }
    pinMode(ENA_9V_PIN, INPUT);  //disable stepUp booster 9V
  }
  int mm = (int)(floor((minute() * 60 + second()) / (60 * timeToSleep)) * timeToSleep + timeToSleep);
  Serial.print ("next integer minutes " );
  Serial.println (mm);
  long tt;                                              //time to sleep
  tt = mm * 60 - (minute() * 60 + second());
  display_time();
  esp_sleep_enable_timer_wakeup(tt * uS_TO_S_FACTOR);
  esp_deep_sleep_start();                               //enter deep sleep mode
  delay(1000);
  abort();
}
