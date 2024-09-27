//=====================================================================================
// TITLE:         WeatherSense9001-CrowPanel7.ino
// DATE:          07/27/2024
// AUTHOR:        All Things Bobot
//                https://www.youtube.com/@AllThingsBobot
//                bobot@allthingsbobot.com
// VERSION:       1.0
//
// SUITABLE FOR:  Elecrow CrowPanel 7.0" ESP32 HMI Display (v3)
//
// DESCRIPTION:   Displays weather conditions on CrowPanel 7.0"
//                for sensor data read from SensorPush API
//                for multiple sensors
//                Respects API sensor names and min/max values
//                and sets alert colors for labels on display
//                if values exceed thresholds
//
// UPDATED:       07.27.24  Bobot   Initial release
//                07.29.24  Bobot   Add data push to ThingSpeak API
//                                  Fix length of char array for
//                                  "upd" date time in readings structure
//                07.30.24  Bobot   Added secrets.h with credentials
//                07.31.24  Bobot   Fix crash when error code returned from ThingSpeak
//                                  Adjust pressure chart axis, labels and tick marks 
//                                  to better show value range
//                08.05.24	Bobot   Rework Pressure Chart graphics and update timing
//                09.11.24	Bobot   Add fix for reconnect on wifi connection loss
//                              
//=====================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <PCA9557.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <time.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone

#include "secrets.h"

// LVGL and UI
#include <lvgl.h>
#include "ui.h"

// Config the display panel and touch panel in gfx_conf.h
#include "gfx_conf.h"
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf1[screenWidth * screenHeight / 8];
static lv_color_t disp_draw_buf2[screenWidth * screenHeight / 8];
static lv_disp_drv_t disp_drv;

// WiFi Credentials / IP Setup 
const char* ssid = SECRET_WIFISSID;
const char* password = SECRET_WIFIPASS;
IPAddress ip(192, 168, 20, 21);
IPAddress gateway(192, 168, 20, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 20, 1); //primaryDNS

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
Timezone usET(usEDT, usEST);

// Define enum for level of serial monitor logs
enum loglevel {MIN, MAX};
loglevel llLogLevel = MIN;

// Define enum for type of API call
enum calltype {AUTH, ACCESS, SAMPLE, SENSOR, THINGSPEAK};

// Define structure to hold readings returned for a sensor
struct readings{
  char desc[10];
	float temp;
  float tempalert_min;
  float tempalert_max;
	float hum;
  float humalert_min;
  float humalert_max;
  float pres;
	char upd[30];
	float bat;
	float wifi;
};

// Constants for sensors
const char *cchrSensor1 = SECRET_SP_SENSOR1_ID;
const char *cchrSensor2 = SECRET_SP_SENSOR2_ID;
const char *cchrSensor3 = SECRET_SP_SENSOR3_ID;
const float cfltLowBatteryThreshold = 2.3;
const float cfltLowWifiThreshold = -85.0;

const int auth_minutes = 23;      //refresh interval (in minutes) for SensorPush API authentication refresh call
const int refresh_minutes = 10;   //refresh interval (in minutes) for SensorPush API calls to retrieve sensor readings
const int retry_minutes = 1;      //retry interval (in minutes) for bad SensorPush API read
const int pressupd_minutes = 30;  //refresh interval (in minutes) for pressure chart update

const long authinterval = auth_minutes * 60000;           //refresh interval for API authenthication refresh (minutes x 1000 x 60)
const long refreshinterval = refresh_minutes * 60000;     //refresh interval for API sensor reading calls (minutes x 1000 x 60)
const long retryinterval = retry_minutes * 60000;         //retry interval on bad API read
const long pressupdinterval = pressupd_minutes * 60000;   //refresh interval for pressure chart (minutes x 1000 x 60)


//Constants for ThingSpeak
const unsigned long glngThingSpeakChannel = SECRET_TS_CHANNEL_ID;
const char * gchrThingSpeakWriteAPIKey = SECRET_TS_WRITE_APIKEY;

// Global Project variables
WiFiClientSecure *wcsClient;   //wifi "secure" client for encrypted https without certificate
int gintResponseCode = 0;
bool gblnGoodRead = false;
bool gblnFirstRun = true;
bool gblnFirstPressChartUpd = true;
char authorization[1000];   //authorization code returned from auth API call with login creds
char accesstoken[2350];     //access token returned from oauth API call to be used for subsequent API calls for sensor data
char errormessage[100];    //error message returned in JSON if HTTP/API call fails
unsigned long previousMillis = 0;
unsigned long previousMillis_Wifi = 0;
unsigned long interval_Wifi = 30000;
lv_timer_t * timerauth;     //timer for authorization calls
lv_timer_t * timersamples;  //timer for sensor sample calls
lv_timer_t * timerpressupd;  //timer for pressure chart update
readings rdgReadings[3];  //sensor readings array for multiple sensors
bool gblnRetValThingSpeak;   //return value from ThingSpeak API call

PCA9557 Out;    //for touch timing init


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
   uint32_t w = ( area->x2 - area->x1 + 1 );
   uint32_t h = ( area->y2 - area->y1 + 1 );

   tft.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);

   lv_disp_flush_ready( disp );
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
   uint16_t touchX, touchY;
   bool touched = tft.getTouch( &touchX, &touchY);
   if( !touched )
   {
      data->state = LV_INDEV_STATE_REL;
   }
   else
   {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

      Serial.print( "Data x " );
      Serial.println( touchX );

      Serial.print( "Data y " );
      Serial.println( touchY );
   }
}


// Function to call and API via http POST with passed arguments for URL, headers, body and call type
void API_Post(char *pchrURL, char *pchrHeader_ContentType, char *pchrHeader_Accept, char *pchrHeader_Value1, char *pchrHeader_Authorization, calltype penuCallType)
{

  if (WiFi.status() != WL_CONNECTED) 
  {
    BadRead("Wifi error");
    unsigned long currentMillis_Wifi = millis();
    // if WiFi is down, try reconnecting every interval_Wifi milliseconds
    if (currentMillis_Wifi - previousMillis_Wifi >= interval_Wifi) 
    {
      Serial.print(millis());
      Serial.println(" -> Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis_Wifi = currentMillis_Wifi;
    }    
    return;
  }

  if (llLogLevel == MAX) {
    Serial.println(pchrURL);
    Serial.println(pchrHeader_ContentType);
    Serial.println(pchrHeader_Accept);
    Serial.println(pchrHeader_Value1);
    Serial.println(pchrHeader_Authorization);
  }

  if(wcsClient) {

      //create an HTTPClient instance
      HTTPClient https;
      https.setReuse(false);
      https.useHTTP10(true);
      https.begin(*wcsClient, pchrURL);

      https.addHeader("Content-Type", pchrHeader_ContentType);

      if (strlen(pchrHeader_Accept) > 1) 
      {
        https.addHeader("accept", pchrHeader_Accept);
      }

      if (strlen(pchrHeader_Authorization) > 1) 
      {
        https.addHeader("Authorization", pchrHeader_Authorization);
      }

      // Make the API call, get response
      gintResponseCode = 0;
      gintResponseCode = https.POST(pchrHeader_Value1);
      if (llLogLevel == MAX) {
        Serial.print("HTTP Response code for ");
        Serial.print(pchrURL);
        Serial.print(": ");
      }
      Serial.println(gintResponseCode);

      //deserialize JSON call result
      JsonDocument jdDocTemp;
      DeserializationError error = deserializeJson(jdDocTemp, https.getStream());
      if (llLogLevel == MAX) {
        Serial.print("deserializeJson() returned ");
        Serial.println(error.c_str());
      }

      // Evaluate result for error
      // or store good result data to global vars
      if (gintResponseCode == 400)
      {
        strcpy(errormessage, jdDocTemp["error"]["message"]);
        Serial.println(errormessage);
      }
      else
      {
      switch (penuCallType) {

        case calltype::AUTH:
          strcpy(authorization, jdDocTemp["authorization"]);
          break;
        
        case calltype::ACCESS:
          strcpy(accesstoken, jdDocTemp["accesstoken"]);
          break;
        
        case calltype::SAMPLE:
          //Sensor 1 - Temp, Hum, TimeStamp
          rdgReadings[0].temp = jdDocTemp["sensors"][cchrSensor1][0]["temperature"];
          rdgReadings[0].hum = jdDocTemp["sensors"][cchrSensor1][0]["humidity"];
          rdgReadings[0].pres = jdDocTemp["sensors"][cchrSensor1][0]["barometric_pressure"];
          strcpy(rdgReadings[0].upd, jdDocTemp["sensors"][cchrSensor1][0]["observed"]);

          //Sensor 2 - Temp, Hum, TimeStamp
          rdgReadings[1].temp = jdDocTemp["sensors"][cchrSensor2][0]["temperature"];
          rdgReadings[1].hum = jdDocTemp["sensors"][cchrSensor2][0]["humidity"];
          strcpy(rdgReadings[1].upd, jdDocTemp["sensors"][cchrSensor2][0]["observed"]);

          //Sensor 3 - Temp, Hum, TimeStamp
          rdgReadings[2].temp = jdDocTemp["sensors"][cchrSensor3][0]["temperature"];
          rdgReadings[2].hum = jdDocTemp["sensors"][cchrSensor3][0]["humidity"];
          strcpy(rdgReadings[2].upd, jdDocTemp["sensors"][cchrSensor3][0]["observed"]);
          break;
        
        case calltype::SENSOR:
          //Sensor 1 - Name, Batt, Wifi, Temp alert min/max, Hum alert min/max
          strcpy(rdgReadings[0].desc, jdDocTemp[cchrSensor1]["name"]);
          rdgReadings[0].bat = jdDocTemp[cchrSensor1]["battery_voltage"];
          rdgReadings[0].wifi = jdDocTemp[cchrSensor1]["rssi"];
          rdgReadings[0].tempalert_min = jdDocTemp[cchrSensor1]["alerts"]["temperature"]["min"];
          rdgReadings[0].tempalert_max = jdDocTemp[cchrSensor1]["alerts"]["temperature"]["max"];
          rdgReadings[0].humalert_min = jdDocTemp[cchrSensor1]["alerts"]["humidity"]["min"];
          rdgReadings[0].humalert_max = jdDocTemp[cchrSensor1]["alerts"]["humidity"]["max"];

          //Sensor 2 - Name, Batt, Wifi, Temp alert min/max, Hum alert min/max
          strcpy(rdgReadings[1].desc, jdDocTemp[cchrSensor2]["name"]);
          rdgReadings[1].bat = jdDocTemp[cchrSensor2]["battery_voltage"];
          rdgReadings[1].wifi = jdDocTemp[cchrSensor2]["rssi"];
          rdgReadings[1].tempalert_min = jdDocTemp[cchrSensor2]["alerts"]["temperature"]["min"];
          rdgReadings[1].tempalert_max = jdDocTemp[cchrSensor2]["alerts"]["temperature"]["max"];
          rdgReadings[1].humalert_min = jdDocTemp[cchrSensor2]["alerts"]["humidity"]["min"];
          rdgReadings[1].humalert_max = jdDocTemp[cchrSensor2]["alerts"]["humidity"]["max"];        

          //Sensor 3 - Name, Batt, Wifi, Temp alert min/max, Hum alert min/max
          strcpy(rdgReadings[2].desc, jdDocTemp[cchrSensor3]["name"]);
          rdgReadings[2].bat = jdDocTemp[cchrSensor3]["battery_voltage"];
          rdgReadings[2].wifi = jdDocTemp[cchrSensor3]["rssi"];
          rdgReadings[2].tempalert_min = jdDocTemp[cchrSensor3]["alerts"]["temperature"]["min"];
          rdgReadings[2].tempalert_max = jdDocTemp[cchrSensor3]["alerts"]["temperature"]["max"];
          rdgReadings[2].humalert_min = jdDocTemp[cchrSensor3]["alerts"]["humidity"]["min"];
          rdgReadings[2].humalert_max = jdDocTemp[cchrSensor3]["alerts"]["humidity"]["max"];              
          break;

        case calltype::THINGSPEAK:
          gblnRetValThingSpeak = jdDocTemp["success"];
          break;

        default:
          // if nothing else matches, do the default
          // default is optional
          break;
        }
      }

      https.end();

    }
    else
    {
      Serial.printf("[HTTPS] Secure client unable to connect\n");
    }
}


// Function to convert incoming UTC time stamp to local time zone and return as formatted string for project display
void UTCtoLocal(const char *pchrObservedIn, char *pchrFormattedObserved)
{
  struct tm tmObserved = {0};

  // Convert to tm struct
  strptime(pchrObservedIn, "%Y-%m-%dT%H:%M:%S.000Z", &tmObserved);
  
  // Convert struct tm to time_t
  time_t time_tObserved = mktime(&tmObserved);

  TimeChangeRule *tcr; 
  time_t time_tZoned = usET.toLocal(time_tObserved,&tcr);

  sprintf(pchrFormattedObserved, "%.2d:%.2d %.2d/%.2d", hour(time_tZoned), minute(time_tZoned), month(time_tZoned), day(time_tZoned));

}


// SensorPush Authorization
// Log in using a valid email/password to receive an authorization code
// Use authorization code to retrieve access token that will be used for subsequent API calls for sensor data
void SP_Authentication()
{
  gblnGoodRead = true;

  char chrURL[100] = "https://api.sensorpush.com/api/v1/oauth/authorize";
  char chrHeader_ContentType[25] = "application/json";
  char chrHeader_Accept[25] = "application/json";
  char chrHeader_Authorization[1000] = "";
  char chrHeader_Value1[100] = "";
  sprintf(chrHeader_Value1, "{\"email\":\"%s\",\"password\":\"%s\"}",SECRET_SP_LOGINID,SECRET_SP_PASS);

  API_Post(chrURL,chrHeader_ContentType,chrHeader_Accept,chrHeader_Value1,chrHeader_Authorization,calltype::AUTH);

  if (gblnGoodRead)
    {
      if (gintResponseCode != 200) 
        {
          BadRead("BAD authentication");
          return;
        }

      if (llLogLevel == MAX) {Serial.println("Good Authentication!");}

      delay(2000);
      // Request a temporary oauth access token
      // Use the result from the previous step for the authorization code in the body
      strcpy(chrURL, "https://api.sensorpush.com/api/v1/oauth/accesstoken");
      strcpy(chrHeader_Accept, "application/json");
      strcpy(chrHeader_ContentType, "application/json");
      strcpy(chrHeader_Authorization, "");

      JsonDocument doc_access;
      doc_access["authorization"] = authorization;

      //Serialize JSON document
      String JSON_access;
      serializeJson(doc_access, JSON_access);
      strcpy(chrHeader_Value1,JSON_access.c_str());

      API_Post(chrURL,chrHeader_ContentType,chrHeader_Accept,chrHeader_Value1,chrHeader_Authorization,calltype::ACCESS);

      if (gblnGoodRead)
        {
          if (gintResponseCode != 200) 
            {
              BadRead("BAD OAUTH");
              return;
            }

          if (llLogLevel == MAX) {Serial.println("Good OAUTH!");}

        }
    }
}


//Function to report error message to serial monitor and display when a bad API read is encountered
void BadRead(char *pchrErrorDesignator)
{
  gblnGoodRead = false;
  Serial.println(pchrErrorDesignator);
  SetAlertColors(ui_Status1);
  lv_label_set_text(ui_Status1, pchrErrorDesignator);
  if (gintResponseCode == 400)
    {
      Serial.println(errormessage);
      char chrFinalErrorMsg[100];
      sprintf(chrFinalErrorMsg, "%s - %s", pchrErrorDesignator, errormessage);  
      lv_label_set_text(ui_Status1, chrFinalErrorMsg);
    }
  delay(10000);
  Serial.println("Retrying in approx 1 min...");
  lv_label_set_text(ui_Status1,"Attempting to reconnect to wifi...");
  
  previousMillis = millis();
}


void SetAlertColors(lv_obj_t *puiControlName)
{
  lv_obj_set_style_text_color(puiControlName,lv_color_hex(0xFFFF00),LV_PART_MAIN);
  lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0xFF0000), LV_PART_MAIN);
}


void SetNormalColors(lv_obj_t *puiControlName, char pchrControlType)
{

  switch (pchrControlType) {

    case 't':   //temperature
      lv_obj_set_style_text_color(puiControlName,lv_color_hex(0xBDBEBD),LV_PART_MAIN);
      lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0x540592), LV_PART_MAIN);
        break;
      
    case 'h':   //humidity
      lv_obj_set_style_text_color(puiControlName,lv_color_hex(0xBDBEBD),LV_PART_MAIN);
      lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0x1F3FC2), LV_PART_MAIN);
        break;

    case 'b':   //battery
      lv_obj_set_style_text_color(puiControlName,lv_color_hex(0xBDBEBD),LV_PART_MAIN);
      lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0xC52A92), LV_PART_MAIN);
      break;

    case 'w':   //wifi
      lv_obj_set_style_text_color(puiControlName,lv_color_hex(0xBDBEBD),LV_PART_MAIN);
      lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0x2B1D1D), LV_PART_MAIN);
      break;

    case 's': case 'm':   //status(s) or ThingSpeak status(m)
      lv_obj_set_style_text_color(puiControlName,lv_color_hex(0x25302A),LV_PART_MAIN);
      lv_obj_set_style_bg_color(puiControlName,lv_color_hex(0x0A8421), LV_PART_MAIN);
      break;

    default:
        // if nothing else matches, do the default
        // default is optional
        break;

  }
}


// Update display with passed arguments for a FLOAT field 
void UpdateDisplay_Float(float pfltValue, lv_obj_t *puiControlName, int pintSensorNumber, int pintInitialLength, int pintMinimumLength, int pintDecimalPlaces, char *pchrFormat)
{
  char chrValue[pintInitialLength];
  char chrValueFinal[pintInitialLength+strlen(pchrFormat)-2];   //dynamically set final display string length to be value plus NULL terminator minus %s designator
  dtostrf(pfltValue, pintMinimumLength, pintDecimalPlaces, chrValue);   //convert float to string, min len, decimal places
  sprintf(chrValueFinal, pchrFormat, chrValue);  
  Serial.println(chrValueFinal);
  lv_label_set_text(puiControlName, chrValueFinal);

  //Decrement sensor number to account for 0-based array
  pintSensorNumber--;

  //Set text/label alert colors if temperature above or below set min and max alert thresholds
  if (strcmp(pchrFormat, "%s°") == 0) 
    {
      if ((pfltValue < rdgReadings[pintSensorNumber].tempalert_min) || (pfltValue > rdgReadings[pintSensorNumber].tempalert_max)) {SetAlertColors(puiControlName);} else {SetNormalColors(puiControlName,'t');}
    }

  //Set text/label alert colors if humidity above or below set min and max alert thresholds
  if (strcmp(pchrFormat, "%s%%") == 0) 
    {
      if ((pfltValue < rdgReadings[pintSensorNumber].humalert_min) || (pfltValue > rdgReadings[pintSensorNumber].humalert_max)) {SetAlertColors(puiControlName);} else {SetNormalColors(puiControlName,'h');}
    }

  //Set text/label alert colors if battery level below threshold
  if (strcmp(pchrFormat, "%sv") == 0) 
    {
      if (pfltValue <= cfltLowBatteryThreshold) {SetAlertColors(puiControlName);} else {SetNormalColors(puiControlName,'b');}
    }

  //Set text/label alert colors if wifi signal level below threshold
  if (strcmp(pchrFormat, "%sdB") == 0) 
    {
      if (pfltValue <= cfltLowWifiThreshold) {SetAlertColors(puiControlName);} else {SetNormalColors(puiControlName,'w');}
    }
}


// Update display with passed arguments for a time stamp field 
void UpdateDisplay_TimeStamp(const char *pchrTimeStamp, lv_obj_t *puiControlName)
{
  const char *chrValue = pchrTimeStamp;
  char chrValueFinal[13];
  UTCtoLocal(chrValue,chrValueFinal);
  Serial.println(chrValueFinal);
  lv_label_set_text(puiControlName, chrValueFinal);
}


// Function to authenticate API that is called every timer increment 
void my_timer_auth(lv_timer_t * timerauth)
{
  if (llLogLevel == MAX) {
    Serial.println("---------");
    Serial.println(" NATURAL ");
    Serial.println("---------");
  }
  // SensorPush Authorization
  SP_Authentication();  
}


// Function (the meat of the project) that is called every timer increment 
// to make numerous sequential calls to API to get sensor data and update display
void my_timer_samples(lv_timer_t * timersamples)
{
  if (!gblnGoodRead)
    {
      if (llLogLevel == MAX) {
        Serial.println("**************");
        Serial.println(" FORCING CALL ");
        Serial.println("**************");
      }
      // if bad access token or other bad API read, force call to refresh authentication 
      SP_Authentication();
    }

    delay(2000);
    // Query Sensors
    // Use the access token (global variable) in the header
    if (gblnGoodRead)
    {
      char chrURL[100] = "https://api.sensorpush.com/api/v1/devices/sensors";
      char chrHeader_ContentType[25] = "text/plain";
      char chrHeader_Accept[25] = "application/json";
      char chrHeader_Value1[100] = "{}";
      char chrHeader_Authorization[2350] = "";

      strcpy(chrHeader_Authorization, accesstoken);
      
      API_Post(chrURL,chrHeader_ContentType,chrHeader_Accept,chrHeader_Value1,chrHeader_Authorization,calltype::SENSOR);

      if (gblnGoodRead)
        {
          if (gintResponseCode != 200)
            {
              BadRead("BAD sensors read");
              return;
            }

          if (llLogLevel == MAX) {Serial.println("Good Sensors read");}

          // Update display field for Sensor Descriptions
          lv_label_set_text(ui_Desc1, rdgReadings[0].desc);
          lv_label_set_text(ui_Desc2, rdgReadings[1].desc);          
          lv_label_set_text(ui_Desc3, rdgReadings[2].desc);

          // Update display fields for battery voltage and wifi signal
          UpdateDisplay_Float(rdgReadings[0].bat, ui_Batt1, 1, 4, 4, 2, "%sv");     // Batt1
          UpdateDisplay_Float(rdgReadings[0].wifi, ui_Wifi1, 1, 7, 4, 1, "%sdB");   // Wifi1

          UpdateDisplay_Float(rdgReadings[1].bat, ui_Batt2, 2, 4, 4, 2, "%sv");     // Batt2
          UpdateDisplay_Float(rdgReadings[1].wifi, ui_Wifi2, 2, 7, 4, 1, "%sdB");   // Wifi2

          UpdateDisplay_Float(rdgReadings[2].bat, ui_Batt3, 3, 4, 4, 2, "%sv");     // Batt3
          UpdateDisplay_Float(rdgReadings[2].wifi, ui_Wifi3, 3, 7, 4, 1, "%sdB");   // Wifi3

          
          delay(2000);
          // Query Samples
          // Use the access token (global var) in the header
          // Set sample limit in body
          strcpy(chrURL, "https://api.sensorpush.com/api/v1/samples");
          strcpy(chrHeader_ContentType, "text/plain");
          strcpy(chrHeader_Accept, "application/json");
          strcpy(chrHeader_Value1, "");
          strcpy(chrHeader_Authorization, accesstoken);

          // set up body data for call to limit number of samples to return for each sensor
          JsonDocument doc_samples;
          const int samplelimit = 1;
          doc_samples["limit"] = samplelimit;

          //Serialize JSON document
          String JSON_samples;
          serializeJson(doc_samples, JSON_samples);
            
          strcpy(chrHeader_Value1,JSON_samples.c_str());

          API_Post(chrURL,chrHeader_ContentType,chrHeader_Accept,chrHeader_Value1,chrHeader_Authorization,calltype::SAMPLE);

          if (gblnGoodRead)
          {
            if (gintResponseCode != 200) 
              {
                BadRead("BAD samples read");
                return;
              }

            if (llLogLevel == MAX) {Serial.println("Good Samples read");}

            // Update display fields for temp and hum
            UpdateDisplay_Float(rdgReadings[0].temp, ui_Temp1, 1, 6, 3, 1, "%s°");   // Temp1
            UpdateDisplay_Float(rdgReadings[0].hum, ui_Hum1, 1, 4, 1, 0, "%s%%");    // Hum1

            UpdateDisplay_Float(rdgReadings[1].temp, ui_Temp2, 2, 6, 3, 1, "%s°");   // Temp2
            UpdateDisplay_Float(rdgReadings[1].hum, ui_Hum2, 2, 4, 1, 0, "%s%%");    // Hum2

            UpdateDisplay_Float(rdgReadings[2].temp, ui_Temp3, 3, 6, 3, 1, "%s°");   // Temp3
            UpdateDisplay_Float(rdgReadings[2].hum, ui_Hum3, 3, 4, 1, 0, "%s%%");    // Hum3

            UpdateDisplay_Float(rdgReadings[0].pres, ui_Pres1, 1, 10, 10, 2, "%s inHg");   // Pres1

            // Update display field for readings time and date 
            UpdateDisplay_TimeStamp(rdgReadings[0].upd, ui_Upd1);
            UpdateDisplay_TimeStamp(rdgReadings[1].upd, ui_Upd2);
            UpdateDisplay_TimeStamp(rdgReadings[2].upd, ui_Upd3);        


            delay(2000);
            // Set up to write values to ThingSpeak IoT data aggregation web site
            // Uses global constants defined for glngThingSpeakChannel and gchrThingSpeakWriteAPIKey
            sprintf(chrURL,"https://api.thingspeak.com/channels/%ld/bulk_update.json",glngThingSpeakChannel);
            strcpy(chrHeader_ContentType, "application/json");
            strcpy(chrHeader_Accept, "");
            strcpy(chrHeader_Authorization, "");
            strcpy(chrHeader_Value1, "");   //clear previously used body variable
            char chrHeader_Value2[500] = "";   //use new variable for longer body text for this call

            // set up JSON body data for sensor data values to be sent to ThingSpeak
            JsonDocument doc_thingspeak;

            doc_thingspeak["write_api_key"] = gchrThingSpeakWriteAPIKey;

            //Sensor 1 to ThingSpeak
            doc_thingspeak["updates"][0]["created_at"] = rdgReadings[0].upd;
            doc_thingspeak["updates"][0]["field1"] = rdgReadings[0].temp;
            doc_thingspeak["updates"][0]["field2"] = rdgReadings[0].hum;
            doc_thingspeak["updates"][0]["field3"] = rdgReadings[0].pres;
            doc_thingspeak["updates"][0]["field4"] = rdgReadings[0].bat;

            //Sensor 2 to ThingSpeak
            doc_thingspeak["updates"][1]["created_at"] = rdgReadings[1].upd;
            doc_thingspeak["updates"][1]["field5"] = rdgReadings[1].temp;
            doc_thingspeak["updates"][1]["field6"] = rdgReadings[1].hum;

            //Sensor 3 to ThingSpeak
            doc_thingspeak["updates"][2]["created_at"] = rdgReadings[2].upd;
            doc_thingspeak["updates"][2]["field7"] = rdgReadings[2].temp;
            doc_thingspeak["updates"][2]["field8"] = rdgReadings[2].hum;

            //Serialize JSON document
            String JSON_thingspeak;
            serializeJson(doc_thingspeak, JSON_thingspeak);
              
            strcpy(chrHeader_Value2,JSON_thingspeak.c_str());
            
            gblnRetValThingSpeak = false;
            API_Post(chrURL,chrHeader_ContentType,chrHeader_Accept,chrHeader_Value2,chrHeader_Authorization,calltype::THINGSPEAK);

            if (gblnRetValThingSpeak)
              {
                if (llLogLevel == MAX) {Serial.println("ThingSpeak channel update successful.");}
                SetNormalColors(ui_TSStatus,'m');
              }
            else
              {
                SetAlertColors(ui_TSStatus);
                if (llLogLevel == MAX) 
                  {
                    Serial.println("Problem updating ThingSpeak channel.");
                    if (gintResponseCode == 400)
                      {
                        Serial.println(errormessage);
                      }
                  }
              }


            //Show good read status on display status field   
            if (llLogLevel == MAX) {Serial.println("GOOD - Full Sensor and Sample Read");}
            SetNormalColors(ui_Status1,'s');
            lv_label_set_text(ui_Status1, "Good Read");
			
            if (gblnFirstPressChartUpd)
				{
				  // forces pressure chart update time to run once immediately after initial read of sensors
				  lv_timer_ready(timerpressupd);
				  gblnFirstPressChartUpd = false;
				}
          
            Serial.printf("Stack:%d,Heap:%lu\n", uxTaskGetStackHighWaterMark(NULL), (unsigned long)ESP.getFreeHeap());
            Serial.println("here End.");

        }
      }
  }

}


// Function that is called every timer increment (pressupdinterval)
// to update the barometric pressure chart
void my_timer_pressupd(lv_timer_t * timerpressupd)
{
  if (llLogLevel == MAX) 
	  {
		Serial.println("Updating Pressure Chart");
	  }
  lv_chart_set_next_value(ui_PressureChart, ui_PressureChart_series_1, rdgReadings[0].pres*100);
  lv_chart_refresh(ui_PressureChart);
}


// This sets Arduino Stack Size - comment this line to use default 8K stack size
SET_LOOP_TASK_STACK_SIZE(11*1024); // 11K

void setup()
{
  Serial.begin(9600);
  delay(1000);

  if (llLogLevel == MAX) {Serial.println("WeatheratorCrow7");} else {Serial.println("WC7");}

  //GPIO init
  #if defined (CrowPanel_50) || defined (CrowPanel_70)
    pinMode(38, OUTPUT);
    digitalWrite(38, LOW);
    pinMode(17, OUTPUT);
    digitalWrite(17, LOW);
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);
    pinMode(42, OUTPUT);
    digitalWrite(42, LOW);
	
    //touch timing init
    Wire.begin(19, 20);
    Out.reset();
    Out.setMode(IO_OUTPUT);
    Out.setState(IO0, IO_LOW);
    Out.setState(IO1, IO_LOW);
    delay(20);
    Out.setState(IO0, IO_HIGH);
    delay(100);
    Out.setMode(IO1, IO_INPUT);
  #elif defined (CrowPanel_43)
    pinMode(20, OUTPUT);
    digitalWrite(20, LOW);
    pinMode(19, OUTPUT);
    digitalWrite(19, LOW);
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(38, OUTPUT);
    digitalWrite(38, LOW);
    pinMode(0, OUTPUT);//TOUCH-CS
  #endif

  //Display Prepare
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  delay(200);

  lv_init();

  delay(100);

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, screenWidth * screenHeight/8);
  
  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.full_refresh = 1;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  tft.fillScreen(TFT_BLACK);

  // Squareline Studio UI initialization
  ui_init();


  // Connect to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, dns, gateway, subnet); 
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (llLogLevel == MAX) {Serial.print(".");}
  }

  if (llLogLevel == MAX) {
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Set up encrypted http (for SensorPush API calls)
  wcsClient = new WiFiClientSecure;
        
  // set secure client without certificate (for SensorPush API calls)
  wcsClient->setInsecure();
  
  // Set up timer to call my_timer_auth method every timer increment
  //Refresh of API access token required only every 60 minutes
  timerauth = lv_timer_create(my_timer_auth, authinterval , NULL);

  // Set up timer to call my_timer_samples method every timer increment
  timersamples = lv_timer_create(my_timer_samples, refreshinterval,  NULL);
  
  // Set up timer to call my_timer_pressupd method every timer increment
  timerpressupd = lv_timer_create(my_timer_pressupd, pressupdinterval,  NULL);
  

   // makes a timer run on the next call of lv_timer_handler() - forces an initial call on start
  lv_timer_ready(timersamples);

  if (llLogLevel == MAX) {Serial.println("Setup done");}

}


void loop()
{
    if (gblnFirstRun){
      Serial.printf("Stack:%d,Heap:%lu\n", uxTaskGetStackHighWaterMark(NULL), (unsigned long)ESP.getFreeHeap());
      gblnFirstRun = false;
    }
    
    unsigned long currentMillis = millis();
    if ((!gblnGoodRead) && (currentMillis - previousMillis >= retryinterval)) 
    {
      previousMillis = currentMillis;
      lv_timer_ready(timersamples);
    }
    
    lv_timer_handler();
    delay(5);  //required for display refresh
}
