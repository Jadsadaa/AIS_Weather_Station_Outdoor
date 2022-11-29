#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>

#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <ModbusMaster.h>
#include "DOSensor.h"
#include <HardwareSerial.h>
#include "NB_BC95_G.h"
#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

#define SERIAL1_RXPIN 14
#define SERIAL1_TXPIN 27

HardwareSerial modbus(2);
HardwareSerial myserial(1);

NB_BC95_G AISnb;
BluetoothSerial SerialBT;


#define battPIN  34
#define donePIN  25
String Ver = "0.1" ;
String HOSTNAME = "";
String deviceToken = "";
String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;
String json = "";


const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password


ModbusMaster node;

#define trigWDTPin    32
#define ledHeartPIN   0


struct Meter
{
  String SO2;
  String NO2;
  String CO;
  String O3;
  String PM2_5;
  String PM10;
  String temp;
  String hum;

  float bat;
  String rssi;

};
Meter meter;

uint16_t dataWeather[8];

unsigned long currentMillis;
unsigned long previousMillis;
int interval = 60; // Interval Time
int intervalSleep = 30; // Interval Time
//unsigned int previous_check = 0;
boolean waitSleep = 0;
unsigned long previousMillisSleep;

float Batt = 0.0;
int countSend = 0;

void setupWIFI()
{
  Serial.println("Connecting...");
  Serial.println(String(ssid));
  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}



void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  //No authentication by default
  ArduinoOTA.setPassword(password);
  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);
    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));

  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.begin();
}



void setup()
{

  Serial.begin(115200);

  modbus.begin(9600, SERIAL_8N1, 16, 17);

  SerialBT.begin("AIS_Weather_Station"); //Bluetooth device name
  SerialBT.println("AIS_Weather_Station");
  AISnb.debug = true;
  AISnb.delayAfterCommand = 1000;
  AISnb.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);

  // AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  nb_resp_t res_DeviceIP = AISnb.getDeviceIP();
  nb_resp_t res_testPing = AISnb.testPing(serverIP);
  nb_resp_t res_nccid = AISnb.getNCCID();
  deviceToken = res_nccid.data;
  while (deviceToken == 0) {
    nb_resp_t res_nccid = AISnb.getNCCID();
    deviceToken = res_nccid.data;
  }
  HOSTNAME = deviceToken;
  Serial.print("nccid:");
  Serial.println(deviceToken);
  SerialBT.print("NCCID:");
  SerialBT.println(deviceToken);
  SerialBT.println("VER:" + Ver);

  Serial.println();
  Serial.println(F("***********************************"));

  Serial.println("Initialize...");
  setupWIFI();
  setupOTA();

}

void sendViaNBIOT()
{
  nb_signal_t res_signal = AISnb.getSignal();
  meter.rssi = res_signal.rssi;

  while (meter.rssi == 0) {
    nb_signal_t res_signal = AISnb.getSignal();
    meter.rssi = res_signal.rssi;
  }
  Serial.print("RSSI:"); Serial.println(meter.rssi);

  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);

  json.concat("\",\"so2\":");
  json.concat(meter.SO2);
  json.concat(",\"no2\":");
  json.concat(meter.NO2);
  json.concat(",\"co\":");
  json.concat(meter.CO);
  json.concat(",\"o3\":");
  json.concat(meter.O3);
  json.concat(",\"pm2_5\":");
  json.concat(meter.PM2_5);
  json.concat(",\"pm10\":");
  json.concat(meter.PM10);
  json.concat(",\"temp\":");
  json.concat(meter.temp);

  json.concat(",\"hum\":");
  json.concat(meter.hum);
  json.concat(",\"bat\":");
  json.concat(meter.bat);

  json.concat(",\"rssi\":");
  json.concat(meter.rssi);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  //
  /* nb_resp_t res_send = AISnb.sendUDPMessage(1, serverIP, serverPort, json.length(), json, MODE_STRING_HEX);

    if (!res_send.status)
    {
     AISnb.createUDPSocket(serverPort);
    }
    String getResponse = AISnb.getSocketResponse();
    if (getResponse.length() > 0) {
     Serial.print("UDP response: ");
     Serial.println(getResponse);
    }*/

  SerialBT.print("rssi:");
  SerialBT.println(meter.rssi);

}



float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3450.0;
  int bitRes = 4096;
  int16_t adc = 0;
  Serial.println("Read_Batt()");
  for (int a = 0; a < 20; a++)
  {
    adc  += analogRead(battPIN);
    Serial.print(adc);
    delay(1);
  }

  vRAW = adc / 20;
  Vout = (vRAW * 3.3) / bitRes;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  Serial.println("end.Read_Batt()");
  return Vin;
}


void readMeter()
{
  readWeather(_SO2);


  Serial.print("meter.SO2:"); Serial.println( meter.SO2);
  Serial.print("meter.NO2:"); Serial.println( meter.NO2);
  Serial.print("meter.CO:"); Serial.println( meter.CO);
  Serial.print("meter.O3:"); Serial.println( meter.O3);
  Serial.print("meter.PM25:"); Serial.println( meter.PM2_5);
  Serial.print("meter.PM10:"); Serial.println( meter.PM10);
  Serial.print("meter.temp:"); Serial.println( meter.temp);
  Serial.print("meter.hum:"); Serial.println( meter.hum);


  Serial.println("");

}

void t1CallgetMeter() {     // Update read all data
  readMeter();
  meter.bat = Read_Batt();
  sendViaNBIOT();
}



void readWeather(uint16_t  REG)
{
  static uint32_t i;
  uint16_t j, result;

  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 8);
  Serial.print("result:"); Serial.print(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 8; j++)
    {
      dataWeather[j] = node.getResponseBuffer(j);
      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(dataWeather[j]);
      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(dataWeather[j]);
    }

    meter.SO2 = dataWeather[0];
    meter.NO2 = dataWeather[1];
    meter.CO = dataWeather[2];
    meter.O3 = dataWeather[3];
    meter.PM2_5 = dataWeather[4];
    meter.PM10 = dataWeather[5];
    meter.temp = dataWeather[6] / 100 - 40;

    meter.hum = dataWeather[7] / 100;

  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug

  }
}



void doneProcess()
{
  Serial.println("!!!!!! Done ready to Sleep ~10 Min (TPL5110) !!!!!!");
  pinMode(donePIN, OUTPUT);
  digitalWrite(donePIN, HIGH);
  delay(100);
}

void loop()
{
  unsigned long currentMillis = millis();
  currentMillis = millis() / 1000;


  if (currentMillis % 30000 == 0)
  {
    SerialBT.println("VER:" + Ver);
    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    setupWIFI();
    setupOTA();
  }

  if ((currentMillis - previousMillis >= interval) && (waitSleep == 0))
  {
    t1CallgetMeter();
    nb_resp_t res_send = AISnb.sendUDPMessage(1, serverIP, serverPort, json.length(), json, MODE_STRING_HEX);
    if (!res_send.status)
    {
      AISnb.createUDPSocket(serverPort);
    }
    String getResponse = AISnb.getSocketResponse();
    if (getResponse.length() > 0) {
      Serial.print("UDP response: ");
      Serial.println(getResponse);
    }


    previousMillis = millis() / 1000;
    //doneProcess();
    countSend++;

    if (countSend >= 4)
    {
      waitSleep = 1;
      previousMillisSleep = millis() / 1000;
    }
  }

  if (waitSleep == 1)
  {
    if (currentMillis - previousMillisSleep >= intervalSleep)
    {
      countSend = 0;
      waitSleep = 0;
      delay(2000);
      doneProcess();
    }
  }

  ArduinoOTA.handle();



}
