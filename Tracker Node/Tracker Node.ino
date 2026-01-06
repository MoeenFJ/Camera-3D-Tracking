#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TinyMPU6050.h>
#include <NeoPixelBus.h>
#include <Wire.h>

#include "wificreds.h"
//  OR
//  #define SSID "SSID"
//  #define PASSWORD "PASSWORD"


#define LED_PIN 3
#define MPUaddr 0x69
#define UDP_PORT 50404
#define TCP_PORT 50405
#define TCP_PACKET_SIZE 32
#define BROADCAST_IP "255.255.255.255"

typedef struct dataStruct {
  float ax,ay,az;
  float rx,ry,rz;
} dataStruct;




WiFiUDP udp;
WiFiServer tcpServer(TCP_PORT);
WiFiClient manager;
MPU6050 mpu(Wire);

NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> led(1);
uint8_t packetBuffer[WIFICLIENT_MAX_PACKET_SIZE + 1];
uint8_t mac[6];
dataStruct dataToSend;
IPAddress managerIP;

bool isPaired = false;
bool sendAccGyro = false;


void SendAccGyro() {
  mpu.Execute();
  dataToSend.ax = mpu.GetAccX();
  dataToSend.ay = mpu.GetAccY();
  dataToSend.az = mpu.GetAccZ();
  dataToSend.rx = mpu.GetAngX();
  dataToSend.ry = mpu.GetAngY();
  dataToSend.rz = mpu.GetAngZ();
  udp.beginPacket(managerIP, UDP_PORT);
  udp.write(mac,6);
  udp.print("TRACKNODE_UPD");
  udp.write((uint8_t*)&dataToSend,sizeof(dataToSend));
  udp.endPacket();
}

void SendDebugMessage(String s)
{
  udp.beginPacket(managerIP, UDP_PORT);
  udp.write(mac,6);
  udp.print("TRACKNODE_DBG");
  udp.print(s);
  udp.endPacket();
}
void SendDebugMessage(uint8_t* s, size_t size)
{
  udp.beginPacket(managerIP, UDP_PORT);
  udp.write(mac,6);
  udp.print("TRACKNODE_DBG");
  udp.write(s,size);
  udp.endPacket();
}

void setup() {
  delay(1000);
  led.Begin();
  led.Show();
  mpu.Initialize(0, 2);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  WiFi.macAddress(mac);

  while (WiFi.status() != WL_CONNECTED) {
    led.SetPixelColor(0, RgbColor(255, 0, 0)); 
    led.Show();
    delay(500);
    led.SetPixelColor(0, RgbColor(0, 0, 0)); 
    led.Show();
    delay(500);
  }

  udp.begin(UDP_PORT);
  tcpServer.begin();

  led.SetPixelColor(0, RgbColor(0, 0, 255)); 
  led.Show();
  delay(1000);

  //mpu.Calibrate();
}

void loop() {
  if(!isPaired)
  {
    manager = tcpServer.accept();
    if(manager)
    {
      int t = 0;
      while(manager.available()==0)
      { 
        delay(100);
        t++;
        if(t == 20)
          return;
      }

      if(manager.available()>0)
      {
        int n = manager.read(packetBuffer, TCP_PACKET_SIZE);
        //SendDebugMessage(packetBuffer,n);
        if(packetBuffer[0] == 0x03)
        {
          managerIP = manager.remoteIP();
          isPaired = true;
          return;
        }
      }
    }
    udp.beginPacket(BROADCAST_IP, UDP_PORT);
    udp.write(mac,6);
    udp.print("TRACKNODE_REG");
    udp.endPacket();
    delay(2000);
    return;
  }

  if(manager.connected())
  {
    while(manager.available()>0)
    {
      int n = manager.read(packetBuffer, TCP_PACKET_SIZE);

      switch (packetBuffer[0])
      {
        case 0x00:
          led.SetPixelColor(0, RgbColor(packetBuffer[1], packetBuffer[2], packetBuffer[3])); 
          led.Show();
          break;
        case 0x01:
          sendAccGyro = true;
          break;
        case 0x02:
          sendAccGyro = false;
          break;
        default:
          break;
      }
    }
  }
  else
  {
    isPaired = false;
    manager.stop();
    led.SetPixelColor(0, RgbColor(0,0,255)); 
    led.Show();
    return;
  }

  if(sendAccGyro)
  {
    SendAccGyro();
    delay(15);
  }
}
