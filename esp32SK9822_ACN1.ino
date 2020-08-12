/*
This example will receive multiple universes via Artnet and control a strip of ws2811 leds via 
Adafruit's NeoPixel library: https://github.com/adafruit/Adafruit_NeoPixel
This example may be copied under the terms of the MIT license, see the LICENSE file for details
*/

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <ArtnetWifi.h>
#include <Adafruit_NeoPixel.h>
#include "ws2812Driver.h"
//#include <portmacro.h>
#include <SPI.h>
#include <E131.h>
#include <APA102.h>

//Wifi settings
const char* ssid = "iptime_min";  //"NETGEAR67";         //"DaeYang";  //"iptime";    //
const char* password = "daeyang5";  //"magicalbird957";  //"daeyang5"; //

IPAddress local_ip(192,168,1,13);  //0,41); 
IPAddress gateway_ip(192,168,1,1);
IPAddress subnet_ip(255,255,255,0);

const int startUniverse = 1; // CHANGE FOR YOUR SETUP most software this is 1, some software send out artnet first universe as 0.

// Neopixel settings
const int numLeds = 512;  //1024;  //240; // change for your setup
const int numberOfChannels = numLeds * 3; // Total number of channels you want to receive (1 led = 3 channels)
const int numLeds1 = numLeds/2;  //240; // change for your setup
const int numberOfChannelsA = numLeds1 * 3; // Total number of channels you want to receive (1 led = 3 channels)
const byte DMX_TX_A = 0;  //D3;    //2;
const byte DMX_TX_B = 2;  //D4;    //2;
//Adafruit_NeoPixel leds = Adafruit_NeoPixel(numLeds1, DMX_TX_A, NEO_GRB + NEO_KHZ800); //NEO_GRB
//Adafruit_NeoPixel leds1 = Adafruit_NeoPixel(numLeds1, DMX_TX_B, NEO_GRB + NEO_KHZ800); //NEO_GRB

// Define which pins to use.
const uint8_t dataPin = D7;   //MOSI;   //11;
const uint8_t clockPin = D5;  //SCK;    //12;

// Check if we got all universes
//const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
const int maxUniverses = numberOfChannels / 384 + ((numberOfChannels % 384) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;
int previousDataLength1 = 0;
int preSequence = 0;

// Create an object for writing to the LED strip.
APA102<dataPin, clockPin> ledStrip;

// E1.31 settings
E131 e131;
// Artnet settings
ArtnetWifi artnet;

ws2812Driver pixDriver;

// connect to wifi – returns true if successful or false if not
boolean ConnectWifi(void)
{
  boolean state = true;
  int i = 0;

  WiFi.begin(ssid, password);
  WiFi.config(local_ip, gateway_ip, subnet_ip);
  Serial.println("");
  Serial.println("Connecting to WiFi");
  
  // Wait for connection
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 20){
      state = false;
      break;
    }
    i++;
  }
  if (state){
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("Connection failed.");
  }
  
  return state;
}

void initTest()
{
  initTest_BLUE();
  delay(500);
  initTest_GREEN();
  delay(500);
  initTest_RED();
  delay(500);
  initTest_OFF();
  delay(500);
}
void initTest_BLUE()
{
  ledStrip.startFrame();
  for (int i = 0 ; i < numLeds ; i++) {
    ledStrip.sendColor(127,0,0);
  }
  ledStrip.endFrame(numLeds);

}
void initTest_GREEN()
{
  ledStrip.startFrame();
  for (int i = 0 ; i < numLeds ; i++) {
    ledStrip.sendColor(0,127,0);
  }
  ledStrip.endFrame(numLeds);

}
void initTest_RED()
{
  ledStrip.startFrame();
  for (int i = 0 ; i < numLeds ; i++) {
    ledStrip.sendColor(0,0,127);
  }
  ledStrip.endFrame(numLeds);

}
void initTest_OFF()
{
  ledStrip.startFrame();
  for (int i = 0 ; i < numLeds ; i++) {
    ledStrip.sendColor(0,0,0);
  }
  ledStrip.endFrame(numLeds);
}

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  sendFrame = 1;

    Serial.printf("%d,%d,%d\r\n", length, universe, sequence);
//    Serial.println("art-net");


  // Store which universe has got in
  if ((universe - startUniverse) < maxUniverses)
    universesReceived[universe - startUniverse] = 1;
    
  // Copy DMX data to the pixels buffer
  pixDriver.setBuffer(0, universe * 384, data, length);

  if (sequence != preSequence)
  {
    preSequence = sequence;
     // Reset universeReceived to 0
 //   memset(universesReceived, 0, maxUniverses);
  }
  else
  {
     for (int i = 0 ; i < maxUniverses ; i++)
      {
        if (universesReceived[i] == 0)
        {
     //     Serial.println("Broke");
          sendFrame = 0;
          break;
        }
      }
  }

//  if (sendFrame)
  {
  }
  // read universe and put into the right part of the display buffer
/*  for (int i = 0; i < length / 3; i++)
  {
    if (universe <= 3)
    {
      int led = i + (universe - startUniverse) * (previousDataLength / 3);
      if (led < numLeds) {
//        leds.setPixelColor(led, data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
          SPI.transfer(0xe2); 
          SPI.transfer(data[i * 3]); 
          SPI.transfer(data[i * 3 + 1]); 
          SPI.transfer(data[i * 3 + 2]);
      }
    }
    else
    {
      int led = i + ((universe - 4) - startUniverse) * (previousDataLength1 / 3);
      if (led < numLeds1){
//        leds1.setPixelColor(led, data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
          SPI.transfer(0xe0); 
          SPI.transfer(data[i * 3]); 
          SPI.transfer(data[i * 3 + 1]); 
          SPI.transfer(data[i * 3 + 2]);
      }
    }
  }
  */
  if (universe <= 3)
    previousDataLength = length;     
  else
    previousDataLength1 = length;     
  
  if (sendFrame)
  {
    uint8_t* data1 = pixDriver.getBuffer(0);
    
    ledStrip.startFrame();
    for (int i = 0 ; i < numLeds ; i++) {
      ledStrip.sendColor(data1[i*3],data1[i*3+1],data1[i*3+2]);  //0,127,0);
    }
    ledStrip.endFrame(numLeds);

    // Reset universeReceived to 0
    memset(universesReceived, 0, maxUniverses);
 //  delay(10);

  }
}

void setup()
{
  Serial.begin(115200);
//  ConnectWifi();
//  artnet.begin();
  pixDriver.clearBuffer(0,0);
     /* Choose one to begin listening for E1.31 data */
    e131.begin(ssid, password,(IPAddress)local_ip, (IPAddress)subnet_ip, (IPAddress)gateway_ip, (IPAddress)(0,0,0,0));                       /* via Unicast on the default port */
    //e131.beginMulticast(ssid, passphrase, UNIVERSE);  /* via Multicast for Universe 1 */
 
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

//  leds.begin();
//  leds1.begin();
  initTest();
  
  // this will be called for each packet received
//  artnet.setArtDmxCallback(onDmxFrame);
}

void loop()
{
  // we call the read function inside the loop
//  artnet.read();
 
    
  /* Parse a packet and update pixels */
   if(int length = e131.parsePacket()) {
          sendFrame = 1;
          int universe = e131.universe;
          byte sequence = e131.sequence;
//          Serial.printf("%d, %d, %d\r\n", length, universe, sequence);
//          Serial.println("e131");

           // Store which universe has got in
            if ((universe - startUniverse) < maxUniverses)
              universesReceived[universe - startUniverse] = 1;
          
            // Copy DMX data to the pixels buffer
            pixDriver.setBuffer(0, (universe - startUniverse) * 384, e131.data, length);

          if (sequence != preSequence)
          {
            preSequence = sequence;
            // Reset universeReceived to 0
//            memset(universesReceived, 0, maxUniverses);
          }
          else
          {
            for (int i = 0 ; i < maxUniverses; i++)    //maxUniverses ; i++)
            {
              if (universesReceived[i] == 0)
              {
           //     Serial.println("Broke");
                sendFrame = 0;
                break;
              }
            }
          }

        if (sendFrame)
        {
            uint8_t* data1 = pixDriver.getBuffer(0);
            
            ledStrip.startFrame();
            for (int i = 0 ; i < numLeds ; i++) {
              ledStrip.sendColor(data1[i*3],data1[i*3+1],data1[i*3+2]);  //0,127,0);
            }
            ledStrip.endFrame(numLeds);

            //delay(10);
            // Reset universeReceived to 0
            memset(universesReceived, 0, maxUniverses);
        
        }
    }
 
}

