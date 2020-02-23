#include "REG_SDM120.h"
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <MicroGear.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFiUdp.h>

// ********** Set Port **********

#define  KEY          D0
#define  LED          D4
#define  DIO1         D1
#define  DIO2         D2
#define  DIO3         D3
#define  ANA0         A0
#define  SDCS         D8

// ********** Set **********

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

unsigned long epoch;

WiFiUDP udp;

// const char* ssid     = "PUTTICHAMP";
// const char* password = "p9210764";
//const char* ssid     = "EESS_MAKER_LAB";
//const char* password = "1212312121";
const char* ssid     = "NSTDA-D";
const char* password = "i0t#dEsS";

#define APPID   "Smartmetersd230"
#define KEY     "CN7mSgqC3nP8HPB"
#define SECRET  "lTdK3lcC7oPAevaNx3kJGO9jZ"
#define ALIAS   "smartmeter"

bool SDFIRST;

String Power = "";
// test
WiFiClient client;
int timer_publish = 0;
int timer_reconnect = 0;
MicroGear microgear(client);

//String log_sd(String log);

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  Serial.println((char *)msg);
}

/* When a microgear is connected, do this */
void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.println("Connected to NETPIE...");
  /* Set the alias of this microgear ALIAS */
  microgear.setAlias(ALIAS);
  timer_reconnect = 0;
  timer_publish = 0;
  //log_sd("Connected to NETPIE");
}

ModbusMaster node;
//01 04 00 00 00 02 71 3F // Test 30001
//------------------------------------------------
// Convent 32bit to float
//------------------------------------------------
float HexTofloat(uint32_t x) {
  return (*(float*)&x);
}

uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}
//------------------------------------------------

float Read_Meter_float(char addr , uint16_t  REG) {
  float i = 0;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  node.begin(addr, Serial);
  result = node.readInputRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
  delay(500);
  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
    i = HexTofloat(value);
    //Serial.println("Connec modbus Ok.");
    return i;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    delay(1000);
    return 0;
  }
}

void GET_METER_EASTRON() {     // Update read all data
  delay(1000);                              // เคลียบัสว่าง
    for (char i = 0; i < Total_of_EASTRON ; i++){
      DATA_METER_EASTRON [i] = Read_Meter_float(ID_meter_EASTRON, Reg_addr_EASTRON[i]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
    }
}
void METER_EASTRON() {
  GET_METER_EASTRON();
  Serial.println();
  Serial.println(F("****************METER_EASTRON*******************"));
  Serial.print("Voltage = "); Serial.print(DATA_METER_EASTRON[0]);Serial.println(" VAC");
  Serial.print("Current = "); Serial.print(DATA_METER_EASTRON[1]);Serial.println(" Amps");
  Serial.print("Active Power= "); Serial.print(DATA_METER_EASTRON[2]);Serial.println(" Watts");
  Serial.print("Power Factor = "); Serial.println(DATA_METER_EASTRON[3]);
  Serial.print("Frequency = "); Serial.print(DATA_METER_EASTRON[4]);Serial.println(" Hz");
  Serial.print("Total Active Energy = "); Serial.print(DATA_METER_EASTRON[5]);Serial.println(" kWh");
}

void METER_MITSU() {

  node.begin(ID_meter_Mitsu, Serial);
for(int i=0;i<2;i++){
  uint8_t result = node.readHoldingRegisters(ADDR_START, ADDR_NUMBER);
      if (result == node.ku8MBSuccess) {
          Serial.println(F("****************METER_MITSU*******************"));

          Serial.print("Line Voltage (RMS) (V) >>>>> ");
          DATA_METER_Mitsu[0] = node.getResponseBuffer(0x02)/100.0f;
          Serial.println(DATA_METER_Mitsu[0]); // Line Voltage (RMS) (V)

          Serial.print("Frequency (Hz) >>>>> ");
          DATA_METER_Mitsu[3] = node.getResponseBuffer(0x05)/10.0f;
          Serial.println(DATA_METER_Mitsu[3]); // Frequency (Hz)

          Serial.print("Active Energy (Wh) imp+exp >>>>> ");
          DATA_METER_Mitsu[5] = node.getResponseBuffer(0x0A);
          Serial.println(DATA_METER_Mitsu[5]); // Active Energy (Wh) imp+exp

          Serial.print("Line Current (RMS) (A) >>>>> ");
          DATA_METER_Mitsu[2] = node.getResponseBuffer(0x0C)/100.0f;
          Serial.println(DATA_METER_Mitsu[2]); // Line Current (RMS) (A)

          Serial.print("Active Power (W) >>>>> ");
          DATA_METER_Mitsu[4] = node.getResponseBuffer(0x0F)/1000.0f;
          Serial.println(DATA_METER_Mitsu[4]); // Active Power (W)

          // รวมค่าทั้งหมดเป็นสตริงไว้ที่ตัวแปร msg สำหรับส่งค่าผ่าน netpie
          //String msg = active_energy+","+active_power+","+line_current+","+line_voltage+","+frequency;
          //Serial.println(msg);
          }
    }
}

void test_read_sd (void) {
  char a;
  if (SDFIRST && !SD.begin (SDCS)) Serial.print ("Card Failed !\r\r");
  else {
    SDFIRST = 0;
    File df = SD.open ("readme.txt");
    if (!df) Serial.print ("No File readme.txt\r\r");
    else {
      while (df.available ()) {
        a = df.read ();
        Serial.print (a);
      }
      df.close ();
      Serial.print ("\r\r");
    }
  }
}
// String log_sd (String log) {
//   char a;
//   if (SDFIRST && !SD.begin (SDCS)) Serial.print ("error opening log.txt\r\r");
//   else {
//     SDFIRST = 0;
//     File df = SD.open ("log.txt");
//     if (!df) Serial.print ("No File readme.txt\r\r");
//     else {
//       Serial.print("Writing ");
//       df.println(log);
//       df.close ();
//       Serial.print ("\r\r");
//     }
//   }
// }

void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//**************************************************************************************************************
void setup() {
  Serial.begin(2400);
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  //pinMode (KEY,INPUT);
  pinMode (LED,OUTPUT);
  pinMode (DIO1,OUTPUT);
  pinMode (DIO2,OUTPUT);
  pinMode (DIO3,OUTPUT);
  pinMode (ANA0,INPUT);
  pinMode (SDCS,OUTPUT);
  digitalWrite (LED,HIGH);
  delay (500);
  for (int i=0;i<=1;i++) {
    digitalWrite (LED,LOW);
    delay (100);
    digitalWrite (LED,HIGH);
    delay (150);
  }

  SDFIRST = 1;

  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);

  Serial.println("Starting...");

  /* Initial WIFI, this is just a basic method to configure WIFI on ESP8266.                       */
  /* You may want to use other method that is more complicated, but provide better user experience */
  if (WiFi.begin(ssid, password)) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /* Initial with KEY, SECRET and also set the ALIAS here */
  microgear.init(KEY,SECRET,ALIAS);

  /* connect to NETPIE to a specific APPID */
  microgear.connect(APPID);


  Serial.println();
  Serial.println();
  Serial.println(F("****************RS485 RTU SDM230*******************"));

}

void loop() {
  // float x = Read_Meter_float(ID_meter,Reg_Volt);


  if (microgear.connected())
  {
    Serial.println("connected");
    microgear.loop();
    if (timer_publish >= 500)
    {
            Serial.println("Publish...");
            //METER_MITSU();
            METER_EASTRON();

            Power = "";
            Power += DATA_METER_EASTRON[1];
            Power += "/";
            Power += DATA_METER_EASTRON[0];
            Power += "/";
            Power += DATA_METER_EASTRON[2];
            Power += "/";
            Power += DATA_METER_EASTRON[3];
            Power += "/";
            Power += DATA_METER_EASTRON[4];
            Power += "/";
            Power += DATA_METER_EASTRON[5];
            epoch = 0;
            Serial.println(Power);
            microgear.chat("Factory/Sensor/Power", Power);
            timer_publish = 0;
    }
    else
     {
       timer_publish += 100;
     }
    }
  else {
        Serial.println("connection lost, reconnect...");
        //log_sd("internet connection lost");
        if (timer_reconnect >= 2000) {
            microgear.connect(APPID);
            timer_reconnect = 0;

        }
        else timer_reconnect += 100;
    }

    delay(1000);
}
