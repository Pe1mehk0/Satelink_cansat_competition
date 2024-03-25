#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <RHDatagram.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <floatToString.h>


#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#endif

#define RF95_FREQ 433.0

#define GS_ADDRESS 1
#define CAN_ADDRESS 2
#define ROVER_ADDRESS 3


RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, CAN_ADDRESS);
Adafruit_BMP280 bmp280;
File myFile;


float mean_pressure = 0; 
float mean_temp = 0;
int flag = 0;
int command = 0;
float temp_check, pres_check;
int counter_for_buzzer = 0;
const int buzzer = 1;


char pressure1[10];
char temp_ar[6];
char command1[2];
char flag1[2];


int redPin = 9;
int greenPin = 6;
int bluePin = 10;

void setup() {

  pinMode(buzzer, OUTPUT);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);

  bool all_ok = true;

  if (!manager.init())
  {
    all_ok = false;

  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(20, false);
  if(!bmp280.begin(0x76))
  {
    all_ok = false;
  }
  digitalWrite(8, HIGH);
  digitalWrite(11, LOW);
  if(!SD.begin(11))
  {
    all_ok = false;
  }
  digitalWrite(8, LOW);
  digitalWrite(11, HIGH);
  if(all_ok)
  {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }else
  {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);
  }
}

char buf[RH_RF95_MAX_MESSAGE_LEN];
char tel[25];

void loop() 
{
  if(counter_for_buzzer % 3 == 0)
  {
    tone(buzzer, 1000);
  }
  else
  {
    noTone(buzzer); 
  }
  mean_temp = bmp280.readTemperature();
  mean_pressure = bmp280.readPressure();
  floatToString(mean_pressure, pressure1, sizeof(pressure1), 2);
  floatToString(mean_temp, temp_ar, sizeof(temp_ar), 2);
  itoa(command, command1, 10);
  itoa(flag, flag1, 10);
  tel[0] = '\0';
  strcat(tel, pressure1);
  strcat(tel, ",");
  strcat(tel, temp_ar);
  strcat(tel, ",");
  strcat(tel, command1);
  strcat(tel, flag1);
  manager.sendto((uint8_t *)tel, sizeof(tel), ROVER_ADDRESS);
  manager.waitPacketSent();
  manager.sendto((uint8_t *)tel, sizeof(tel), GS_ADDRESS);
  manager.waitPacketSent();
  flag = 0;
  if(manager.waitAvailableTimeout(300))
  {
    uint8_t len = sizeof(buf);   
    if (manager.recvfrom((uint8_t *)buf, &len))
    {
      command = atoi(buf);
      flag = 1;
    }
  }
  counter_for_buzzer += 1;
  digitalWrite(8, HIGH);
  digitalWrite(11, LOW);
  myFile = SD.open("results.txt", FILE_WRITE);
   if (myFile) 
   {
    myFile.println(tel);
    myFile.close();
  }
  digitalWrite(8, LOW);
  digitalWrite(11, HIGH);
}