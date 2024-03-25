#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>

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
RHDatagram manager(rf95, GS_ADDRESS);


float pressure;
float tempreture;
float alt;
float P1 = 0;
float T1 = 0;
float a = 29.3;
int command;
float time1 = 0;
float time2 = 0;
float time3 = 0;
char P[10];
char T[6];

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);
  delay(100);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!manager.init())
  {
    Serial.println ("init failed");
    while (1);
  }
  Serial.println("Radio module started successfully");
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(20, false);

}

char buf[RH_RF95_MAX_MESSAGE_LEN];
char command_array[2] = " ";
char tel[25];

void loop() 
{
  if(manager.available())
  {
    uint8_t len = sizeof(buf);   
    if (manager.recvfrom((uint8_t *)buf, &len))
    {
      time2 = millis();
      if (Serial.available() != 0)
      {
        command = Serial.parseInt();
        itoa(command, command_array, 10);
        manager.sendto((uint8_t *)command_array, sizeof(command_array), CAN_ADDRESS);
        manager.waitPacketSent();
      }
      tel[0] = 0;
      strcpy(tel, buf);
      char* token = strtok(buf, ",");
      if (token != NULL)
      {
        strcpy(P, token);
        token = strtok(NULL, ",");
      }
      if (token != NULL)
      {
        strcpy(T, token);
        token = strtok(NULL, ",");
      }
      if(P1 == 0 || T1 == 0)
      {
        P1 = atof(P);
        T1 = atof(T) + 273.15;
      }
      pressure = atof(P);
      tempreture = atof(T); 
      alt = a * (T1 + tempreture)/2 * log(P1/pressure);
      {
        time1 = time2;
      }
      time3 = (time2 - time1)/1000;
      Serial.println(pressure);
      Serial.println(tempreture - 273.15);
      Serial.println(alt);
      Serial.println(time3, 1);
    }
  }
}