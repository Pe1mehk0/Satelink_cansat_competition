#include <DFRobot_RGBLCD1602.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>
#include <Servo.h>

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#endif

#define RF95_FREQ 433.0

#define GS_ADDRESS 1
#define CAN_ADDRESS 2
#define ROVER_ADDRESS 3

char dtaUart[15];
char dtaLen = 0;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, ROVER_ADDRESS);
DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x2D ,/*lcdCols*/16,/*lcdRows*/4);
Servo myservo; 
Servo myservo1;

void setSpeed1(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(99,0,100,0,180)); 
}
void setSpeed2(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(1,0,100,0,180)); 
}
void setSpeed3(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(50,0,100,0,180)); 
}
void setSpeed4(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(99,0,100,0,180)); 
} 
void setSpeed5(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(1,0,100,0,180)); 
}    
void setSpeed6(int speed) 
{ 
int angle = map(speed, 0, 100, 0, 180); 
myservo.write(angle); 
myservo1.write(map(50,0,100,0,180)); 
} 

void setup() {
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  lcd.init();

  if(!manager.init())
  {
    lcd.print("ERROR2");
    delay(2000);
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(20, false);

  myservo.attach(10); 
  myservo1.attach(11); 
}

char buf[RH_RF95_MAX_MESSAGE_LEN];
int flag, command;
int telemetry;
int commands[50];
int index_of_last_number = 0;
int counter;
int speed;

void loop() 
{
  if(manager.available())
  {
    uint8_t len = sizeof(buf);   
    if (manager.recvfrom((uint8_t *)buf, &len))
    {
      command = 0;
      flag = 2;
      counter = 0;
      for (int i = 0; i < sizeof(buf); ++i)
      {
        if(buf[i] == ',')
        {
          counter += 1;
        }
        if(counter == 2)
        {
          command = buf[i+1] - '0';
          flag = buf[i+2] - '0';
          break;
        }
      } 
      if (command == 9)
      {
        for (int i = 0; i < index_of_last_number + 1; ++i)
        {
          switch(commands[i])
          {
            case 1:
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed1(speed); 
                delay(500); 
              } 
              break;
            case 2:
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed2(speed); 
                delay(500); 
              } 
              break;
            case 3:
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed3(speed); 
                delay(500); 
              } 
              break;
            case 4:
              for(speed = 50; speed >= 41; speed-= 1) 
              { 
                setSpeed4(speed); 
                delay(500); 
              } 
              break;
            case 5:
              for(speed = 50; speed >= 41; speed-= 1) 
              { 
                setSpeed5(speed); 
                delay(500); 
              } 
              break;
            case 6:
              for(speed = 50; speed >= 41; speed-= 1) 
              { 
                setSpeed6(speed); 
                delay(500); 
              } 
              break;
            default:
              break;
          }
        }
        for (int i = 0; i < index_of_last_number + 1; ++i)
        {
          commands[i] = 0;
        }
        index_of_last_number = 0;
      }
      else
      {
        if(flag == 1)
        {
          commands[index_of_last_number + 1] = command;
          index_of_last_number +=1;
        }
        if (flag == 0)
        {
          if(commands[index_of_last_number] != command)
          {
            commands[index_of_last_number + 1] = command;
            index_of_last_number +=1;
          }
        }
      }
    }
  }
}