#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>
#include <Servo.h>
#include <SD.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "DFRobot_RGBLCD1602.h"
#include  <MQUnifiedsensor.h>

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#endif

#define RF95_FREQ 433.4

#define GS_ADDRESS 1
#define CAN_ADDRESS 2
#define ROVER_ADDRESS 3

#define         Board                   ("Adafruit Feather M0")
#define         RatioMQ3CleanAir          (60) //RS / R0 = 60 ppm 
#define         RatioMQ4CleanAir          (4.4)  //RS / R0 = 4.4 ppm 
#define         RatioMQ135CleanAir        (3.6) //RS / R0  = 10 ppm 
#define         RatioMQ7CleanAir          (27.5) //RS / R0 = 27.5 ppm  
#define         RatioMQ8CleanAir          (70) //RS / R0 = 70 ppm   
#define         RatioMQ9CleanAir          (9.6) //RS / R0 = 9.6 ppm 
#define         ADC_Bit_Resolution        (12) // 12 bit ADC 
#define         Voltage_Resolution        (5) //  Volt resolution to calc the voltage
//Declare Sensor
MQUnifiedsensor MQ3(Board, Voltage_Resolution,  ADC_Bit_Resolution, A5, "MQ3");
MQUnifiedsensor MQ4(Board, Voltage_Resolution,  ADC_Bit_Resolution, A4, "MQ4");
MQUnifiedsensor MQ135(Board, Voltage_Resolution,  ADC_Bit_Resolution, A3, "MQ135");
MQUnifiedsensor MQ7(Board, Voltage_Resolution,  ADC_Bit_Resolution, A2, "MQ7");
MQUnifiedsensor MQ8(Board, Voltage_Resolution,  ADC_Bit_Resolution, A1, "MQ8");
MQUnifiedsensor MQ9(Board, Voltage_Resolution,  ADC_Bit_Resolution, A0, "MQ9");

char dtaUart[15];
char dtaLen = 0;
DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x2D ,/*lcdCols*/16,/*lcdRows*/2);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, ROVER_ADDRESS);
Servo myservo; 
Servo myservo1;
const int chipSelect = 9;
unsigned long last_command_time = 0;
unsigned long interval = 0;
int counter_for_time = 0;
int counter_for_cycle = 0;
unsigned long limit = 180000; //Added and didn't test
unsigned long counter2 = 0;

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
void setSpeed7(int speed)
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


  digitalWrite(8, LOW);
  digitalWrite(chipSelect, HIGH);

  manager.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(20, false);
  
  myservo.attach(10); 
  myservo1.attach(11); 
  
  digitalWrite(8, HIGH);
  digitalWrite(chipSelect, LOW);
  SD.begin(chipSelect);
  digitalWrite(8, LOW);
  digitalWrite(chipSelect, HIGH);
  
  lcd.init();
  MQ3.init();
  MQ3.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ3.setR0(0.45);
  MQ4.init();
  MQ4.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ4.setR0(14.23);
  MQ135.init();
  MQ135.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ135.setR0(9.03);
  MQ7.init();
  MQ7.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ7.setR0(5.90);
  MQ8.init();
  MQ8.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ8.setR0(0.91);
  MQ9.init();
  MQ9.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ9.setR0(13.39);
}

char buf[26]; //RH_RF95_MAX_MESSAGE_LEN];
int flag, command;
int telemetry;
int commands[50];
signed int rssi[3000];
int index_of_last_number = 0;
int counter;
int speed;
int index_of_last_rssi = -1;

void loop() 
{
  if(manager.available())
  {
    uint8_t len = 26; //sizeof(buf);
    if (manager.recvfrom((uint8_t *)buf, &len))
    {
      command = 0;
      flag = 2;
      counter = 0;
      for (int i = 0; i < len; ++i)
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
      //if(command != 0) last_command_time = millis();
      if (command == 9)
      {
        commands[++index_of_last_number] = 7;
        last_command_time = millis();
        for (int i = 0; i <= index_of_last_number; ++i)
        {
          switch(commands[i])
          {
            case 1: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed1(speed); 
                delay(500); 
              } 
            } break;
            case 2: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed2(speed); 
                delay(500); 
              } 
            } break;
            case 3: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed3(speed); 
                delay(500); 
              } 
            } break;
            case 4: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed4(speed); 
                delay(500); 
              } 
            } break;
            case 5: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed5(speed); 
                delay(500); 
              } 
            } break;
            case 6: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed6(speed); 
                delay(500); 
              } 
            } break;
            case 7:{
              setSpeed7(50);
            } break;
            case 8: {
              MQ3.update();
              MQ4.update();
              MQ135.update();  
              MQ7.update();
              MQ8.update();
              MQ9.update();

              MQ3.setA(0.3934); MQ3.setB(-1.504); //Alcohol
              float  Alcohol = MQ3.readSensor(); 

              MQ3.setA(4.8387); MQ3.setB(-2.68); //Benzene
              float  Benzene = MQ3.readSensor(); 
              
              MQ3.setA(7585.3); MQ3.setB(-2.849); //Hexane
              float  Hexane = MQ3.readSensor(); 

              MQ4.setA(1012.7); MQ4.setB(-2.786); //CH4
              float  CH4 = MQ4.readSensor(); 

              MQ4.setA(30000000); MQ4.setB(-8.308); //smoke  
              float smoke = MQ4.readSensor(); 
            
              MQ135.setA(110.47); MQ135.setB(-2.862);  //CO2 
              float CO2 = MQ135.readSensor(); 
              
              MQ135.setA(44.947); MQ135.setB(-3.445);  // Toluene
              float Toluene = MQ135.readSensor(); 
              
              MQ135.setA(102.2 );  MQ135.setB(-2.473); //NH4 
              float NH4 = MQ135.readSensor(); 
              
              MQ135.setA(34.668);  MQ135.setB(-3.369); //Acetone
              float Acetone = MQ135.readSensor(); 
            
              MQ7.setA(99.042);  MQ7.setB(-1.518); //CO
              float CO = MQ7.readSensor(); 

              MQ8.setA(976.97);  MQ8.setB(-0.688); // H2
              float H2 = MQ8.readSensor();

              MQ9.setA(1000.5);  MQ9.setB(-2.186); //flamable gas
              float FG = MQ9.readSensor();
              
              digitalWrite(8, HIGH);
              digitalWrite(chipSelect, LOW);

              File myFile = SD.open("results.txt", FILE_WRITE);

              if (myFile) 
              {
                myFile.print("Alcohol:  "); myFile.println(Alcohol);
                myFile.print("Benzene:  "); myFile.println(Benzene);
                myFile.print("Hexane:   "); myFile.println(Hexane);
                myFile.print("Methane:  "); myFile.println(CH4);
                myFile.print("Smoke:    "); myFile.println(smoke);
                myFile.print("CO2:      "); myFile.println(CO2);
                myFile.print("Toluene:  "); myFile.println(Toluene);
                myFile.print("NH4:      "); myFile.println(NH4);
                myFile.print("Acetone:  "); myFile.println(Acetone);  
                myFile.print("CO:       "); myFile.println(CO);
                myFile.print("H2:       "); myFile.println(H2);
                myFile.print("FG:       "); myFile.println(FG);
                myFile.println("--------------------------------------------------------");
                for (int i = 0; i <= index_of_last_rssi; ++i)
                {
                  myFile.println(rssi[i]);
                }
                myFile.close();
                for (int i = 0; i <= index_of_last_rssi; ++i)
                {
                  rssi[i] = 0;
                }
                index_of_last_rssi = -1;
              }
              // else
              // {

              // }
              digitalWrite(8, LOW);
              digitalWrite(chipSelect, HIGH);
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Alcohol ");
              lcd.print(Alcohol);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("Benzene ");
              lcd.print(Benzene);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);

              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Hexane  ");
              lcd.print(Hexane);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CH4     ");
              lcd.print(CH4);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Smoke   ");
              lcd.print(smoke);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CO2     ");
              lcd.print(CO2);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Toluene ");
              lcd.print(Toluene);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("NH4     ");
              lcd.print(NH4);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Acetone ");
              lcd.print(Acetone);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CO      ");
              lcd.print(CO);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("H2      ");
              lcd.print(H2);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("FG      ");
              lcd.print(FG);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);
              lcd.clear();
            }
             break;
            default:
              break;
          }
        }
        for (int i = 0; i <= index_of_last_number; ++i)
        {
          commands[i] = 0;
        }
        index_of_last_number = -1;
      }
      else if(command != 0)
      {
        if(flag == 1)
        {
          if(command == 8) commands[++index_of_last_number] = 7;
          commands[++index_of_last_number] = command;

          last_command_time = millis(); // - 180000*counter_for_time;
        }
        if (flag == 0)
        {
          if(index_of_last_number != -1 && commands[index_of_last_number] != command)
          {
            if(command == 8) commands[++index_of_last_number] = 7;
            commands[++index_of_last_number] = command;
            last_command_time = millis();
          }
        }
      }
      rssi[++index_of_last_rssi] = rf95.lastRssi();
    }
  }
  if(millis() - last_command_time >= 120000)
  {
    commands[++index_of_last_number] = 7;
        last_command_time = millis();
        for (int i = 0; i <= index_of_last_number; ++i)
        {
          switch(commands[i])
          {
            case 1: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed1(speed); 
                delay(500); 
              } 
            } break;
            case 2: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed2(speed); 
                delay(500); 
              } 
            } break;
            case 3: {
              for(speed = 50; speed <= 59; speed+= 1) 
              { 
                setSpeed3(speed); 
                delay(500); 
              } 
            } break;
            case 4: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed4(speed); 
                delay(500); 
              } 
            } break;
            case 5: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed5(speed); 
                delay(500); 
              } 
            } break;
            case 6: {
              for(speed = 50; speed >= 45; speed-= 1) 
              { 
                setSpeed6(speed); 
                delay(500); 
              } 
            } break;
            case 7:{
              setSpeed7(50);
            } break;
            case 8: {
              MQ3.update();
              MQ4.update();
              MQ135.update();  
              MQ7.update();
              MQ8.update();
              MQ9.update();

              MQ3.setA(0.3934); MQ3.setB(-1.504); //Alcohol
              float  Alcohol = MQ3.readSensor(); 

              MQ3.setA(4.8387); MQ3.setB(-2.68); //Benzene
              float  Benzene = MQ3.readSensor(); 
              
              MQ3.setA(7585.3); MQ3.setB(-2.849); //Hexane
              float  Hexane = MQ3.readSensor(); 

              MQ4.setA(1012.7); MQ4.setB(-2.786); //CH4
              float  CH4 = MQ4.readSensor(); 

              MQ4.setA(30000000); MQ4.setB(-8.308); //smoke  
              float smoke = MQ4.readSensor(); 
            
              MQ135.setA(110.47); MQ135.setB(-2.862);  //CO2 
              float CO2 = MQ135.readSensor(); 
              
              MQ135.setA(44.947); MQ135.setB(-3.445);  // Toluene
              float Toluene = MQ135.readSensor(); 
              
              MQ135.setA(102.2 );  MQ135.setB(-2.473); //NH4 
              float NH4 = MQ135.readSensor(); 
              
              MQ135.setA(34.668);  MQ135.setB(-3.369); //Acetone
              float Acetone = MQ135.readSensor(); 
            
              MQ7.setA(99.042);  MQ7.setB(-1.518); //CO
              float CO = MQ7.readSensor(); 

              MQ8.setA(976.97);  MQ8.setB(-0.688); // H2
              float H2 = MQ8.readSensor();

              MQ9.setA(1000.5);  MQ9.setB(-2.186); //flamable gas
              float FG = MQ9.readSensor();
              
              digitalWrite(8, HIGH);
              digitalWrite(chipSelect, LOW);

              File myFile = SD.open("results.txt", FILE_WRITE);

              if (myFile) 
              {
                myFile.print("Alcohol:  "); myFile.println(Alcohol);
                myFile.print("Benzene:  "); myFile.println(Benzene);
                myFile.print("Hexane:   "); myFile.println(Hexane);
                myFile.print("Methane:  "); myFile.println(CH4);
                myFile.print("Smoke:    "); myFile.println(smoke);
                myFile.print("CO2:      "); myFile.println(CO2);
                myFile.print("Toluene:  "); myFile.println(Toluene);
                myFile.print("NH4:      "); myFile.println(NH4);
                myFile.print("Acetone:  "); myFile.println(Acetone);  
                myFile.print("CO:       "); myFile.println(CO);
                myFile.print("H2:       "); myFile.println(H2);
                myFile.print("FG:       "); myFile.println(FG);
                myFile.println("--------------------------------------------------------");
                for (int i = 0; i <= index_of_last_rssi; ++i)
                {
                  myFile.println(rssi[i]);
                }
                myFile.close();
                for (int i = 0; i <= index_of_last_rssi; ++i)
                {
                  rssi[i] = 0;
                }
                index_of_last_rssi = -1;
              }
              // else
              // {

              // }
              digitalWrite(8, LOW);
              digitalWrite(chipSelect, HIGH);
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Alcohol ");
              lcd.print(Alcohol);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("Benzene ");
              lcd.print(Benzene);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);

              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Hexane  ");
              lcd.print(Hexane);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CH4     ");
              lcd.print(CH4);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Smoke   ");
              lcd.print(smoke);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CO2     ");
              lcd.print(CO2);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Toluene ");
              lcd.print(Toluene);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("NH4     ");
              lcd.print(NH4);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("Acetone ");
              lcd.print(Acetone);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("CO      ");
              lcd.print(CO);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);


              lcd.clear();
              delay(70);
              lcd.setCursor  (0,0);
              lcd.print("H2      ");
              lcd.print(H2);
              lcd.setCursor (13,0);
              lcd.print("ppm");
              lcd.setCursor  (0,1);
              lcd.print("FG      ");
              lcd.print(FG);
              lcd.setCursor (13,1);
              lcd.print("ppm");
              delay(3000);
              lcd.clear();
            }
             break;
            default:
              break;
          }
        }
        for (int i = 0; i <= index_of_last_number; ++i)
        {
          commands[i] = 0;
        }
        index_of_last_number = -1;
  }
}
