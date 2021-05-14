#include <SoftwareSerial.h>
#include <Servo.h> 
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified BMPSensor = Adafruit_BMP085_Unified(10085);

int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control
int redpin = 11; // select the pin for the red LED
int bluepin =9; // select the pin for the  blue LED
int greenpin =8; // select the pin for the green LED
int servoPin = 12; 
Servo Servo1;   // Create a servo object 
int poser = 90; // initial position of server
int pos_check; //  position of server checker
int val=1;
int gear_speed;
unsigned long deadline = 500;
const int TrigPin = 10;
const int EchoPin = 13;
float distance;
int ThermistorPin = A0;
int Vo;
float R1 = 10000; // value of R1 on board
float logR2, R2, T;
float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741; //steinhart-hart coeficients for thermistor



void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
}

void advance()          //Move forward
{
  analogWrite (E1,gear_speed);      //PWM Speed Control
  digitalWrite(M1,HIGH);
}

void back_off (char b)          //Move backward
{
  analogWrite (E1,b);
  digitalWrite(M1,LOW);
}

void turn_left(char c)          //Move forward
{
  analogWrite (E2,c);      //PWM Speed Control
  digitalWrite(M2,HIGH);
}
void turn_right(char d)          //Move forward
{
  analogWrite (E2,d);      //PWM Speed Control
  digitalWrite(M2,LOW);
}

void setup(void)
{
  int i;
  for(i=4;i<=7;i++)
   {pinMode(i, OUTPUT);}
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  Servo1.attach(servoPin);
  Serial.begin(9600);      //Set Baud Rate
  Serial.println("ROVER is ready !");
}

void loop(void)
{
  if(Serial.available()>0)
 {
  char val = Serial.read();
  delay(2);
  if(val != -1)
    {
      switch(val)
      {
      case'1': gear_speed=150; //1 gear
       break;
      case'2': gear_speed=200; //2 gear
       break;
      case'3': gear_speed=255; //3 gear
       break;
      case 'B'://Move Forward
        advance ();  
        analogWrite(11, 0);  //R
        analogWrite(9, 255); //B
        analogWrite(8, 0);   //G
        //delay(20);       
        break;
      case 'C'://Move Backward
        back_off (255);   //move back in max speed
        analogWrite(11, 255);  //R
        analogWrite(9, 0);     //B
        analogWrite(8, 0);     //G
       // delay(20);
        break;
      case 'A'://Turn left
        turn_left (255);
        analogWrite(11, 255);  //R
        analogWrite(9, 255);  //B
        analogWrite(8, 255);  //G
       // delay(20);                
        break;
      case 'D'://Turn right
        turn_right (255);
        analogWrite(11, 255); //R
        analogWrite(9, 255); //B
        analogWrite(8, 255); //G
        //delay(20);           
        break;   
      case ' ':
        stop();
        analogWrite(11, 0);  //R
        analogWrite(9, 0);  //B
        analogWrite(8, 255);  //G
        break;
      case 'E' :     //Left view
         if(pos_check==90)
         {poser=0;
          Servo1.write(poser);
          analogWrite(11, 255);
          analogWrite(9, 0);
          analogWrite(8, 255);}
         else
         {poser -= 3; //than position of servo motor decreases by 3 (clockwise)
          Servo1.write(poser);
          analogWrite(11, 255);
          analogWrite(9, 0);
          analogWrite(8, 255);}
         delay(20); 
         break;
      case 'F' :     //Front view
         Servo1.write(90); 
         pos_check=90;
         analogWrite(11, 255);
         analogWrite(9, 0);
         analogWrite(8, 255);         
         break;   
      case '5' :    // Right view
         if(pos_check==90)
         {poser=180;
          Servo1.write(poser);      
          analogWrite(11, 255);
          analogWrite(9, 0);
          analogWrite(8, 255);}
         else
         {poser += 3; //than position of servo motor increases by 3 ( anti clockwise)
          Servo1.write(poser);      
          analogWrite(11, 255);
          analogWrite(9, 0);
          analogWrite(8, 255);}
         delay(20);        
         break;
      case'u':
        digitalWrite(TrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(TrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW);
        distance = pulseIn(EchoPin, HIGH) / 58.00;
       // Serial.print("Distance: "); 
        Serial.print(distance);
        Serial.print(" cm");
        Serial.println();
      //  delay(20);  
        break;
      case't':
        Vo = analogRead(ThermistorPin);
        R2 = R1 * (1023.0 / (float)Vo - 1.0); //calculate resistance on thermistor
        logR2 = log(R2);
        T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); // temperature in Kelvin
        T = T - 273.15; //convert Kelvin to Celcius
        // T = (T * 9.0)/ 5.0 + 32.0; //convert Celcius to Farenheit
        //Serial.print("Temperature: "); 
        Serial.print(T);
        Serial.println(" C"); 
        //delay(20);  
        break; 
      case'p':
        sensors_event_t event;
        BMPSensor.getEvent(&event);
        Serial.print(1000+event.pressure*10);
        Serial.println(" hPa");    
        break;
    }  
  }
 } 
}
