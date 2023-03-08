/* ESTE CODIGO ES PARA EL ARDUINO MEGA CON SHIELD USB HOST PARA PS4 
 * EL RS485 ESTA CONECTADO AL TX Y RX DEL ARDUINO
 */
#include <Arduino.h>
#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


#include <ArduinoJson.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS4BT PS4(&Btd, PAIR);
// Después de esto, simplemente puedes crear la instancia así y luego presionar el botón PS en el dispositivo
//PS4BT PS4(&Btd);

bool printAngle, printTouch, printServ;

uint8_t oldL2Value, oldR2Value;

unsigned long currentTime1 = 0;
unsigned long currentTime2 = 0;
unsigned long currentTime3 = 0;
unsigned long currentTime4 = 0;
float Time = 0;

#define pin_J_MOTOR_X     A0
#define pin_J_MOTOR_Y     A1/// A7 SE INTERCAMBIO PARA PROTOTIPO1
#define pin_J_MOTOR_PUSH  7
#define pin_J_LED_Z       A4    //A6

#define pin_J_SERVO_X   A2
#define pin_J_SERVO_Y   A3
#define pin_J_SERVO_PUSH   8
#define pin_J_SERVO_Z   A5///A4 SE INTERCAMBIO PARA PROTOTIPO1

#define pin_RS485_MODE  4
#define RSTENC          3

int camara_X = 90, camara_Y = 90, led_Z=0;
float cx = 90.0, cy = 90.0;


int J1_Y;
int J1_X;
int J2_Y ;
int J2_X;
int J1_Z;

uint8_t i=0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  //RS485.begin(9600);
  pinMode(pin_J_MOTOR_PUSH,INPUT_PULLUP); 
  pinMode(pin_J_SERVO_PUSH,INPUT_PULLUP);
  pinMode(pin_RS485_MODE,OUTPUT);
  pinMode(RSTENC,OUTPUT);
  digitalWrite(pin_RS485_MODE, HIGH);
  digitalWrite(RSTENC, HIGH);
  delay(3000);
  
  pinMode(3,OUTPUT);

currentTime1 = millis();
currentTime2 = millis();

printServ = true;

}
void loop() {

  StaticJsonDocument<200> doc;
  //TASK1
 
    //Serial.println("  task1");

    Usb.Task();
    PS4.setLed(Green);
    //PS4.setRumbleOn(RumbleLow);
   
  if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
    }
  if (PS4.getButtonClick(OPTIONS)) {
        Serial2.println("AT+KEY=MENU\r\n");
      } 
  if (PS4.getButtonClick(CIRCLE)){//cirQlo
        Serial2.println("AT+KEY=ESC\r\n");
      }
  if (PS4.getButtonPress(SHARE)) {
        digitalWrite(RSTENC, LOW);
      }
  else{
        digitalWrite(RSTENC, HIGH);
      } 
  if (PS4.getButtonClick(UP)) {
        Serial2.println("AT+KEY=UP\r\n");
      } 
  if (PS4.getButtonClick(RIGHT)) {
        Serial2.println("AT+KEY=RIGHT\r\n");
      } 
  if (PS4.getButtonClick(DOWN)) {
        Serial2.println("AT+KEY=DOWN\r\n");
      } 
  if (PS4.getButtonClick(LEFT)) {
        Serial2.println("AT+KEY=LEFT\r\n");
      }
  if (PS4.getButtonClick(CROSS)) {//x
        Serial2.println("AT+KEY=OK\r\n");
      }
  if (PS4.getButtonClick(TRIANGLE)) {
        Serial2.println("AT+KEY=SNAP\r\n");
      }
  if (PS4.getButtonPress(L1) && PS4.getButtonClick(CIRCLE)) {
        Serial2.println("AT+KEY=RECORD\r\n");
      }
  if (PS4.getButtonPress(L1) && PS4.getButtonClick(TRIANGLE)) {
        Serial2.println("AT+KEY=PLAY\r\n");
      }
  if (PS4.getButtonPress(L1) && PS4.getButtonClick(SQUARE)) {
        Serial2.println("AT+KEY=STOP\r\n");
      }
//********************** MOV. MOTOR ***********************
if (PS4.getAnalogHat(LeftHatY)<90) {
        PS4.setRumbleOn(RumbleLow);
      }
  else{
        PS4.setRumbleOff();    
  }
      /*      
if (PS4.getAnalogHat(LeftHatX)) {
         J1_X = PS4.getAnalogHat(LeftHatX);
      }*/




  J1_Y = PS4.getAnalogHat(LeftHatY);
  J1_X = PS4.getAnalogHat(LeftHatX);
  
  J1_Y = constrain(J1_Y,0,255);
  J1_X = constrain(J1_X,0,255);
//*********************** MOV. SERVOS ********************************
  J2_Y = PS4.getAnalogHat(RightHatX);
  J2_X = PS4.getAnalogHat(RightHatY);

      //map(J2_Y,130,0,0,255);  
  //camara_Y = J2_Y;
  //camara_X = J2_X;
  if (PS4.getButtonClick(R1)) {
        //Serial.print(F("\r\nTouchpad"));
        printAngle = !printAngle;
        printServ = !printServ;
      }
  if (printAngle) { 
        
        camara_X = map(PS4.getAngle(Pitch),90,270,0,180);
        if(camara_X>180){
            camara_X = 180;
        }
        if(camara_X<0){
            camara_X = 0;
        }
        camara_Y = map(PS4.getAngle(Roll),270,90,0,180);
        if(camara_Y>180){
            camara_Y = 180;
        }
        if(camara_Y<0){
            camara_Y = 0;
        }
        

      }

  
  if( printServ ){ //SERVOS 
    if( J2_Y<100 ){cy-=0.1;
      if(cy <= 0){cy = 0;}
      camara_Y=(int)cy;      
      }
    if( J2_Y>160 ){cy+=0.1;
      if(cy >= 180){cy = 180;}
      camara_Y=(int)cy;
      }

    if( J2_X<100 ){cx-=0.1;
      if(cx <= 0){cx = 0;}
      camara_X=(int)cx; 
      }
    if( J2_X>160 ){cx+=0.1; 
      if(cx >= 180){cx = 180;}
      camara_X=(int)cx;  
      } 
  }
  if (PS4.getButtonClick(R3)){
    camara_Y = 90;
    camara_X = 90;
    cx = 90.0, 
    cy = 90.0;
    }
              if(PS4.getButtonClick(SQUARE)){
                                 
                    StaticJsonDocument<200> doc;
                    camara_X = 0;
                    camara_Y = 0;
                    //for(int i = 90;i>=0;i--)
                    for(float i = 0;i<=180;i+=0.05)
                    {
                    
                    camara_Y = i;
                    //delay(10);
                    doc["MI"] = J1_X;
                    doc["MD"] = J1_Y;
                    doc["S1"] = camara_Y;
                    doc["S2"] = camara_X; 
                    doc["LED"] = led_Z;  
                    
                    String output;
                    serializeJson(doc, output);
                    if (millis() >= currentTime2 + 50){
                    currentTime2 = millis();  
                    Serial.println(output);
                    Serial.println("");    
                    }   
                  }
                  camara_X = 180;
                  for(float i = 180;i>=0;i-=0.05)
                  {
                    camara_Y = i;
                    //delay(10);
                    doc["MI"] = J1_X;
                    doc["MD"] = J1_Y;
                    doc["S1"] = camara_Y;
                    doc["S2"] = camara_X; 
                    doc["LED"] = led_Z;  
                    
                    String output;
                    serializeJson(doc, output);
                    if (millis() >= currentTime2 + 50){
                    currentTime2 = millis();  
                    Serial.println(output);
                    Serial.println("");    
                    } 
                  }
                  camara_X = 90;
                  camara_Y = 90;
                  
             }
  
//******************** LED ***********************
  if (PS4.getButtonClick(TOUCHPAD)) {
        Serial.print(F("\r\nTouchpad"));
        printTouch = !printTouch;
      }

  J1_Z = PS4.getX(i);

  if( digitalRead(pin_J_MOTOR_PUSH)){ //INTENSIDAD DE LUZ
    led_Z = map(J1_Z, 50, 1820, 0, 255);
    if( led_Z < 5   ){ led_Z = 0; }
    if( led_Z >= 255   ){ led_Z = 255; }
    //led_Z = 50;
  }
//*******************************************

  doc["MI"] = J1_X;
  doc["MD"] = J1_Y;
  doc["S1"] = camara_Y; //J2_Y;
  doc["S2"] = camara_X; //J2_X;
  doc["LED"] = led_Z; //J2_X;  //pwm LUZ LED
  
    String output;
    serializeJson(doc, output);

  //*********************************************************

 //TASK2
  if (millis() >= currentTime2 + 50){
    currentTime2 = millis();
    //Serial.println("  task2");
  
    Serial.println(output);
    Serial.println("");    
  }



}//IF LOOP
