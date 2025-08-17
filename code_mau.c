//Library
#include "HX711.h"

// Encoder output to Arduino Interrupt pin
#define ENCODER_PIN_A 34  // Pin A of encoder 1 
#define ENCODER_PIN_B 35  // Pin B of encoder 1
#define ENCODER_PIN_C 36 //  Pin A of encoder 2
#define ENCODER_PIN_D 39  // Pin B of encoder 2 

// PWM connected to pin 10
#define PWMB 5
#define PWMA 23

// MD10C DIR connected to pin 12
#define BIN1 21
#define BIN2 19
#define AIN1 22
#define AIN2 18 

// Tracking sensor pins
#define ss1 32
#define ss2 33
#define ss3 25
#define ss4 26
#define ss5 27
#define ss6 14
#define ss7 12  

//Define the pins for the HX711 communication 
#define DATA_PIN 2 
#define CLOCK_PIN 4 

//Loadcell factor 
int weight_In_g;
float CALIBRATION_FACTOR = -437.27;
HX711 LOADCELL_HX711;


//Tracking sensor factor
int s1, s2, s3, s4, s5, s6, s7, d1, d2, d3, d4, d5, d6, d7, D, threshold = 1200;
float sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7;
float x, TB;

// One-second interval for measurements
int interval = 10;
 
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

//PID factor
long int pos1, pos2, step1, step2, lastpos1, lastpos2;
float e1,e2, eSum1,eSum2, ePre1, ePre2,e3Pre,e3;
float v1,v2;
float output1, output2;
float Kp, Ki, Kd;
double va1,va2;
double add;

//set point 
double setpoint1=50; 
double setpoint2=50;

// duty cycle 
double dutyCycle1, dutyCycle2;

//State of robot
int state=0;

//Flag
//bool enable = true;
///////////////////////////////////
void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Set encoder as input with internal pullup  
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_PIN_C, INPUT_PULLUP);
  pinMode(ENCODER_PIN_D, INPUT_PULLUP);
  
  // Set PWM and DIR connections as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Interrupt
  attachInterrupt (digitalPinToInterrupt(ENCODER_PIN_A), encoder1_sub_pin, RISING);
  attachInterrupt (digitalPinToInterrupt(ENCODER_PIN_C), encoder2_sub_pin, RISING);
  
  // Set input pin for tracking sensors
  pinMode(ss1,INPUT);
  pinMode(ss2,INPUT);
  pinMode(ss3,INPUT);
  pinMode(ss4,INPUT);
  pinMode(ss5,INPUT);
  pinMode(ss6,INPUT);
  pinMode(ss7,INPUT);

  //Set up HX711
  LOADCELL_HX711.begin(DATA_PIN,CLOCK_PIN);

  // Setup initial values for timer
  previousMillis = millis();
}

//Function Set up Interrupt to read encoder
void encoder1_sub_pin(){ //Encoder motor 1
  if(digitalRead(ENCODER_PIN_B)==0){
    pos1++;
  }
  else{
    pos1--;
  }
}
void encoder2_sub_pin(){ //Encoder motor 2
  if(digitalRead(ENCODER_PIN_D)==0){
    pos2--;
  }
  else{
    pos2++;
  }
}

//Function stop
void stop(){
  setpoint1=0;
  setpoint2=0;
  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, LOW);
  // digitalWrite(BIN1, LOW);
  // digitalWrite(BIN2, LOW);
}

//Function tracking
void Tracking(){
//Read analog from sensor 
  //Sensor 1
  s1 = analogRead(ss1); 
  if(s1 > threshold) { d1 = 1; }
  else { d1 = 0; }
  //Sensor 2
  s2 = analogRead(ss2);
  if(s2 > threshold) { d2 = 1; }
  else { d2 = 0; }
  //Sensor 3
  s3 = analogRead(ss3);
  if(s3 > threshold) { d3 = 1; }
  else { d3 = 0; }
  //Sensor 4
  s4 = analogRead(ss4);
  if(s4 > threshold) { d4 = 1; }
  else { d4 = 0; }
  //sensor 5
  s5 = analogRead(ss5); 
  if(s5 > threshold) { d5 = 1; }
  else { d5 = 0; }
  //sensor 6
  s6 = analogRead(ss6); 
  if(s6 > threshold) { d6 = 1; }
  else { d6 = 0; }
  //sensor7
  s7 = analogRead(ss7);
  if(s7 > threshold) { d7 = 1; }
  else { d7 = 0; }

  //Calculate how many sensor read black
  D = d1 + d2 + d3 + d4 + d5 + d6 + d7;

  //Calib sensor 
  sensorValue1 = 123.4286 + 1.0109 * (s1 - 112);
  sensorValue2 = 123.4286 + 0.9597 * (s2 - 115);
  sensorValue3 = 123.4286 + 1.0116 * (s3 - 115);
  sensorValue4 = 123.4286 + 1.0659 * (s4 - 97);
  sensorValue5 = 123.4286 + 1.0399 * (s5 - 100);
  sensorValue6 = 123.4286 + 0.9791 * (s6 - 160);
  sensorValue7 = 123.4286 + 0.9443 * (s7 - 165);

  //feedback factor 
  TB = (3 * (sensorValue1 - sensorValue7) + 2 * (sensorValue2 - sensorValue6) + (sensorValue3 - sensorValue5)) * 17 / 
       (sensorValue1 + sensorValue2 + sensorValue3 + sensorValue4 + sensorValue5 + sensorValue6 + sensorValue7);
       
  Serial.println(TB);
  if(D==3){
    x=1000;
  }
  else if(D==0){
    x=999;
    stop();
  }
  else{
    x=TB*0.9822-1.6627;
  }
  delay(10);
}

//Function of loadcell
void loadcell(){
  LOADCELL_HX711.set_scale(CALIBRATION_FACTOR);
  for(int i=0;i<4;i++){
    weight_In_g = LOADCELL_HX711.get_units(10)-102;
    delay(200); 
  }
  if(weight_In_g <300){
    stop();
  }
  else if ( weight_In_g > 1300){
    state=2;
  }
  else if(weight_In_g >300 && weight_In_g < 1300){
    state=1;
  }
}

// Function PID motor 1 
void VControl1(){
   Kp = 1, Ki = 0.66, Kd = 0;
   v1=(step1*1500)/330;    
   e1 = setpoint1 + v1;
    output1 = Kp*e1 + Ki*(e1+ePre1) + Kd*(e1-ePre1);
    ePre1 = e1; //save last error  
    if      (output1<=0){ output1 = 0;}
    else if(output1>=333){ output1 = 333;}
    else { output1 = output1;}
    va1=(output1/333)*255;
    dutyCycle1=va1;
    analogWrite(PWMA, dutyCycle1);  
  }

// Function PID motor 2
void VControl2(){
    Kp = 1.05, Ki = 0.65, Kd = 0;
    v2=(step2*1500)/330;    
    e2 = setpoint2 + v2;
    output2 = Kp*e2 + Ki*(e2+ePre2) + Kd*(e2-ePre2);
    ePre2 = e2; //save last error
    if      (output2<=0){ output2 = 0;}
    else if(output2>=333){ output2 = 333;}
    else { output2 = output2;} 
    va2=(output2/333)*255;
    dutyCycle2=va2;
    analogWrite(PWMB, dutyCycle2);  
  }

//Function control PID for tracking sensor
void AControl(){
  Kp = 0.8, Ki = 0, Kd = 0.6;
  if(state==0){
    e3=atan(x/255);
  }
  else if(state==1)
  {
    e3=atan((x-12)/255);
  }
  else if(state==2){
    e3=atan((x+12)/255);
  }
  add = (Kp*e3 + Ki*(e3+e3Pre)+Kd*(e3-e3Pre))*350;  //Kp*e2 + Ki*(e2+ePre2) + Kd*(e2-ePre2)
  e3Pre = e3;
  //add = e3*734.56; 
  setpoint1=50-add;
  setpoint2=50+add;
  if(x==999){
      setpoint1=0;
      setpoint2=0;
  }
  else if (x==1000){
    setpoint1=0;
    setpoint2=0;
  }
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > 10) {
    step1=pos1-lastpos1;
    step2=pos2-lastpos2;
    lastpos1=pos1;
    lastpos2=pos2;
    previousMillis=currentMillis;
  }
  VControl1();
  VControl2();
  Tracking();
  // Serial.println(x);
  // if (x==1000){
  // while(state==0){
  //   stop();
  //   loadcell();
  // }
  // }
  // else if (x==999){
  //   stop();
  // }
  AControl();
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

}

