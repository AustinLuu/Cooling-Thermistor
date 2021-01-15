#include <PID_v1.h>
int ThermistorPin = 0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int dir1 = 3;
int dir2 = 4;
int spin = 5;
volatile int mspeed;
double Setpoint, distance, Output, Input;
double Kp=10, Ki=2, Kd=0.2;
PID myPID(&Input, &Output,&Setpoint,Kp,Ki,Kd,REVERSE);

  
void setup() {
myPID.SetMode(AUTOMATIC); 
myPID.SetTunings(Kp,Ki,Kd);
myPID.SetOutputLimits(110,255);

for (int i = 0; i<10; i++){
Vo = analogRead(ThermistorPin);
R2 = R1 * (1023.0 / (float)Vo - 1.0);
logR2 = log(R2);
T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
T = T - 273.15;
Setpoint = Setpoint + T;
}
Setpoint = (Setpoint/10 + 20)/2;

Serial.begin(9600);
pinMode(255,OUTPUT);
pinMode(dir1,OUTPUT);
pinMode(dir2,OUTPUT);
delay(500);
}

void loop() {

  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  Input = T; 
  myPID.Compute();
  mspeed = round(Output);
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  analogWrite(spin,mspeed);

  Serial.print(T);
  Serial.print(" ");
  Serial.println(Setpoint);
  delay(500);
}
