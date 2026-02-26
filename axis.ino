#include<Adafruit_PWMServoDriver.h>
#include<Wire.h>
#include<math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 140
#define SERVOMAX 520

float z;
const int y = 0;
float b;
float x;

int xin = A0;
int yin = A1;

float currentpos;

int xval;
int yval;

 
int L1 = 12;
int L2 = 9;

void setangle(int channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = SERVOMIN + ((float)angle / 180.0) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(channel,0,pulse);
}

void smoothservo(int channel, int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int i = startAngle; i <= endAngle; i++) {
      setangle(channel, i);
      delay(8);
    }
  } else {
    for (int i = startAngle; i >= endAngle; i--) {
      setangle(channel, i);
      delay(8);
    }
  }
}



void setup() {
  // put your setup code here, to run once:
 pwm.begin();
 Serial.begin(9600);
 pwm.setPWMFreq(50);
 delay(10);
 pinMode(xin, INPUT);
 pinMode(yin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 int xread = analogRead(xin);
 int yread = analogRead(yin);
 
 if(xread >= 680) {
    x = map(xread,680,1023,1,15);
 }
 else if(xread <= 580) {
   x = map(xread,0,580,-10,-1);
 }
 if(yread <= 580) {
   z = map(yread,0,580,1,20);
 }
 else if(yread >= 680) {
   z = map(yread,680,1023,-5,-1);
 }
 else if(xread < 680 && yread > 580 && yread < 680) {
   x == 1 && z == 1;
 }
 float D = sqrt(x*x + y*y);
 float l = sqrt(z*z + D*D);
 float s1 = atan2(z,D) * 180 / PI;
 float s2 = acos((sq(L1) + sq(l) - sq(L2)) / (2 * L1 * l)) * 180 / PI;
 float e = acos((sq(L1) + sq(L2) - sq(l)) / (2 * L1 * L2)) * 180 / PI; //elbow angle
 float s = s1 + s2; //sholder angle
 float bf = b + 90;
 float ef = 180 - e;
 float sf = s + 45;
 float bf1 = constrain(bf, 0 , 180);
 float ef1 = constrain(ef, 0, 150);
 float sf1 = constrain(sf, 40, 180);
 
 setangle(15, bf1);
 setangle(14, sf1);
 setangle(13, ef1);



 Serial.print("b: ");
 Serial.print(bf1);
 
 Serial.print(" || s: ");
 Serial.print(sf1);

 Serial.print(" || e: ");
 Serial.print(ef1);

 Serial.print(" || x: ");
 Serial.print(x);

 Serial.print(" || z: ");
 Serial.print(z);

 Serial.print(" || xval: ");
 Serial.print(xread);

 Serial.print(" || yval: ");
 Serial.println(yread);

 delay(200);
}
