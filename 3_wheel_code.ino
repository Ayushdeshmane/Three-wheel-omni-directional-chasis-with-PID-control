#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//motor pins
const int pin[6] = { 2, 1, 4, 3, 6, 5 };
#define f1 pin[0] 
#define f1d pin[1]
#define b1 pin[3]
#define b1d pin[2]
#define b2 pin[4]
#define b2d pin[5]

//declarations
int pwm_limit = 55;
int ac = 0;
int VX = 0;
int VY = 0;
byte button[5] = { 0 };
float v1 = 0;
float v2 = 0;
float v3 = 0;
int wp = 0;
float error = 0;
int w = 0;
float Wchange = 0;
float wpre = 0;
float wx = 0;
int p_x = 0;
float target_angle = 0;
int p_angle = 0;
float c_time;
float p_time = 0;
float integral = 0;
float p_time2 = 0;
int rotation = 0;
int preangle = 0;
int currentangle = 0;


//no change
float rotErrorRange() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float angle = euler.x();
  if (angle != 0) {
    angle = 360 - angle;
  }
  float rotError = angle;
  return rotError;
}

//motor logic
//no change
void controlMotor(float vf, float vbr, float vbl) {
    int maxnum = findGreatest(abs(vf), abs(vbr), abs(vbl));

  if (maxnum > pwm_limit) {
    
    vf = map(abs(vf), 0, maxnum, 0, pwm_limit) * (vf / abs(vf));  //vFL2
    vbr = map(abs(vbr), 0, maxnum, 0, pwm_limit) * (vbr / abs(vbr));  //vBR2
    vbl = map(abs(vbl), 0, maxnum, 0, pwm_limit) * (vbl / abs(vbl));  //vBL2
  }
  if (vf >= 0) {
    analogWrite(f1, abs(vf));
    analogWrite(f1d, 0);
  } else if (vf < 0) {
    analogWrite(f1, 0);
    analogWrite(f1d, abs(vf));
  }
  if (vbr >= 0) {
    analogWrite(b1, abs(vbr));
    analogWrite(b1d, 0);
  } else if (vbr < 0) {
    analogWrite(b1, 0);
    analogWrite(b1d, abs(vbr));
  }
  if (vbl >= 0) {
    analogWrite(b2, abs(vbl));
    analogWrite(b2d, 0);
  } else if (vbl < 0) {
    analogWrite(b2, 0);
    analogWrite(b2d, abs(vbl));
  }
}

//bno object
//no change
int findGreatest(int vFL2, int vBR2, int vBL2) {
  int greatestNumber = vFL2;  // Assume vFL2 is initially the greatest

  // Compare vBR2 with greatestNumber
  if (vBR2 > greatestNumber) {
    greatestNumber = vBR2;
  }

  // Compare vBL2 with greatestNumber
  if (vBL2 > greatestNumber) {
    greatestNumber = vBL2;
  }

  // Return the greatest number
  return greatestNumber;
}

//equations
//no change
void equation(float vx, float vy, float W) {
  float Vxb = vx, Vzb = vy;
  float bnoAngle = rotErrorRange();

  vx = Vxb * cos(bnoAngle * M_PI / 180.0) + Vzb * sin(bnoAngle * M_PI / 180.0);
  vy = -Vxb * sin((bnoAngle * M_PI / 180.0)) + Vzb * cos(bnoAngle * M_PI / 180.0);

  v1 = vx + W * 0.288;
  v2 = -0.5 * vx + 0.57735 * vy + W * 0.288;  // 1.732 = 0.57735
  v3 = -0.5 * vx - 0.57735 * vy + W * 0.288;  // 1.732 = 0.57735

}

//bno logic
//no change
void bno_data() {
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  error = euler.x();    // 0 to 360
  currentangle =  euler.x(); 

  if (abs(preangle-currentangle) > 258){
    rotation +=  abs(preangle-currentangle)/(preangle-currentangle);
  }
  error = -( error + (360*rotation)); // rotation = 360*rotation , 360*rotation    if preangle - curentangle > 258 = rotation +=  abs(preangle - curentangle)/ preangle - curentangle
  preangle = currentangle;
}

//pid logic

float pid(float x) {
  c_time = millis();
  integral = integral + x * (c_time - p_time);
  if(abs(x)<2){
    integral = 0;
  }
  wp = 5 * x + 0.0003 * integral + 35 * (p_x - x) / (c_time - p_time);
  p_time = c_time;
  p_x = x;
  return wp;
}

//setup
//no change
void setup() {
  Serial.begin(115200);
  bno.begin();
  Serial5.begin(9600);
  bno.setExtCrystalUse(true);
  Serial.println("Ganapati Bappa Moraya");
  Wire2.begin();
  for (int i = 0; i < 6; i++) {
    pinMode(pin[i], OUTPUT);
  }
  delay(2000);
}

//loop
void loop() {

  bno_data();
  // Request and read data from the slave device
  Wire2.requestFrom(8, 5);  // Request 5 bytes from device address 8
  if (!Wire2.available()) {
    ac++;
  }
  if (ac >= 25) {
    VX = 0, VY = 0, w = 0;
  }
  if (Wire2.available() >= 5) {  // Wait for all bytes to be available
    ac = 0;
    Wire2.readBytes(button, 5);
    w = button[2];
    VX = button[3];
    VY = button[4];
    w = map(w, 0, 255, -127, 127);
    VX = map(VX, 0, 255, -128, 127);
    VY = map(VY, 0, 255, -128, 127);
    if (abs(VX) < 10){
      VX = 0;
    }
    if (abs(VY) < 10){
      VY = 0;
    }
    Wchange = abs(w - wpre);
    if (Wchange > 125) {
      w = 0;
    }
    wpre = map(button[2], 0, 255, -127, 127);
  }
  if (w != 0) {

    wx = -w;
    target_angle = error;
  }
  else {
    wx = pid(error - target_angle);
  }
  equation(VX, VY, wx);
  controlMotor(v1, v2, v3);
  Serial5.print(String(error - target_angle) + 'z');
}
