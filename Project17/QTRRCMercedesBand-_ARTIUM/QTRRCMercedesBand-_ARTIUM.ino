#include <QTRSensors.h>
#include <Arduino_APDS9960.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//PID
int lastError = 0;

const float M1 = 255;  //RIGHT WHEEL
const float M2 = 235;  //LEFT WHEEL  +-5
float M2toM1Ratio = M2/M1;
float currentTime=0;
float elapsedTime=0;
float previousTime=0;
float cumError=0;
float rateError=0;
float KP=6.0/310;
float KI=1.0/1000000000;
float KD=1;

//motor control
int motorPinIn1 = 6;
int motorPinIn2 = 5;
int motorPinENA = 9;
int motorPinIn3 = 3;
int motorPinIn4 = 4;
int motorPinENB = 10;


void qtrSetup(){
  // configure the sensors
  Serial.println("configuring sensors");
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){13, 12, 11, 8, A0, A1, A2, A3}, SensorCount);
  qtr.setEmitterPin(7);
}

void motorSetup(){
  //motor setup
  Serial.println("configuring motors");
  pinMode(motorPinIn1, OUTPUT); //in1
  pinMode(motorPinIn2, OUTPUT); //in2
  pinMode(motorPinENA, OUTPUT); //pwm
  pinMode(motorPinENB, OUTPUT); //pwm
  pinMode(motorPinIn3, OUTPUT); //in3
  pinMode(motorPinIn4, OUTPUT); //in4
  digitalWrite(motorPinIn1, HIGH);
  digitalWrite(motorPinIn2, LOW);
  digitalWrite(motorPinIn3, HIGH);
  digitalWrite(motorPinIn4, LOW);
  
}

void calibrateQtr(){
//QTR8 sensor calibration
  Serial.println("dalaying 0.5s");
  delay(500);
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  Serial.println("calibrating");
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  //digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.print("results - min: ");
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("results - max: ");
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void setup()
{
//////////////////////////////////////////////////////////////
while (!Serial); // Wait for Serial Monitor to open
if (!APDS.begin()) {
Serial.println("Error initializing APDS-9960 sensor.");
while (true); // Stop forever
}

int proximity = 0;
int r = 0, g = 0, b = 0;
unsigned long lastUpdate = 0;

///////////////////////////////////////////////////////////////
  Serial.begin(9600);
  motorSetup();
  qtrSetup();
  calibrateQtr();
  
  Serial.println("dalying 1s");
  //delay(1000);
}

void loop()
{
  digitalWrite(motorPinIn1, HIGH);
  digitalWrite(motorPinIn2, LOW);
  digitalWrite(motorPinIn3, HIGH);
  digitalWrite(motorPinIn4, LOW);
  
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  
    
  //time
   currentTime = millis();
   elapsedTime = currentTime - previousTime;
   previousTime = currentTime;
  

  //PID
  int error = position - 3500;

  cumError += error * elapsedTime;
  rateError = (error - lastError)/elapsedTime;
  float scaledError = rateError * rateError * rateError;
  scaledError = clamp(scaledError, -200, 200);
  float motorSpeed = KP * error + KI * cumError + KD * scaledError;
  lastError = error;

  //motors
  float m1Speed = M1;
  float m2Speed = M2;
  if(motorSpeed > 0){
    m1Speed = M1 - motorSpeed;
  } else {
    m2Speed = M2 + motorSpeed;
  }
  
  m1Speed = clamp(m1Speed, 0, 255);
  m2Speed = clamp(m2Speed, 0, 255);

  analogWrite(motorPinENA, m1Speed);
  analogWrite(motorPinENB, m2Speed);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position);
  Serial.print("\t speed=");
  Serial.print(motorSpeed);
  Serial.print("\t m1=");
  Serial.print(m1Speed);
  Serial.print("\t m2=");
  Serial.print(m2Speed);
  Serial.print("\t Kp*p=");
  Serial.print(KP * error);
  Serial.print("\t Ki*I=");
  Serial.print(KI * cumError);
  Serial.print("\t Kd*D=");
  Serial.println(KD * scaledError);
}

float clamp(float value, float min, float max){
  return value > max ? max : (value < min ? min : value);  
}

int sign(float value) {
  return value < 0 ? -1 : 1;
}
