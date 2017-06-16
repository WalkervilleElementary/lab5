#include <phys253.h>          
#include <LiquidCrystal.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_SENSOR 2
#define RIGHT_SENSOR 0
#define TAPE_THRESHOLD 100

float gain;
float kp;
float ki;
float kd;

int error;
int lastError;
unsigned long lastTime;

float heading;
int motorSpeed;

unsigned int PWM_MAP[256];

int get_error() {
  boolean rightTape = (analogRead(RIGHT_SENSOR) > TAPE_THRESHOLD);
  boolean leftTape = (analogRead(LEFT_SENSOR) > TAPE_THRESHOLD);

  int newError;

  if (rightTape && leftTape)
    newError = 0;
  else if (rightTape && !leftTape)
    newError = -1;
  else if (leftTape && !rightTape)
    newError = 1;
  else if (!(leftTape || rightTape)) {
    if (lastError < 0)
      newError = -5;
    else if (lastError > 0)
      newError = 5;
  }
  return newError;
}

/**
 * Change car's direction. Positive heading = turn left.
 * Negative heading = turn right. Heading of 0 = go straight.
 * Requires: -1 <= heading <= -1
 */
void updateHeading() {
  int rightSpeed = (int) (motorSpeed * (1.0 - heading));
  int leftSpeed = (int) (motorSpeed * (1.0 + heading));

  rightSpeed = (rightSpeed > 255) ? 255 : PWM_MAP[rightSpeed];
  leftSpeed = (leftSpeed > 255 ) ? 255 : PWM_MAP[leftSpeed];
  
  motor.speed(RIGHT_MOTOR, rightSpeed);
  motor.speed(LEFT_MOTOR, leftSpeed);
}

void setup()
{  
  #include <phys253setup.txt>

  // populate PWM table
  for (int i = 0; i < 256; i++) {
    PWM_MAP[i] = (int) (50 + i * 0.804);
  }

  // initial values
  heading = 0;
  motorSpeed = 150;
  
  ki = 0.0;
  
  lastError = 0;
  lastTime = millis();
  
  Serial.begin(9600);
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("SET Kp:");
    LCD.setCursor(0,1);
    LCD.print(analogRead(6));
    delay(50);
  }
  kp = analogRead(6) * 0.001;

  delay(1000);

  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("SET Kd:");
    LCD.setCursor(0,1);
    LCD.print(analogRead(6));
    delay(50);
  }
  kd = analogRead(6) * 0.001;

  delay(1000);
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("SET GAIN:");
    LCD.setCursor(0,1);
    LCD.print(analogRead(6));
    delay(50);
  }
  gain = analogRead(6) * 0.001;
}

void loop()
{
  // push stopbutton to change speed
  if (stopbutton()) {
    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("SET SPEED:");
      LCD.setCursor(0,1);
      LCD.print(analogRead(7));
      delay(50);
    }
    
    motorSpeed = analogRead(7) * (255.0 / 1023);
    Serial.println(motorSpeed);
    delay(1000);
  }
  
  // motor frequency is 50 hz
  // don't do this more than every 100 milliseconds
  unsigned long now = millis();
  if (now >= lastTime + 100) {
    float dt = (now - lastTime) * 0.001;
  
    error = get_error();

    heading = gain * (kp * error + kd * (error - lastError) / dt);
    if (heading > 1)
      heading = 1;
    else if (heading < -1)
      heading = -1;
    updateHeading();
  
    // update errors
    lastError = error;
    lastTime = now;

    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("Heading:");
    LCD.setCursor(0,1);
    LCD.print(heading);
  }
}

