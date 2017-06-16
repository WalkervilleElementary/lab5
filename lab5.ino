#include <phys253.h>          
#include <LiquidCrystal.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

float gain;
float kp;
float ki;
float kd;

int error;
int lastError;
int lastLastError;
unsigned long lastTime;

float heading;
int motorSpeed;

unsigned int PWM_MAP[256];

unsigned int get_error() {
  return 0;
}

/**
 * Change car's direction. Positive heading = turn left.
 * Negative heading = turn right. Heading of 0 = go straight.
 * Requires: -1 <= heading <= -1
 */
void updateHeading() {
  int rightSpeed = (int) (motorSpeed * (1 + heading));
  int leftSpeed = (int) (motorSpeed * (1 - heading));
  
  motor.speed(RIGHT_MOTOR, (rightSpeed > 255) ? 255 : PWM_MAP[rightSpeed]);
  motor.speed(LEFT_MOTOR, (leftSpeed > 255 ) ? 255 : PWM_MAP[leftSpeed]);
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
  motorSpeed = 100;
  
  gain = 0.01;
  kp = 1.0;
  ki = 0.0;
  kd = 0.0;

  lastError = 0;
  lastLastError = 0;

  lastTime = millis();
  
  // Serial.begin(9600);
}

void loop()
{
  unsigned long now = millis();
  // don't do this more than every 10 milliseconds
  // keep dt more consistent
  if (now >= lastTime + 10) {
    float dt = (now - lastTime) * 0.001;
  
    error = get_error();
  
    float dO = gain * ((error - lastError) + ki * dt * error + 
              kd * (error - 2 * lastError + lastLastError) / dt);
  
    // update errors
    lastLastError = lastError;
    lastError = error;
    lastTime = now;
  
    // calculate heading, make sure constrained to [-1, 1]
    heading += dO;
    if (heading > 1)
      heading = 1;
    else if (heading < -1)
      heading = -1;
    updateHeading();
  }
}

