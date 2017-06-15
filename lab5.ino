#include <phys253.h>          
#include <LiquidCrystal.h>

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

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

unsigned int get_error() {
  return 0;
}

/**
 * Change car's direction. Positive heading = turn right.
 * Negative heading = turn left. Heading of 0 = go straight.
 */
void updateHeading(int heading) {
  // requirements: -1 < heading < 1  
  int rightSpeed = motorSpeed * (1 + heading);
  int leftSpeed = motorSpeed * (1 - heading);
  
  motor.speed(RIGHT_MOTOR, (rightSpeed > 255) ? 255 : rightSpeed);
  motor.speed(LEFT_MOTOR, (leftSpeed < 255 ) ? 255 : leftSpeed);
}

void setup()
{  
  #include <phys253setup.txt>

  // initial values
  heading = 0;
  motorSpeed = 60;
  
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
  updateHeading(heading);

  delay(10); // make dt more predictable

  // currently our wheels can't go backwards. Is this a problem?
}

