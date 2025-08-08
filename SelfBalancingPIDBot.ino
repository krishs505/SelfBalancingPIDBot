#include <Servo.h>

Servo servo;

const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 5;

const int servoCenter = 127; // stable servo value
const int servoMin = 90; // max right tilt
const int servoMax = 150; // max left tilt
int servoOutput = 0;

// PID control variables
double setpoint = 12.0; // desired position in cm
double input = 0.0;
double output = 0.0;
double output_test = 0.0;
double error = 0.0;
double lastError = 0.0;
double integral = 0.0;
double integral_test = 0.0;
double derivative = 0.0;

// PID coefficients
double Kp = 2.7; // 1.0 - 5.0
double Ki = 0.09; // 0.01 - 0.5
double Kd = 1.6; // 0.1 - 2.0

unsigned long lastTime;
double elapsedTime;

double lastReading = setpoint;

double readUltrasonic() {
  // reset trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // send an ultrasound by setting trigPin to HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // read the travel time of the sound wave
  long duration = pulseIn(echoPin, HIGH);

  // calculate distance based on speed of sound (0.034 cm/microsec)
  double distance = duration * 0.034 / 2.0;
  return distance;
}

double getMedianDistance() {
  const int numReadings = 13;
  double readings[numReadings];

  for (int i = 0; i < numReadings; i++) {
    readings[i] = readUltrasonic();
    delay(10);
  }

  // sort readings
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = i + 1; j < numReadings; j++) {
      if (readings[j] < readings[i]) {
        double temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  double reading = readings[numReadings / 2];

  // filter out absurd values (difference of 30+ from last OR above 35)
  if (reading > 35 || (abs(reading - lastReading) > 30)) {
    Serial.print("ABSURD value: ");
    Serial.println(reading);
    return lastReading;
  } else {
    lastReading = reading;
    return reading;
  }
}

void setup() {
	Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

  servo.attach(servoPin);
  servo.write(servoCenter);

  lastTime = millis();
}
void loop() {

  // take a sample of data from ultrasonic and get median to eliminate extreme values
  double rawInput = getMedianDistance();

  error = setpoint - rawInput;
  unsigned long now = millis();
  elapsedTime = (now - lastTime) / 1000.0;

  // velocity calculation for D
  derivative = Kd * ((error - lastError) / elapsedTime);

  // if near setpoint and slow velocity, center servo, otherwise PID
  if (abs(derivative) < 5 && abs(error) < 1.5) {
    servoOutput = servoCenter;
  } else {
    // test output with current integral
    integral_test = integral + Ki * error * elapsedTime;
    output_test = servoCenter + Kp * error + integral_test + derivative;

    // if not saturated (within servo limits), add integral, otherwise keep last (to prevent integral windup)
    if (output_test > servoMin && output_test < servoMax) {
      integral = integral_test;
    }

    // calculate PID
    output = servoCenter + Kp * error + integral + derivative;

    // constrain within servo limits
    servoOutput = constrain(output, servoMin, servoMax);
  }

  servo.write(servoOutput);

  lastError = error;
  lastTime = now;
  
  Serial.print("Position: ");
  Serial.print(rawInput);
  Serial.print(" cm | Servo: ");
  Serial.print(servoOutput);
  Serial.print(" | output: ");
  Serial.print(output);
  Serial.print(" | Velocity: ");
  Serial.print(derivative);
  Serial.println(" cm /s");
  
}