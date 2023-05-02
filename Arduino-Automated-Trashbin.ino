//  Libraries ------------------------------------
#include "NewPing.h"
#include <Servo.h>


// Cover Ultrasonic Sensors
#define CoverSensorTrigger 12
#define CoverSensorEcho 13
#define CoverSensorMax 300

// Bin 1  Ultrasonic Sensors
#define Bin1Trigger 10
#define Bin1Echo 11
#define Bin1Max 50

// Bin 2  Ultrasonic Sensors
#define Bin2Trigger 8
#define Bin2Echo 9
#define Bin2Max 50

// Servos
#define CoverServoPin 2
#define SegregateServoPin 7

#define CapacitivePin 4
#define InductivePin 3

#define BuzzerPin 5

//
Servo CoverServo;
Servo SegregateServo;

int pos = 160;

NewPing CoverSensor(CoverSensorTrigger, CoverSensorEcho, CoverSensorMax);
NewPing Bin1Sensor(Bin1Trigger, Bin1Echo, Bin1Max);
NewPing Bin2Sensor(Bin2Trigger, Bin2Echo, Bin2Max);

void setup() {
  Serial.begin(9600);

  CoverServo.attach(CoverServoPin);
  // CoverServo.write(160);

  SegregateServo.attach(SegregateServoPin);
  SegregateServo.write(90);

  pinMode(BuzzerPin, OUTPUT);
  pinMode(InductivePin, INPUT);
  pinMode(CapacitivePin, INPUT);


  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  beep();
}

void loop() {

  Serial.print("Bin1Sensor:");
  Serial.print(Bin1Sensor.ping_cm());
  Serial.print(",");
  Serial.print("Bin2Sensor:");
  Serial.print(Bin2Sensor.ping_cm());
  Serial.print(",");
  Serial.print("CoverSensor:");
  Serial.print(CoverSensor.ping_cm());
  Serial.print(",");
  Serial.print("InductivePin:");
  Serial.print(digitalRead(InductivePin));
  Serial.print(",");
  Serial.print("CapacitivePin:");
  Serial.println(digitalRead(CapacitivePin));

  // if (CoverSensor.ping_cm() < 50 && checkBins()) {

  if (CoverSensor.ping_cm() <= 40 && checkBins()) {
    // Serial.println("1");
    CoverServo.write(20);
    delay(3000);
    // Serial.println("2");
    for (int pos = 20; pos <= 160; pos += 1) {
      CoverServo.write(pos);
      delay(15);
    }
  } else {
    // Serial.println("3");
    CoverServo.write(160);
  }



  // while (digitalRead(InductivePin) == HIGH && digitalRead(CapacitivePin) == HIGH) return;

  if (digitalRead(InductivePin) == LOW && digitalRead(CapacitivePin) == LOW) {

    Serial.println("Metal Object");

    metal();

  } else if (digitalRead(CapacitivePin) == LOW && digitalRead(InductivePin) == HIGH) {

    Serial.println("Plastic Object");

    plastic();
  }
}


bool checkBins() {

  Serial.println("Checking bins");

  if (Bin1Sensor.ping_cm() < 15) {
    Serial.println("Bin 1 Full");
    digitalWrite(18, HIGH);  // turn the LED on (HIGH is the voltage level)
    beep();
    digitalWrite(18, LOW);
    return false;
  } else if (Bin2Sensor.ping_cm() < 15) {
    Serial.println("Bin 2 Full");
    digitalWrite(19, HIGH);
    beep();
    digitalWrite(19, LOW);
    return false;
  } else {
    digitalWrite(18, LOW);
    digitalWrite(19, LOW);
  }

  Serial.println("bins not yet full");
  return true;
}

void beep() {
  digitalWrite(BuzzerPin, HIGH);
  delay(500);
  digitalWrite(BuzzerPin, LOW);
}

void metal() {
  SegregateServo.write(180);
  digitalWrite(18, HIGH);
  delay(500);
  SegregateServo.write(90);
  digitalWrite(18, LOW);
}

void plastic() {
  SegregateServo.write(0);
  digitalWrite(19, HIGH);
  delay(500);
  SegregateServo.write(90);
  digitalWrite(19, LOW);
}