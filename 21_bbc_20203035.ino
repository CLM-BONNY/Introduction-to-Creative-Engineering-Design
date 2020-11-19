#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

// initialize serial port
  Serial.begin(57600);

// initialize servoMotor
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1450);
  delay(1000);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  if (103 < raw_dist && raw_dist < 209) {
    myservo.writeMicroseconds(1650);
    }
  else if (209 < raw_dist && raw_dist < 260){
    myservo.writeMicroseconds(1250);
    }
}
