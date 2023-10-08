#include <Servo.h>

Servo ground_servo;

int servo_pin = 9;
int relay_pin = 10;

int servo_raised = 180;
int servo_lowered = 10;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  pinMode(relay_pin, OUTPUT);
  ground_servo.attach(servo_pin); // pin 9 PWM capable
  ground_servo.write(servo_raised);
}

// the loop function runs over and over again forever
void loop() {
  if (Serial.available() > 0)
  {
    int cmd = Serial.parseInt();

    if(cmd == 1)
    {
      delay(1000);
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(relay_pin, HIGH);
      ground_servo.write(servo_lowered);
      delay(1000);              // wait for a second
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(relay_pin, LOW);
      ground_servo.write(servo_raised);
      delay(250);
      Serial.println("ok");     // movement completed, reply ok. 
    }
  }
}