#include <Servo.h>

Servo ground_servo;

int servo_pin = 9;
int relay_pin = 10;

int servo_raised = 180;
int servo_lowered = 10;

boolean charge_state = False;  // False is off, True is on

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
      String command = Serial.readStringUntil('\n');  // Read the string until a newline character is encountered

      if (command == "ground")
      {
        delay(1000);
        digitalWrite(13, HIGH);   // Turn the LED on (HIGH is the voltage level)
        digitalWrite(relay_pin, HIGH);
        ground_servo.write(servo_lowered);
        delay(1000);              // Wait for a second
        digitalWrite(13, LOW);    // Turn the LED off by making the voltage LOW
        digitalWrite(relay_pin, LOW);
        ground_servo.write(servo_raised);
        delay(250);
        Serial.println("ACK");     // Movement completed, reply with "ok"
      }
      else if(command == "charge")
      {
        charge = !charge;
        digitalWrite(14, charge);
        Serial.println("ACK");
      }
      else
      {
        Serial.println("NAK")
      }
    }
}