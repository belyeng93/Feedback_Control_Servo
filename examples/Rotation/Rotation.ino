#include "FeedbackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 12 //D6
#define SERVO_PIN 14 //D5

// set feedback signal pin number
FeedbackServo servo = FeedbackServo(FEEDBACK_PIN);

void setup() {
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);
}

void loop() {
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
    servo.rotate(270, 4);
    delay(1000);
    servo.rotate(-180, 4);
    delay(1000);
}
