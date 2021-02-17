#include "FeedbackServo.h"
// define feedback signal pin
#define FEEDBACK_PIN 12

// set feedback signal pin number
FeedbackServo servo = FeedbackServo(FEEDBACK_PIN);

void setup() {
    // serial communication start with 115200 baud
    Serial.begin(115200);
}

void loop() {
    Serial.print("Now angle: ");
    Serial.println(servo.Angle());
}
