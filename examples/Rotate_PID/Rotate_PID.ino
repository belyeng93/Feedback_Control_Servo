#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 12 //D6
#define SERVO_PIN 14 //D5

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN, 1.0, 0.0, 0.0, 0.0, -200, 200, 20.0);
//FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);


void setup() {
    // set servo control pin number
    Serial.begin(115200);
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);
}

void loop() {
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
    servo.rotate_PID(270, 1);
    delay(1000);
    servo.rotate_PID(-180, 1);
    delay(1000);

    //Serial.println(servo.out_glob);
}
