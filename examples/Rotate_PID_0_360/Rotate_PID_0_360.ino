#include "FeedbackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 12 //D6
#define SERVO_PIN 14 //D5

// set feedback signal pin number
FeedbackServo servo = FeedbackServo(FEEDBACK_PIN, 1.0, 0.2, 0.0, 0.0, -200, 200, 20.0);

int count = 0;
int delta = 10;
void setup() 
{
    // set servo control pin number
    Serial.begin(115200);
    servo.setServoControl(SERVO_PIN);
}

void loop() 
{
    servo.rotate_PID(count%360, 1);
    count = count + delta;
    Serial.println(count);
    delay(100);
}
