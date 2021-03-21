#ifndef FEEDBACK360_CONTROL_LIBRARY
#define FEEDBACK360_CONTROL_LIBRARY
#include <Arduino.h>
#include <Servo.h>
#include "PID.h"

#define DC_MIN 0.029
#define DC_MAX 0.971
#define UNITS_FC 360
#define DUTY_SCALE 1
#define Q2_MIN UNITS_FC/4
#define Q3_MAX Q2_MIN * 3

class FeedbackServo : public PID
{
    public:
        FeedbackServo(int feedbackPinNumber);
        FeedbackServo(int _feedbackPinNumber, const double& Kp, const double& Ki, const double& Kd, const double& Kff, const double& minimum, const double& maximum, const double& anti_wind_up_guard);
        void setServoControl(int servoPinNumber);
        void rotate_PID(int degrees, int threshold);
        void rotate(int degrees, int threshold);
        void write(int degrees);

        int read();
        float out_glob = 0;

    
    private:
        static void feedback();
        static Servo Parallax;
        static int feedbackPinNumber;
        static volatile int angle;
        static float thetaPre;
        static unsigned int tHigh, tLow;
        static unsigned long rise, fall;
        static int turns;
        static float Kp;
        
};

#endif