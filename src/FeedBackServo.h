#ifndef FEEDBACK360_CONTROL_LIBRARY
#define FEEDBACK360_CONTROL_LIBRARY
#include <Arduino.h>
#include <Servo.h>

#define DC_MIN 0.029
#define DC_MAX 0.971
#define UNITS_FC 360
#define DUTY_SCALE 1
#define Q2_MIN UNITS_FC/4
#define Q3_MAX Q2_MIN * 3




class FeedBackServo {
    public:
        FeedBackServo(int feedbackPinNumber = 2);
        void setServoControl(int servoPinNumber = 3);
        void setKp(float _Kp = 1.0);
        void rotate(int degree, int threshold = 4);
        int Angle();
    
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