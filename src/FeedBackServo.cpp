#include "FeedBackServo.h"
#include "PID.h"

Servo FeedBackServo::Parallax;
int FeedBackServo::feedbackPinNumber;
volatile int FeedBackServo::angle;
float FeedBackServo::thetaPre;
unsigned int FeedBackServo::tHigh, FeedBackServo::tLow;
unsigned long FeedBackServo::rise, FeedBackServo::fall;
int FeedBackServo::turns = 0;
float FeedBackServo::Kp = 1.0;

FeedBackServo::FeedBackServo(int _feedbackPinNumber)
{
    feedbackPinNumber = _feedbackPinNumber;
    int internalPinNumber = digitalPinToInterrupt(_feedbackPinNumber);
    attachInterrupt(internalPinNumber, feedback, CHANGE);
}

void FeedBackServo::setServoControl(int servoPinNumber)
{
    // Servo control pin attach
    Parallax.attach(servoPinNumber);
}


void FeedBackServo::rotate(int degree, int threshold)
{
    float output, offset, value;

    for(int errorAngle = degree - angle; abs(errorAngle) > threshold; errorAngle = degree - angle) {

        output = errorAngle * Kp;

        if(output > 200.0)
            output = 200.0;
            
        if(output < -200.0)
            output = -200.0;

        if(errorAngle > 0)
            offset = 30.0;
        else if(errorAngle < 0)
            offset = -30.0;
        else
            offset = 0.0;
        
        value = output + offset;
        Parallax.writeMicroseconds(1490 - value);
    }
    Parallax.writeMicroseconds(1490);
}

int FeedBackServo::Angle()
{
    return angle;
}

ICACHE_RAM_ATTR void FeedBackServo::feedback() 
{
    if(digitalRead(feedbackPinNumber)) {
        rise = micros();
        tLow = rise - fall;

        int tCycle = tHigh + tLow;
        if((tCycle < 1000) || (tCycle > 1200))
            return;
        
        float dc = (DUTY_SCALE * tHigh) / (float)tCycle;
        float theta = ((dc - DC_MIN) * UNITS_FC) / (DC_MAX - DC_MIN);

        if(theta < 0.0)
            theta = 0.0;
        else if(theta > (UNITS_FC - 1.0))
            theta = UNITS_FC - 1.0;

        if((theta < Q2_MIN) && (thetaPre > Q3_MAX))
            turns++;
        else if((thetaPre < Q2_MIN) && (theta > Q3_MAX))
            turns--;

        if(turns >= 0)
            angle = (turns * UNITS_FC) + theta;
        else if(turns < 0)
            angle = ((turns + 1) * UNITS_FC) - (UNITS_FC - theta);

        thetaPre = theta;
    } else {
        fall = micros();
        tHigh = fall - rise;
    }

}