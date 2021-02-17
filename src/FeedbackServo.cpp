#include "FeedbackServo.h"

Servo FeedbackServo::Parallax;
int FeedbackServo::feedbackPinNumber;
volatile int FeedbackServo::angle;
float FeedbackServo::thetaPre;
unsigned int FeedbackServo::tHigh, FeedbackServo::tLow;
unsigned long FeedbackServo::rise, FeedbackServo::fall;
int FeedbackServo::turns = 0;
float FeedbackServo::Kp = 1.0;



FeedbackServo::FeedbackServo(int _feedbackPinNumber) : PID::PID()
{

    feedbackPinNumber = _feedbackPinNumber;
    int internalPinNumber = digitalPinToInterrupt(_feedbackPinNumber);
    attachInterrupt(internalPinNumber, feedback, CHANGE);
}

// FeedbackServo::FeedbackServo(int _feedbackPinNumber, double _Kp, double _Ki, double _Kd, double wind) : PID::PID()
FeedbackServo::FeedbackServo(int _feedbackPinNumber, const double& Kp, const double& Ki, const double& Kd, const double& Kff, const double& minimum, const double& maximum, const double& anti_wind_up_guard) 
: PID::PID(Kp, Ki, Kd, Kff, minimum, maximum, anti_wind_up_guard)
{

    feedbackPinNumber = _feedbackPinNumber;
    int internalPinNumber = digitalPinToInterrupt(_feedbackPinNumber);
    attachInterrupt(internalPinNumber, feedback, CHANGE);
}

void FeedbackServo::setServoControl(int servoPinNumber)
{
    // Servo control pin attach
    Parallax.attach(servoPinNumber);
}


ICACHE_RAM_ATTR void FeedbackServo::rotate_PID(int degree, int threshold)
{
    double output, offset, value;

    for(int errorAngle = degree - angle; abs(errorAngle) > threshold; errorAngle = degree - angle) {

        Serial.println("-------here");
        
        Serial.println(PID::update(errorAngle, 0.0, output));
        Serial.print(output);
        Serial.print(" -- ");
        Serial.println(errorAngle);


        out_glob = output;

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

void FeedbackServo::rotate(int degree, int threshold)
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

int FeedbackServo::Angle()
{
    return angle;
}

ICACHE_RAM_ATTR void FeedbackServo::feedback() 
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
