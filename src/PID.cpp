//****************************************************************************
//****************************************************************************
//*
//*                  PID for Control Systems
//*
//* Author(s):
//* Emanuele Belia
//*
//* Implementation notes:
//* None
//*
//****************************************************************************
//****************************************************************************/

//-----------------------------------
// INCLUDE FILES
//-----------------------------------
#include "PID.h"
#include <Arduino.h>
// #include <iostream>
// #include <algorithm>

//-----------------------------------
// PRIVATE DEFINES
//-----------------------------------
#define K_P_DEFAULT             0
#define K_I_DEFAULT             0
#define K_D_DEFAULT             0
#define K_FF_DEFAULT            0
#define ANTI_WIND_UP_DEFAULT    3
#define MIN_OUPUT_DEFAULT       -1
#define MAX_OUPUT_DEFAULT       1  
#define INITIAL_ERR_DEFAULT     0
#define DELTA_TIME_EPS          1e-7


// void print_coeffs(const PID& pid)
// {
//     // TODO: FAI I METHODI GET
//     printf("kp: %f \t ki: %f \t kd: %f \t kf: %f \t antiwindup: %f \t output: (%f, %f)\n", pid.getKp(), pid.getKi(), pid.getKd(), pid.getKf(), pid.getWindupGuard(), pid.getMinOutput(), pid.getMaxOutput());
// }

//-----------------------------------
// Constructors & destructors
//-----------------------------------
    
PID::PID() { init(); }
PID::PID(const double& K_P, const double& K_I, const double& K_D, const double& K_FF, const double& minimum, const double& maximum, const double& anti_wind_up_guard) : PID()
{
    setKp(K_P);
    setKi(K_I);
    setKd(K_D);
    setKf(K_FF);
    setWindupGuard(anti_wind_up_guard);
    setMinMaxOutput(minimum, maximum);
    // setSamplingTime(SAMPLING_TIME_DEFAULT);
    
}

PID::~PID(){}

//-----------------------------------
// Public methods
//-----------------------------------
int PID::update(const double& target, const double& feedback_value, double& output)
{
    update(target, feedback_value, 0.0, output);
}

int PID::update(const double& target, const double& feedback_value, const double& feed_forward, double& output)
{
// Compute error
    double error = target - feedback_value;
    // Get current time
    clock_t current_time = clock();
    //compute delta time
    double delta_time = (current_time - this->last_time) / (double) CLOCKS_PER_SEC;

    //-----------------------------------
    // UPDATE STATUS
    //-----------------------------------
    this->last_time = current_time;

    if(this->first_time_update)
    {
        // std::cout << "************* FIRST TIME *****************" << std::endl;
        this->first_time_update = false;
        delta_time = 0.0;
    }

    if (delta_time < DELTA_TIME_EPS ) 
    {
        output = 0.0;
        return -1;
    }

    //-----------------------------------
    // COMPUTE PROPORZIONAL
    //-----------------------------------
    this->P_e = getKp() * error;

    //-----------------------------------
    // COMPUTE INTEGRATIVE
    //-----------------------------------
    this->I_e += getKi() * error * delta_time;
 
    // Anti Windup Check
    this->I_e = max(this->I_e, -this->windup_guard);
    this->I_e = min(this->I_e, this->windup_guard);
    

    //-----------------------------------
    // COMPUTE DERIVATIVE
    //-----------------------------------
    // Compute error gradient
    double derivative_inc = (error - this->last_error) / delta_time;
    this->D_e = getKd() * derivative_inc;
    // printf("D_e: %f\n", this->D_e);


    //-----------------------------------
    // COMPUTE FEEDFORWARD
    //-----------------------------------
    this->FF = getKf() * feed_forward;

    this->last_error = error;

    //BIBO output
    this->output = std::max(this->min_output, std::min(this->max_output, (this->P_e + this->I_e + this->D_e + this->FF)));

    output = this->output;

    // printf("P_e: %f \t I_e: %f \t D_e: %f \t FF: %f \t error: %f \t output: %f \n", this->P_e, this->I_e, this->D_e, this->FF, error, output);
    // printf("target: %f \t feedback: %f \t error: %f \t output: %f \n", target, feedback_value, error, output);
    // this->print_coeffs();


    return 1;  
}

void PID::clear()
{
    // clear output
    this->output = 0.0;
    set_last_error(INITIAL_ERR_DEFAULT);

    clear_components_errors();

    this->first_time_update = true;

    init_time();
}

void PID::clear_components_errors()
{
    // Clear PID Errors
    this->P_e = 0; 
    this->I_e = 0;
    this->D_e = 0;
    this->FF  = 0;
}

void PID::setKp(const double& proportional_gain)    { this->K_P = proportional_gain; }
double PID::getKp() const    { return this->K_P; }

void PID::setKi(const double& integrative_gain)     { this->K_I = integrative_gain; }
double PID::getKi() const    { return this->K_I; }

void PID::setKd(const double& derivative_gain)      { this->K_D = derivative_gain; }
double PID::getKd() const    { return this->K_D; }

void PID::setKf(const double& feedforward_gain)     { this->K_FF = feedforward_gain; }
double PID::getKf() const    { return this->K_FF; }

void PID::setWindupGuard(const double& windup_guard){ this->windup_guard = windup_guard; }
double PID::getWindupGuard() const    { return this->windup_guard; }

void PID::set_last_error(const double& last_error)  { this->last_error = last_error;}
double PID::get_last_error() const    { return this->last_error; }

void PID::setMinOutput(const double& min_ouput)     { this->min_output = min_ouput; }
double PID::getMinOutput() const    { return this->min_output; }

void PID::setMaxOutput(const double& max_ouput)     { this->max_output = max_ouput; }
double PID::getMaxOutput() const    { return this->max_output; }
void PID::setMinMaxOutput(const double& min_ouput, const double& max_ouput)
{
    setMinOutput(min_ouput);
    setMaxOutput(max_ouput);
}
    
//-----------------------------------
// Protected methods
//-----------------------------------

//-----------------------------------
// Private methods
//-----------------------------------
void PID::init_time() { this->last_time = clock(); }
void PID::init() 
{ 
    clear();
    setKp(K_P_DEFAULT);
    setKi(K_I_DEFAULT);
    setKd(K_D_DEFAULT);
    setKf(K_FF_DEFAULT);
    setWindupGuard(ANTI_WIND_UP_DEFAULT);
    setMinMaxOutput(MIN_OUPUT_DEFAULT, MAX_OUPUT_DEFAULT);    
}


//****************************************************************************
//****************************************************************************

