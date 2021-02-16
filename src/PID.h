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

#ifndef PID_H
#define PID_H


#include <time.h>


//------------------------------
// PUBLIC DEFINES
//------------------------------
// None

//------------------------------
// PUBLIC MACROS
//------------------------------
// None

//------------------------------
// PUBLIC TYPEDEFS
//------------------------------
// None

//-----------------------------------
// CLASS DECLARATION
//-----------------------------------

//----------------------------------------------------------------------------
// Class name - Class description
//----------------------------------------------------------------------------
///
/// @brief Defines a class for stuff. 
///
class PID
{
    public:
        //------------------------------
        // Constructors & Destructors
        //------------------------------
        /**
         * @brief PID Constructor
        */
        PID();
        PID(const double& K_P, const double& K_I, const double& K_D, const double& K_FF, const double& minimum, const double& maximum, const double& anti_wind_up_guard);

        /**
         * @brief PID Destructor
        */
        ~PID();
        
        //------------------------------
        // Public methods
        //------------------------------	
        /**
         * @brief   update: compute PID output value
         * 
         * @param target [in] needed target
         * @param feedback_value [in] feedback value from you're system
         * @param feed_forward [in] feedforward value
         * @param output [out] feed_forward: feedforward value
         * @return 1 if the PID output has been succesfully computed, -1 otherwise.
        */
        int update(const double& target, const double& feedback_value, const double& feed_forward, double& output);
        int update(const double& target, const double& feedback_value, double& output);

        /**
         * @brief   clear: Clears PID error and coefficients 
        */
        void clear();
        void clear_components_errors();


        // void print_coeffs();


        /**
         * @brief   setKp: Determines how aggressively the PID reacts to the current error with setting Proportional Gain
         * @param proportional_gain [in] proportional value
         * 
        */
        void setKp(const double& proportional_gain);
        double getKp() const;

        /**
         * @brief   setKi: Determines how aggressively the PID reacts to the current error with setting Integrative Gain
         * @param proportional_gain [in] Integrative value
         * 
        */
        void setKi(const double& integrative_gain);
        double getKi() const;

        /**
         * @brief   setKp: Determines how aggressively the PID reacts to the current error with setting Derivative Gain
         * @param proportional_gain [in] Derivative value
         * 
        */
        void setKd(const double& derivative_gain);
        double getKd() const;

        /**
         * @brief   setKf: Determines how aggressively the PID reacts to the current error with setting Feed Forward Gain
         * @param feedforward_gain [in] feed forward value
         * 
        */
        void setKf(const double& feedforward_gain);
        double getKf() const;
        
        /**
         * @brief   setWindup: Integral windup, also known as integrator windup or reset windup,
         *  refers to the situation in a PID feedback controller where
         *  a large change in setpoint occurs (say a positive change)
         *  and the integral terms accumulates a significant error
         *  during the rise (windup), thus overshooting and continuing
         *  to increase as this accumulated error is unwound
         *  (offset by errors in the other direction).
         *  The specific problem is the excess overshooting.
         * 
         * @param windup_guard [in] Maximum anti windup value.
         * 
        */
        void setWindupGuard(const double& windup_guard);
        double getWindupGuard() const;
        
        /**
         * @brief   Set the PID error
         * 
         * @param last_error [in] starting error for the PID
        */
        void set_last_error(const double& last_error);
        double get_last_error() const;

        /**
         * @brief   setSamplingTime: PID that should be updated at a regular interval.
         *   Based on a pre-determined sampling time, the PID decides if it should compute or return immediately.
         * 
         * @param sampling_time [in] set sample time
         * 
        */
        // void setSamplingTime(const double& sampling_time);

        /**
         * @brief   Lower bound PID output
         * 
         * @param min_ouput [in] set min output
         * 
        */
        void setMinOutput(const double& min_ouput);
        double getMinOutput() const;
        
        /**
         * @brief   Upper bound PID output
         * 
         * @param max_ouput [in] set max output
         * 
        */
        void setMaxOutput(const double& max_ouput);
        double getMaxOutput() const;

        /**
         * @brief   Bound PID output
         * 
         * @param min_ouput [in] set min output
         * @param max_ouput [in] set max output
         * 
        */
        void setMinMaxOutput(const double& min_ouput, const double& max_ouput);

        //-----------------------------------
        // Private methods
        //-----------------------------------

        /**
         * @brief   Initialize PID object
         * 
        */   
        void init() ;

        /**
         * @brief   Initialize PID time
         * 
        */    
        void init_time();

        //------------------------------
        // Private members
        //------------------------------
    private:        
        // Propotional Gain
        double K_P;
        // Integrative Gain
        double K_I;
        // Derivative Gain
        double K_D;
        // FeedForward Gain 
        double K_FF;
        // Proportional Error
        double P_e; 
        // Integrative Error
        double I_e;
        // Derivative Error
        double D_e;
        //FeefForward action
        double FF;
        // Min value output
        double min_output;
        // Max value output
        double max_output;
        double sampling_time;
        double windup_guard;
        // PID output
        double output;
        // last error for derivative computation
        double last_error;
        bool first_time_update;

        clock_t last_time;
};

// void print_coeffs(const PID& pid);

#endif // PID_H

//****************************************************************************
//****************************************************************************
