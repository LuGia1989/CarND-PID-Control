#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
   // Coeffs and their error
   this->Kp = Kp;
   p_error = 0.0;

   this->Ki = Ki;
   i_error = 0.0;

   this->Kd = Kd;    
   d_error = 0.0;

   cte_ = 0.0;

}

void PID::UpdateError(double cte) {
   p_error = cte;
   d_error = cte - cte_;
   i_error=i_error+cte;
   
   cte_ = cte;    
}

double PID::TotalError() {
   return -Kp * p_error - Ki * i_error - Kd * d_error;
}

