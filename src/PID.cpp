#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  //Set parameters
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  //Initialize errors to 0.
  p_error = 0.;
  i_error = 0.;

}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}


double PID::TotalError() {
  double total = Kp * p_error + Ki * i_error + Kd * d_error;

  return total;
}

