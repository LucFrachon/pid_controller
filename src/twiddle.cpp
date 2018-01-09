#include "twiddle.h"

using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(const double Kp, const double Ki, const double Kd,
  const double dKp, const double dKi, const double dKd, const double tol, 
  const int steps, const int epochs) {

  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;

  best_p[0] = Kp;
  best_p[1] = Ki;
  best_p[2] = Kd;

  dp[0] = dKp;
  dp[1] = dKi;
  dp[2] = dKd;


  dp_ini[0] = dKp;
  dp_ini[1] = dKi;
  dp_ini[2] = dKd;

  directions[0] = 1;
  directions[1] = 1;
  directions[2] = 1;

  tuning = false;
  tolerance = tol;
  step = 1;
  epoch = 0;
  max_steps = steps;
  max_epochs = epochs;
  dp_factor = 0.25;
  p_i = 0;
  avg_err = 0.;
  best_err = 9999999;  //some very high value
  sum_err = 0.;

}

void Twiddle::updateDp(double factor)
{
  dp[p_i] *= factor;
}

double Twiddle::sumScaledDp()
{
  double sum = dp[0] / dp_ini[0] + dp[1] / dp_ini[1] + dp[2] / dp_ini[2];

  return sum;
}