#ifndef TWIDDLE_H
#define TWIDDLE_H

class Twiddle {

public:
  bool tuning;  // Are we tuning parameters i.e. using Twiddle?
  double tolerance;  // Tolerance for when to stop tuning
  int step;  // Current simulator step
  int max_steps; // Number of steps over which to tune parameters
  int epoch;  // Current number of tuning epoch since last improvement
  int max_epochs;  // Max number of tuning epochs without improvement
  double dp_factor;  // Factor by which to increase or decrease dp's
  double sum_err;
  double avg_err;  // Average error over n_steps 
  double best_err;  // Best recorded average error so far in tuning process
  double p[3];  // Array of currently active parameters
  double dp[3];  // Array of currently active parameter variances
  double dp_ini[3];  //  Array of initial parameter variances
  double best_p[3];  // Array with the best parameters seen so far
  int directions[3];  // Flags taking values -1 or 1 used to select branch in algorithm
  int p_i;  // Index of parameter currently being tuned

  Twiddle();  // Constructor

  virtual ~Twiddle();  // Destructor

  // Initialize Twiddle
  void Init(const double Kp, const double Ki, const double Kd,
    const double dKp, const double dKi, const double dKd, 
    const double tol, const int steps, const int epochs);

  void updateDp(double factor);
  /*
  Precondition: factor is either 1 - dp_factor or 1 + dp_factor. dp elements have double values.
  Postcontition: dp elements are multiplied by factor.
  */

  double sumScaledDp();
  /*
  Precondition: dp is an array that holds the tuning step size for each of the three PID parameters. dp_ini 
    are the initial values for each parameter and are used to scale the sum of Dps.
  Postcondition: Returns the sum of the scaled values in dp.
  */

};


#endif /* TWIDDLE_H */