#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  //Precondition: cte is a double representing the cross-track error of the vehicle.
  //  p_error, i_error and d_error contain values from the previous execution.
  //Postcondition: Error values are updated according to the PID-controller formulas.

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  //Postcondition: Returns the of errors weighted by the K_ parameters
  
};

#endif /* PID_H */
