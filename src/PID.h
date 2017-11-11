#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>
#include "twiddle.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

  double error_;        // Total squared error
  int iter_;            // iteration #
  double prev_cte_;     // for derivative error
  bool is_initialized_;  // initialization not done
  long long msSinceEpochPrev_;  // time tag (ms)

  /*
  Twiddle object
  */
  Twiddle twiddle_;
  // concatenation of Kp, Ki, Kd
  std::vector<double> K_;

  // Constructor
  PID();
  //Destructor.
  virtual ~PID();
  // Init
  void Init(double Kp, double Ki, double Kd, Twiddle twiddle);
  // Update the PID error given cross track error and time_step
  double UpdateError(double cte, double dt);
  //Compute sum of  squared cte
  void SSE(double cte);
  //Perform twiddle - Returns a boolean indicating to reset simulator
  bool Process_twiddle(bool flag_reset);

};

#endif /* PID_H */
