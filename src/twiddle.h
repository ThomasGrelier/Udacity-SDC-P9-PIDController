#ifndef TWIDDLE_H_INCLUDED
#define TWIDDLE_H_INCLUDED
#include <iostream>
#include <vector>

class Twiddle {

public:
  bool is_used_;        // perform twiddle
  int n_steps_;         // nb steps
  double best_err_;     // best error
  double dKp_;          // Kp increment
  double dKi_;          // Ki increment
  double dKd_;          // Kd increment
  double tol_;          // for stopping twiddle
  std::vector<double> dK_;    // concatenation of dK
  int index_K_;         // index of vector K to be used
  int index_step_;      // step in the twiddle algorithm

  //Constructor
  Twiddle();
  // Destructor.
  virtual ~Twiddle();
  //Init twiddle.
  void Init(int n_steps, double best_err, double dKp, double dKi, double dKd, double tol);
};



#endif // TWIDDLE_H_INCLUDED
