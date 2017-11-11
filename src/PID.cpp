#include "PID.h"

using namespace std;

void PID::Init(double Kp, double Ki, double Kd, Twiddle twiddle) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    error_ = 0;
    iter_ = 0;
    is_initialized_ = false;
    twiddle_ = twiddle;
    // initialize K
    K_.push_back(Kp_+twiddle.dKp_);
    K_.push_back(Ki_+twiddle.dKi_);
    K_.push_back(Kd_+twiddle.dKd_);
}

double PID::UpdateError(double cte, double dt) {
  if (is_initialized_ == false) {
    cout << "PID initialized" << endl;
    iter_ = 0;
    error_ = 0;
    p_error_ = -Kp_*cte;
    i_error_ = 0;
    d_error_ = 0;
    is_initialized_ = true;
  }
  else {
    p_error_ = -Kp_*cte;
    i_error_ = i_error_ - Ki_*cte*dt;
    d_error_ = -Kd_*(cte-prev_cte_)/dt;
  }
  prev_cte_ = cte;
  iter_ += 1;
  double error = p_error_+i_error_+d_error_;
  return error;
}

void PID::SSE(double cte) {
  error_ += cte*cte;
}

bool PID::Process_twiddle(bool flag_reset) {
  if (flag_reset) {
    iter_ = 0;                  // reset iteration counter
    if (twiddle_.index_step_==0) {
        // update parameter
        K_[twiddle_.index_K_] -= 2*twiddle_.dK_[twiddle_.index_K_];
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        twiddle_.index_step_ = 1;           // move to second step
    }
    else if (twiddle_.index_step_==1) {
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // go back to initial value
        twiddle_.dK_[twiddle_.index_K_] *= 0.9;           // reduce dK
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dK
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        twiddle_.index_step_ = 0;           // move back to first step for next param
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
    }
    return true;
  }

  if (iter_>twiddle_.n_steps_) {
    if (twiddle_.index_step_==0) {  // first step of algorithm
      if (error_<twiddle_.best_err_) {
        twiddle_.best_err_ = error_;
        cout<<"Best error updated - step 0" << endl;
        // update parameter
        twiddle_.dK_[twiddle_.index_K_] *= 1.1;        // increase dK
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
      }
      else {
        // update parameter
        K_[twiddle_.index_K_] -= 2*twiddle_.dK_[twiddle_.index_K_]; // subtract dK to initial K value
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_step_ = 1;           // move to second step
      }
    }
    else if (twiddle_.index_step_==1) {  // second step of algorithm
      if (error_<twiddle_.best_err_) {
        twiddle_.best_err_ = error_;
        cout<<"Best error updated - step 1" << endl;
        // update parameter
        twiddle_.dK_[twiddle_.index_K_] *= 1.1;         // increase dK
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dK
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
      }
      else {
        cout<<"Best error not updated" << endl;
        // update parameter
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // go back to initial value
        twiddle_.dK_[twiddle_.index_K_] *= 0.9;           // reduce dK
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dK
        std::cout << "Kp: " << K_[0] << " Ki: " << K_[1] <<" Kd: " << K_[2] <<std::endl;
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_step_ = 0;           // move back to first step for next param
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
      }

    }
    // test if sum(dK)<tol
    if ((twiddle_.dK_[0]+twiddle_.dK_[1]+twiddle_.dK_[2])<twiddle_.tol_) {
      cout << "Optimization terminée!" << endl;
      twiddle_.is_used_ = false;
    }
    return true;
  }
  else {  // no reset
    return false;
  }
}
