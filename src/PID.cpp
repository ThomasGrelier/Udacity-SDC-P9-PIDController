#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, Twiddle twiddle) {
    if (twiddle.is_used_) {
      // prepare first iteration of twiddle: add dKp to Kp
      Kp_ = Kp + twiddle.dKp_;
    }
    else {
      Kp_ = Kp;
    }
    Ki_ = Ki;
    Kd_ = Kd;
    error_ = 0;
    iter_ = 0;
    is_initialized_ = false;
    twiddle_ = twiddle;
    // initialize K
    K_.push_back(Kp_);
    K_.push_back(Ki_);
    K_.push_back(Kd_);
}

void PID::UpdateError(double cte, double dt) {
  if (is_initialized_ == false) {
    //cout << "PID initialized" << endl;
    iter_ = 0;
    error_ = 0;
    p_error_ = -Kp_*cte;
    i_error_ = 0;
    d_error_ = 0;
    is_initialized_ = true;
  }
  else {
    p_error_ = cte;
    // reset I component is cte is zero or too big
    if ((fabs(cte)<0.001) || (fabs(cte)>6)) {
      i_error_ = 0;
    }
    else {
      i_error_ = i_error_ + cte*dt;
    }
    d_error_ = (cte-prev_cte_)/dt;
  }
  prev_cte_ = cte;
  iter_ += 1;
  //double error = -Kp_*p_error_-Ki_*i_error_-Kd_*d_error_;
  //return error;
}

double PID::TotalError() {
  double error = -Kp_*p_error_-Ki_*i_error_-Kd_*d_error_;
  return error;
}

void PID::SSE(double cte) {
  if (iter_ > twiddle_.n_init_step_rem_) {
    error_ += cte*cte;
  }
}

bool PID::Process_twiddle(bool flag_reset) {
  if (flag_reset && iter_>2) {  // check iter_>1 to avoid unwanted reset
    iter_ = 0;                  // reset iteration counter
    error_ = -1;
    if (twiddle_.index_step_==0) {
        cout<<"Error = "<<error_<<" - Best error not updated @ step 0 -> step 1" << endl;
        K_[twiddle_.index_K_] -= 2*twiddle_.dK_[twiddle_.index_K_];
        twiddle_.index_step_ = 1;           // move to second step
    }
    else if (twiddle_.index_step_==1) {
        cout<<"Error = "<<error_<<" - Best error not updated @ step 1" << endl;
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // go back to initial value
        twiddle_.dK_[twiddle_.index_K_] *= 0.8;           // reduce dK
        twiddle_.index_step_ = 0;           // move back to first step for next param
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dp to next K value
    }
    Kp_ = K_[0];
    Ki_ = K_[1];
    Kd_ = K_[2];
    cout << "-------------------------------------------"<<endl;
    cout << "Next twiddle iteration: ";
    cout << "Kp: " << Kp_ << " Ki: " << Ki_ <<" Kd: " << Kd_ <<endl;
    return true;
  }
  else if (iter_>twiddle_.n_steps_) {
    if (twiddle_.index_step_==0) {  // first step of algorithm
      //cout << "test" << endl;
      if (error_<twiddle_.best_err_) {
        twiddle_.best_err_ = error_;
        cout<<"Error = "<<error_<<" - Best error updated @ step 0 -> YEAH!" << endl;
        // update parameter
        twiddle_.dK_[twiddle_.index_K_] *= 1.2;        // increase dK
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];
      }
      else {
        cout<<"Error = "<<error_<<" - Best error not updated @ step 0 -> step 1" << endl;
        // update parameter
        K_[twiddle_.index_K_] -= 2*twiddle_.dK_[twiddle_.index_K_]; // subtract dK to initial K value
        iter_ = 0;                  // reset iteration counter
        twiddle_.index_step_ = 1;           // move to second step
      }
    }
    else if (twiddle_.index_step_==1) {  // second step of algorithm
      if (error_<twiddle_.best_err_) {  // error is better
        twiddle_.best_err_ = error_;
        cout<<"Error = "<<error_<<" - Best error updated @ step 1 -> YEAH!" << endl;
        twiddle_.dK_[twiddle_.index_K_] *= 1.2;       // increase dK
        iter_ = 0;                                    // reset iteration counter
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dp to next p value
        twiddle_.index_step_ = 0;           // move back to first step for next param
      }
      else {
        cout<<"Error = "<<error_<<" - Best error not updated @ step 1" << endl;
        // update parameter
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // go back to initial value
        twiddle_.dK_[twiddle_.index_K_] *= 0.8;           // reduce dK
        iter_ = 0;                          // reset iteration counter
        twiddle_.index_step_ = 0;           // move back to first step for next param
        twiddle_.index_K_ = (twiddle_.index_K_+1)%3;  // move to next K index
        K_[twiddle_.index_K_] += twiddle_.dK_[twiddle_.index_K_];  // add dp to next K value
      }
    }
    Kp_ = K_[0];
    Ki_ = K_[1];
    Kd_ = K_[2];
    // test if sum(dK)<tol
    if ((twiddle_.dK_[0]+twiddle_.dK_[1]+twiddle_.dK_[2])<twiddle_.tol_) {
      cout << "Optimization terminée!" << endl;
      twiddle_.is_used_ = false;
    }
    else {
      cout << "-------------------------------------------"<<endl;
      cout << "Next twiddle iteration: ";
      cout << "Kp: " << Kp_ << " Ki: " << Ki_ <<" Kd: " << Kd_ <<endl;
    }
    return true;
  }
  else {  // no reset
    return false;
  }
}
