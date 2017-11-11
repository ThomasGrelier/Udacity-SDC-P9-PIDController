#include "twiddle.h"

using namespace std;

void Twiddle::Init(int n_steps, double best_err, double dKp, double dKi, double dKd, double tol) {
    is_used_ = true;
    index_K_ = 0;
    index_step_ = 0;
    n_steps_ = n_steps;
    best_err_ = best_err;
    dKp_ = dKp;
    dKi_ = dKi;
    dKd_ = dKd;
    tol_ = tol;
    // initialize dK_
    dK_.push_back(dKp_);
    dK_.push_back(dKi_);
    dK_.push_back(dKd_);
}

