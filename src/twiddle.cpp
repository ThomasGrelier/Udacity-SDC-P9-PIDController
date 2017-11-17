#include "twiddle.h"

using namespace std;

Twiddle::Twiddle() {is_used_=false;};
Twiddle::~Twiddle() {};

void Twiddle::Init(int n_steps, double dKp, double dKi, double dKd, double tol, int n_init_step_rem) {
    is_used_ = true;
    index_K_ = 0;
    index_step_ = 0;
    n_steps_ = n_steps;
    best_err_ = 1e8;
    dKp_ = dKp;
    dKi_ = dKi;
    dKd_ = dKd;
    tol_ = tol;
    n_init_step_rem_ = n_init_step_rem;
    // initialize dK_
    dK_.push_back(dKp_);
    dK_.push_back(dKi_);
    dK_.push_back(dKd_);
}

