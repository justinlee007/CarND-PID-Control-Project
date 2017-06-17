#include "Twiddle.h"
#include <iostream>
#include <cmath>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::init(vector<double> params) {
  cout << "Initializing Filter \n";
  params_ = params;

//  paramDeltas_ << 0.01,0.0000001,0.01,0.02,0.05;
  //paramDeltas_ << 0.0,0.0,0.0,0.0,0.0;

  total_cte_ = 0;
  count_ = 0;

  increase_ = 1;
  param_num_ = 0;
}

void Twiddle::incrementCount(double cte) {
  total_cte_ += fabs(cte);
  count_++;
}

int Twiddle::getCount() {
  return count_;
}

void Twiddle::resetCount() {
  count_ = 0;
  total_cte_ = 0;
}

vector<double> Twiddle::updateParams() {

  printf("Kp=%.4f, Ki=%.4f, Kd=%.4f\n", params_[0], params_[1], params_[2]);

  curr_cte_ = total_cte_ / fmax(count_, 1);

  printf("prev_cte_=%.4f, curr_cte_=%.4f, increase_=%s (%i)\n", prev_cte_, curr_cte_, (increase_ ? "true" : "false"), increase_);

  if (increase_ == 1) {
    if (curr_cte_ < prev_cte_) {
      prev_cte_ = curr_cte_;
      paramDeltas_[param_num_] *= 1.1;
    } else {
      params_[param_num_] -= 2 * paramDeltas_[param_num_];
      increase_ = 0;
    }
    updateParamNum();
    return params_;
  } else {
    if (curr_cte_ < prev_cte_) {
      prev_cte_ = curr_cte_;
      paramDeltas_[param_num_] *= 1.1;
    } else {
      paramDeltas_[param_num_] *= 0.9;
      params_[param_num_] += paramDeltas_[param_num_];
    }
    increase_ = 1;
    updateParamNum();
    return params_;
  }
}

void Twiddle::updateParamNum() {
  if (++param_num_ == (params_.size() - 1)) {
    param_num_ = 0;
  }
}