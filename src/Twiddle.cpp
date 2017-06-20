#include <cmath>
#include "Twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::init(double Kp, double Ki, double Kd) {
  params_ = {Kp, Ki, Kd};
  param_deltas_ = {(Kp / 10), (Ki / 10), (Kd / 10)};
  count_ = 0;
  twiddle_phase_ = 0; // TODO: this should be an enum or use callbacks
  param_num_ = 0;
  total_cte_ = 0.0;
  best_cte_ = 1.0;
  tolerance_ = 0.0;
}

void Twiddle::incrementCount(double cte) {
  // TODO: which one is right?
  // total_cte_ += pow(cte, 2);
  total_cte_ += fabs(cte);
  count_++;
}

int Twiddle::getCount() {
  return count_;
}

double Twiddle::getTolerance() {
  return tolerance_;
}

std::vector<double> Twiddle::updateParams() {
  double curr_cte = total_cte_ / count_;
  count_ = 0;
  total_cte_ = 0;
  tolerance_ = param_deltas_[0] + param_deltas_[1] + param_deltas_[2];

  // printf("curr_cte=%.4f, best_cte_=%.4f, twiddle_phase_=%i, param_num_=%i\n", curr_cte, best_cte_, twiddle_phase_, param_num_);
  printf("\u0394Kp=%.5f, \u0394Ki=%.7f, \u0394Kd=%.4f, tolerance=%.3f\n", param_deltas_[0], param_deltas_[1], param_deltas_[2], tolerance_);

  bool update_params = false;
  if (twiddle_phase_ == 0) {
    params_[param_num_] += param_deltas_[param_num_];
    twiddle_phase_ = 1;
  } else if (twiddle_phase_ == 1) {
    if (curr_cte < best_cte_) {
      best_cte_ = curr_cte;
      param_deltas_[param_num_] *= 1.1;
      twiddle_phase_ = 0;
      update_params = true;
    } else {
      params_[param_num_] -= 2 * param_deltas_[param_num_];
      twiddle_phase_ = 2;
    }
  } else if (twiddle_phase_ == 2) {
    if (curr_cte < best_cte_) {
      best_cte_ = curr_cte;
      param_deltas_[param_num_] *= 1.1;
    } else {
      params_[param_num_] += param_deltas_[param_num_];
      param_deltas_[param_num_] *= 0.9;
    }
    twiddle_phase_ = 0;
    update_params = true;
  }
  if (update_params && (++param_num_ == 3)) {
    param_num_ = 0;
  }
  return params_;
}