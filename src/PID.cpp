#include "PID.h"
#include <iostream>

using namespace std;

static const double MAX_STEERING_VALUE = 1.0;
static const double MIN_STEERING_VALUE = -1.0;

PID::PID() {}

PID::~PID() {}

void PID::init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::updateError(double cte) {
  d_error_ = cte - p_error_;
  i_error_ += cte;
  p_error_ = cte;
}

double PID::totalError() {
  double steer = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
  steer = max(MIN_STEERING_VALUE, steer);
  steer = min(MAX_STEERING_VALUE, steer);
  return steer;
}


