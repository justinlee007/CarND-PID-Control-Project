#include "PidController.h"
#include <iostream>
#include <cmath>

using namespace std;

static const double MAX_STEERING_VALUE = 1.0;
static const double MIN_STEERING_VALUE = -1.0;

PidController::PidController() {}

PidController::~PidController() {}

void PidController::init(double Kp, double Ki, double Kd) {
  printf("Kp=%.4f, Ki=%.6f, Kd=%.3f\n", Kp, Ki, Kd);
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PidController::updateError(double cte) {
  d_error_ = cte - p_error_;
  i_error_ += cte;
  p_error_ = cte;
}

double PidController::totalError() {
  double steer = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
  steer = fmax(MIN_STEERING_VALUE, steer);
  steer = fmin(MAX_STEERING_VALUE, steer);
  return steer;
}


