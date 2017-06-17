#include "PID.h"
#include <iostream>

using namespace std;

static const double MAX_STEERING_VALUE = 1.0;
static const double MIN_STEERING_VALUE = -1.0;

PID::PID() {}

PID::~PID() {}

void PID::init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::updateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::totalError() {
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  steer = max(MIN_STEERING_VALUE, steer);
  steer = min(MAX_STEERING_VALUE, steer);
  return steer;
}


