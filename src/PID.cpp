#include "PID.h"
#include <iostream>

using namespace std;

static const double MAX_STEERING_VALUE = 1.0;
static const double MIN_STEERING_VALUE = -1.0;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  steer = max(MIN_STEERING_VALUE, steer);
  steer = min(MAX_STEERING_VALUE, steer);
  return steer;
}


