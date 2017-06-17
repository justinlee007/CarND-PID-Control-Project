#include "Twiddle.h"
#include <iostream>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::init(double gamma, double wait_count, double set_speed, vector<double> params) {
  cout << "Initializing Filter \n";
  this->gamma = gamma;
  this->wait_count = wait_count;
  this->set_speed = set_speed;
  this->params_ = params;

  Prev_Params = params;

//  dParams << 0.01,0.0000001,0.01,0.02,0.05;
  //dParams << 0.0,0.0,0.0,0.0,0.0;

  prev_err = 0;
  count = 0;
  increase = 1;
  param_num = 0;
}

void Twiddle::savePrevious(double value) {
  prev_err = error;
}

void Twiddle::incrementCount() {
  count++;
}

double Twiddle::getCount() {
  count += 1;
  return count;
}

void Twiddle::resetCount() {
  count = 0;
}

void Twiddle::calcError(double cte, double speed) {
  //cout<<", Prev_err : "<< prev_err;
  error = .05 * (set_speed - speed) * (set_speed - speed) / set_speed / set_speed + cte * cte;
  error = error + gamma * prev_err;
  //cout<<"Err : "<<error<<endl;
  prev_err = error;
}

vector<double> Twiddle::updateParams() {

  cout << "Kp_ : " << params_[0];
  cout << ",Ki_ : " << params_[1];
  cout << ",Kd_ : " << params_[2];
  cout << ",Ks : " << params_[3];
  cout << ",Kst : " << params_[4];
  cout << endl;

  cout << "Old_err : " << old_err;
  cout << ",error : " << error;
  cout << endl;
  cout << "increase : " << increase;
  cout << endl;

  if (increase == 1) {
    if (old_err >= error) {
      dParams[param_num] *= 1.1;
      old_err = error;
      param_num++;
      if (param_num == 5) {
        param_num = 0;
      }
      return params_;
    } else {
      params_[param_num] -= 2 * dParams[param_num];
      increase = 0;
      old_err = error;
      param_num++;
      if (param_num == 5) {
        param_num = 0;
      }
      return params_;
    }
  }
  if (increase == 0) {
    if (old_err >= error) {
      dParams[param_num] *= 1.1;
      old_err = error;
      param_num++;
      if (param_num == 5) {
        param_num = 0;
      }
      return params_;
    } else {
//      dParams *= 0.9;
      params_[param_num] += dParams[param_num];
      increase = 1;
      old_err = error;
      param_num++;
      if (param_num == 5) {
        param_num = 0;
      }
      return params_;
    }
  }
  return params_;
}