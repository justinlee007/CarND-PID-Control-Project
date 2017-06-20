#include <cmath>
#include <iostream>
#include "Tracker.h"

Tracker::Tracker() {}
Tracker::~Tracker() {}

void Tracker::init(int sample_size) {
  sample_size_ = sample_size;
}

double Tracker::getAveTps() {
  return count_ / difftime(time(NULL), init_time_);;
}

void Tracker::onMessageProcessed(double cte, double speed, double throttle) {
  best_cte_ = fmin(fabs(cte), best_cte_);
  worst_cte_ = fmax(fabs(cte), worst_cte_);
  total_cte_ += fabs(cte);

  best_speed_ = fmax(speed, best_speed_);
  total_speed_ += speed;

  total_throttle_ += throttle;

  if (count_ == 0) {
    time(&init_time_);
  }
  if ((++count_ % sample_size_) == 0) {
    printTracking();
  }
}

void Tracker::printTracking() {
  ave_cte_ = total_cte_ / count_;
  ave_speed_ = total_speed_ / count_;
  ave_throttle_ = total_throttle_ / count_;
  ave_tps_ = count_ / difftime(time(NULL), init_time_);
  printf("best_cte_=%.5f, worst_cte_=%.5f, ave_cte=%.5f\n", best_cte_, worst_cte_, ave_cte_);
  printf("best_speed_=%.2f, ave_speed_=%.2f, ave_throttle_=%.3f, ave_tps_=%.1f\n", best_speed_, ave_speed_, ave_throttle_, ave_tps_);
}
