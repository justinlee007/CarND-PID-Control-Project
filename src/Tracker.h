#ifndef TRACKER_H
#define TRACKER_H

class Tracker {
 private:
  int sample_size_;
  int count_ = 0;

  // CTE tracking
  double best_cte_ = 1.0;
  double worst_cte_ = 0.0;
  double total_cte_ = 0.0;
  double ave_cte_ = 0.0;

  // Throttle tracking
  double ave_throttle_ = 0.0;
  double total_throttle_ = 0.0;

  // Speed tracking
  double best_speed_ = 0.0;
  double ave_speed_ = 0.0;
  double total_speed_ = 0.0;

  // Throughput tracking
  time_t init_time_;
  double ave_tps_ = 0.0;

  void printTracking();

 public:

  /**
   * Constructor.
   */
  Tracker();

  /**
   * Destructor.
   */
  virtual ~Tracker();

  void init(int sample_size);

  double getAveTps();

  void onMessageProcessed(double cte, double speed, double throttle);

};

#endif //TRACKER_H
