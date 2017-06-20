#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class Twiddle {
 private:
  int count_;
  int twiddle_phase_;
  int param_num_;
  double total_cte_;
  double best_cte_;
  double tolerance_;

  std::vector<double> params_;
  std::vector<double> param_deltas_;

 public:

  /**
   * Constructor.
   */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /**
   * Initialize Twiddle.
   * @param Kp Proportional - to minimize CTE
   * @param Ki Integral - to adjust for steering drift
   * @param Kd Differential - to avoid overshooting
   */
  void init(double Kp, double Ki, double Kd);

  /**
   * Increment twiddle count and total cross track error
   * @param cte Cross Track Error value to sum
   */
  void incrementCount(double cte);

  /**
   * @return Current twiddle count (gets reset in updateParams())
   */
  int getCount();

  /**
   * @return Sum of all the parameter deltas
   */
  double getTolerance();

  /**
   * Evaluates error (sum(cte) / count) and runs the twiddle algorithm to adjust initial parameter values.
   * @return vector with updated {Kp, Ki, Kd} values
   */
  std::vector<double> updateParams();
};

#endif /* TWIDDLE_H */