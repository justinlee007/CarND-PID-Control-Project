#ifndef PID_H
#define PID_H

class PID {
 public:
  // Errors
  double p_error_;
  double i_error_;
  double d_error_;

  // Coefficients
  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param Kp Proportional - to minimize CTE
   * @param Ki Integral - to adjust for steering drift
   * @param Kd Differential - to avoid overshooting
   */
  void init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte Cross Track Error
   */
  void updateError(double cte);

  /**
   * Calculate the total PID error.
   */
  double totalError();
};

#endif /* PID_H */
