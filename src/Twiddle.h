#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

using namespace std;

class Twiddle {
 public:

  double count = 0;
  double gamma = 0;
  double wait_count = 100;
  double set_speed = 0;
  double increase = 0;
  double decrease = 0;
  double param_num = 0;
  double prev_err;
  double old_err = 0;
  double error;

  vector<double> params_ = vector<double>(5);
  vector<double> Prev_Params = vector<double>(5);
  vector<double> dParams = vector<double>(5);

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize PID.
  */
  void init(double gamma, double wait_count, double set_speed, vector<double> params);

  void incrementCount();

  double getCount();

  void resetCount();

  void calcError(double cte, double speed);

  void savePrevious(double value);

  vector<double> updateParams();
};

#endif /* TWIDDLE_H */