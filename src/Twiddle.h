#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

using namespace std;

class Twiddle {
 public:

  int count_ = 0;
  int param_num_ = 0;
  bool increase_ = false;
  double prev_cte_ = 0;
  double curr_cte_;
  double total_cte_;

  vector<double> params_ = vector<double>(5);
  vector<double> paramDeltas_ = vector<double>(5);

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  ~Twiddle();

  /*
  * Initialize PID.
  */
  void init(vector<double> params);

  /**
   *
   * @param cte
   */
  void incrementCount(double cte);

  /**
   *
   * @return
   */
  int getCount();

  /**
   *
   */
  void resetCount();

  /**
   *
   * @return
   */
  vector<double> updateParams();

  /**
   *
   */
  void updateParamNum();
};

#endif /* TWIDDLE_H */