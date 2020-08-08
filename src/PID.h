#ifndef PID_H
#define PID_H

#include <tuple>
#include <vector>


enum class TwiddleState
{
  kReset,
  kAdded,
  kSubtracted
};

class PID {
 public:
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
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);


  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  
  void ProcessTwiddle();

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  std::tuple<double, double, double> GetParams();

  double GetSteering(void);

 private:
  void Twiddle(double error, double tol = 1E-6);
  int number_processed_inputs_{};
  double squared_error_sum_{};
  double best_error{1E5};

  /**
   * PID Errors
   */
  double p_error{0};
  double i_error{0};
  double d_error{0};

  /**
   * PID Coefficients
   */ 
  double Kp{};
  double Ki{};
  double Kd{};

  std::vector<double> dp{.1, .001, .5}; 

  int number_of_iteration{0};
  int current_index{0};

  TwiddleState twiddle_state{TwiddleState::kReset};
};

#endif  // PID_H