#include "PID.h"
#include <iostream>
#include <numeric>      // std::accumulate
#include <math.h>
#include <algorithm>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte; 

  number_of_iteration++;   
}

void PID::ProcessTwiddle()
{
  squared_error_sum_ += p_error * p_error;
    number_processed_inputs_++;
    if (number_processed_inputs_ == 20)
    {
        const auto RMSE{sqrt(squared_error_sum_ / 20)};
        Twiddle(RMSE);
        number_processed_inputs_ = 0;
        squared_error_sum_ = 0.;
    }
}

void PID::Twiddle(double error, double tol) {
  
  double sum{std::accumulate(dp.begin(), dp.end(), decltype(dp)::value_type(0))};
  std::vector<double> p{Kp,Ki,Kd};
  std::cout << "sum" << sum << "tol" << tol <<std::endl; 
  if(sum > tol)
  {
    if(error < best_error)
    {
      best_error = error;
      dp[current_index] *= 1.1;
      twiddle_state = TwiddleState::kReset;  
    }
    else
    {
      switch(twiddle_state)
      {
        case TwiddleState::kReset:
          p[current_index] += dp[current_index];
          twiddle_state = TwiddleState::kAdded;
          break;
        case TwiddleState::kAdded:
          p[current_index] -= 2*dp[current_index];
          twiddle_state = TwiddleState::kSubtracted;
          break;
        case TwiddleState::kSubtracted:
        default:
          p[current_index] += dp[current_index];
          dp[current_index] *= 0.9;
          twiddle_state = TwiddleState::kReset;
          break;  
      }
    }
    if(TwiddleState::kReset == twiddle_state)
    {
      current_index = (current_index + 1) % 3;
    }
    Init(p[0], p[1], p[2]);

    std::cout << " ************* twiddle params " << Kp  << " " <<  Ki << " "  << Kd  << std::endl; 
    sum = std::accumulate(dp.begin(), dp.end(), decltype(dp)::value_type(0));
  }
  else
  {
    static bool onlyOnce{true};
    if(onlyOnce)
    {
      onlyOnce = false;
      std::cout << " ************* twiddle finished " << Kp  << " " <<  Ki << " "  << Kd  << std::endl;
    }
  }
  
}

double PID::GetSteering(void)
{
  auto steering = (- (Kp*p_error) - (Kd*d_error) - (Ki*i_error)); 
  if (std::fabs(steering) > 1.)
  {
    return (steering > 0.) ? 1. : -1;
  }
  return steering; 
}

std::tuple<double, double, double> PID::GetParams() {
  return std::make_tuple(Kp, Ki, Kd);  
}