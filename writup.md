# **PID Controller**

## Writeup

---

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) individually and describe how I addressed each point in my implementation.

---
## Compilation

The code compiles and executes well.

---
## Implementation

Basic PID controller is implemented. It's P, I and D params can be adjusted from the main() using init.

PID class is modified to update p error, i error and d error which are used to calculate the steering value. It also include Twiddle function to calculate the params when the car is moving.
Twiddle takes a CTE from a fixed number of steps, calculates RMSE from those and use it as a objective function. After that the parameters are updated according to the twiddle algorithm, and the next set of CTE values is taken in the following iteration.

Once Twiddle finds the params with error less that tolerance 0.000001, then Twiddle logic is stopped and it is not needed anymore. Hence after param calucation using twiddle, twiddle logic is disabled as you can observe in main function.


---
## Reflection

P parameter make vehicle steer more toward the reference trajectory but resultant trajectory is found to be oscilating around reference. Hence D param is used to make this oscillation smooth. It uses difference of cte errors to make the transition smooth. However, this PD combination doesnt count for the bias in steering because of which driver might have to steer hard, hence I parameter is used which uses cummulative error factor to achieve the desired result.

### Parameter Tuning

#### Manual Tuning

I have used params provided in class lessons as reference and found them to be working fine. Then I used twiddle algorithm to fine tune these params.

Hence values that I used for the manual tuning:
P: 0.2; I: 0.004; D: 3

#### Twiddle Tuning

Twiddle is implemented to work on the moving car. As a starting point the values from the previous step are used. The convergence criterion is that the sum of all the params' deltas need to be smaller than 0.00001. The horizon for RMSE calculation is set to 20 time steps.

The values after the twiddle:
P: 0.21651; I: 0.004; D: 2.5


#### Results

After the manual tuning the car was already able to drive one full lap without getting off the road. Even though there was an improvement in terms of error after twiddle tuning, it wasn't significant.

#### Problems

The oscillations in the cars trajectory are still present.

The behavior is asymmetrical due to the steering offset, and I-gain wasn't able to completely fix this issue.

---
### Simulation

The car is able to follow the road for a full lap, without any tire leaving the drivable part of the road.

