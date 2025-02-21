# Cart-Pole Optimal Control Assignment

### Technical Report: Controller Behavior Analysis


https://github.com/user-attachments/assets/f6af1e70-379f-43c1-b67b-171227c624de




## 1. Introduction

This report summarizes the analysis and tuning of a cart-pole control system, focusing on baseline performance, parameter effects, and disturbance response.

## 2. Baseline Performance

Initial System Behavior

Default Parameters: Q = [1.0, 1.0, 10.0, 10.0], R = 0.1

#### Observations:

Setting "x" to 5 caused rapid cart movement and pole instability.

Adjusting "x_dot" to 0.1 increased balance time but caused rapid cart movement.

Increasing "x_dot" to 9 minimized position movement and stabilized the system.

#### Key Issues:

Over-aggressive control caused instability.

Low responsiveness to disturbances.

## 3. Parameter Effects

#### Q Matrix:

High weights on "x" destabilized the pole.

Increasing "x_dot" weight improved position control.

High "theta" weights caused oscillations.

Adjusting "theta_dot" weight impacted pole stability.

#### R Value:

Lower R (0.1) increased control aggressiveness, improving stabilization but amplifying oscillations.

#### Trade-offs:

High Q weights prioritized specific objectives but reduced overall stability.

Low R improved responsiveness at the cost of energy efficiency.

## 4. Final Parameters

#### Optimized parameters:

Q = [1.0, 9.0, 1.0, 9.0]

R = 0.1

#### Results:

Stable cart-pole balance with minimal position movement.

Smooth disturbance recovery and reduced oscillations.

## 5. Disturbance Response

Low-frequency disturbances: Smooth recovery with minor oscillations.

High-frequency disturbances: Stable balance with higher control effort.

## 6. Discussion and Recommendations

Systematic tuning improved performance significantly.

#### Lessons:

Small changes in Q and R values greatly affect behavior.

Balancing control aggressiveness and stability is critical.

#### Recommendations:

Automate parameter tuning.

Explore adaptive control for dynamic environments.

## 7. Conclusion

The optimized controller (Q = [1.0, 9.0, 1.0, 9.0], R = 0.1) achieved a well-balanced system with effective disturbance handling and minimal energy use.




## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
