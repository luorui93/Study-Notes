>Related resources: \
>[The Unscented Kalman Filter for Nonlinear Estimation](../Library/unscented.pdf)


## Kalman Filter
Kalman filter recursively estimates system states based on linear system model and measurements.

$$
x_{k+1} = F(x_k, Q_k)\\

z_k = H(x_k, R_k)
$$
where $x_k$ represents the system state, $z_k$ represents measurement, $F$ represents system dynamic model, $Q_k$ represents process noise,
 $H$ represents observation model, $R_k$ represents observation noise.

## Extended Kalman Filter (EKF)

EKF is the extended version of Kalman filter aimed to estimate nonlinear system state. The nonlinear system model is estimated by a first-order linearization (oftentimes a Jacobian matrix).



## Unscented Kalman Filter (UKF)


