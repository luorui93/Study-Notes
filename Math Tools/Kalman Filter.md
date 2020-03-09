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

 >### Predict
 >$$
 >\bar x_{k+1} = Fx_k + Bu \\
 >\bar P_{k+1} = FP_kF^T + Q_k
 >$$
 >where $Bu$ is modeling the effect of control inputs, $\bar x_{k+1}$ is the >prediction of the next state based on process model only.
>
 >### Update
 >$$
 >e = z_k - \bar x_{k+1}\\
 >K = \bar P_kH^T(H\bar P_kH^T + R_k) ^{-1}\\
 >x_{k+1} = \bar x_{k+1} + Ke\\
 >P_{k+1} = (I - KH)\bar P
 >$$
 >where $e$ is the residual, $K$ is the **Kalman gain**.  
 
 This is the implementation of a multivariate Kalman filter prediction. We only used matrix multiplication and addition here thus it can only describe a linear system.

## Extended Kalman Filter (EKF)

EKF is the extended version of Kalman filter aimed to estimate nonlinear system state. The nonlinear system model is estimated by a first-order linearization (oftentimes a Jacobian matrix).



## Unscented Kalman Filter (UKF)
UKF samples $2L+1$ points from the data and uses Unscented Transform (UT) to represent the original data, where $L$ is the dimension of state.
The number of sampled points is much less than *Monte Carlo* method. These sampled points are called **sigma points**.  
Sigma points with different weights are passed through the true nonlinear system/function to generate outputs. The mean and covariance of are estimated based on the outputs.  
UKF can achieve third order accuracy (Taylor series) for Gaussian input and second order accuracy for non-Gaussian input.  


