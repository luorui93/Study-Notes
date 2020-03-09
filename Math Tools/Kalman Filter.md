>Related resources: \
>[The Unscented Kalman Filter for Nonlinear Estimation](../Library/unscented.pdf)
>
>Also refer to the *Unscented Kalman Filter* section in python filter jupyter notebook for implementation example.


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
 >\mathbf{e} = z_k - \bar x_{k+1}\\
 >K = \bar P_kH^T(H\bar P_kH^T + R_k) ^{-1}\\
 >x_{k+1} = \bar x_{k+1} + K\mathbf{e}\\
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

A comparison between Kalman Filter and UKF

$$\begin{array}{l|l}
\textrm{Kalman Filter} & \textrm{Unscented Kalman Filter} \\
\hline 
& \boldsymbol{\mathcal Y} = f(\boldsymbol\chi) \\
\mathbf{\bar x} = \mathbf{Fx} & 
\mathbf{\bar x} = \sum w^m\boldsymbol{\mathcal Y}  \\
\mathbf{\bar P} = \mathbf{FPF}^\mathsf T+\mathbf Q  & 
\mathbf{\bar P} = \sum w^c({\boldsymbol{\mathcal Y} - \mathbf{\bar x})(\boldsymbol{\mathcal Y} - \mathbf{\bar x})^\mathsf T}+\mathbf Q \\
\hline 
& \boldsymbol{\mathcal Z} =  h(\boldsymbol{\mathcal{Y}}) \\
& \boldsymbol\mu_z = \sum w^m\boldsymbol{\mathcal{Z}} \\
\mathbf e = \mathbf z - \mathbf{Hx} &
\mathbf e = \mathbf z - \boldsymbol\mu_z \\
\mathbf S = \mathbf{H\bar PH}^\mathsf{T} + \mathbf R & 
\mathbf P_z = \sum w^c{(\boldsymbol{\mathcal Z}-\boldsymbol\mu_z)(\boldsymbol{\mathcal{Z}}-\boldsymbol\mu_z)^\mathsf{T}} + \mathbf R \\ 
\mathbf K = \mathbf{\bar PH}^\mathsf T \mathbf S^{-1} &
\mathbf K = \left[\sum w^c(\boldsymbol{\mathcal Y}-\bar{\mathbf x})(\boldsymbol{\mathcal{Z}}-\boldsymbol\mu_z)^\mathsf{T}\right] \mathbf P_z^{-1} \\
\mathbf x = \mathbf{\bar x} + \mathbf{Ke} & \mathbf x = \mathbf{\bar x} + \mathbf{Ke}\\
\mathbf P = (\mathbf{I}-\mathbf{KH})\mathbf{\bar P} & \mathbf P = \bar{\mathbf P} - \mathbf{KP_z}\mathbf{K}^\mathsf{T}
\end{array}$$


### Sigma Point Computation

The first sigma point is the mean of the input. This is the sigma point displayed in the center of the ellipses in the diagram above. We will call this $\boldsymbol{\chi}_0$.

$$ \mathcal{X}_0 = \mu$$

For notational convenience we define $\lambda = \alpha^2(L+\kappa)-L$, where $n$ is the dimension of $\mathbf x$. The remaining sigma points are computed as

$$ 
\boldsymbol{\chi}_i = \begin{cases}
\mu + \left[ \sqrt{(L+\lambda)\Sigma}\right ]_{i}& \text{for i=1 .. L} \\
\mu - \left[ \sqrt{(L+\lambda)\Sigma}\right]_{i-L} &\text{for i=(L+1) .. 2L }\end{cases}
$$
The $i$ subscript chooses the i$^{th}$ column vector of the matrix.

In other words, we scale the covariance matrix by a constant, take the square root of it, and ensure symmetry by both adding and subtracting it from the mean. We will discuss how you take the square root of a matrix later.

### Weight Computation

This formulation uses one set of weights for the means, and another set for the covariance. The weights for the mean of $\mathcal{X}_0$ is computed as

$$W^m_0 = \frac{\lambda}{L+\lambda}$$

The weight for the covariance of $\mathcal{X}_0$ is

$$W^c_0 = \frac{\lambda}{L+\lambda} + 1 -\alpha^2 + \beta$$

The weights for the rest of the sigma points $\boldsymbol{\chi}_1 ... \boldsymbol{\chi}_{2n}$ are the same for the mean and covariance. They are

$$W^m_i = W^c_i = \frac{1}{2(L+\lambda)}\;\;\;i=1..2L$$


**Important note:** Ordinarily these weights do not sum to one.


### Reasonable Choices for the Parameters

$\beta=2$ is a good choice for Gaussian problems, $\kappa=3-L$ is a good choice for $\kappa$, and $0 \le \alpha \le 1$ is an appropriate choice for $\alpha$, where a larger value for $\alpha$ spreads the sigma points further from the mean.