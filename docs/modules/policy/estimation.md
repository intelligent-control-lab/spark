# Estimation

The `estimation.filtering` hierarchy supplies linear, steady-state, extended,
and unscented Kalman estimators. The `estimation.parameter` hierarchy supplies
recursive least-squares and gradient parameter estimators.

For a linear observation model, the correction uses

```{math}
K=P^-C^T(CP^-C^T+R)^{-1},\qquad
\hat x=\hat x^-+K(y-C\hat x^-).
```
