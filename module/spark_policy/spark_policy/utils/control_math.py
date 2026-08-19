from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are


def matrix_arg(value, default: np.ndarray) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=float)
    arr = np.asarray(value, dtype=float)
    return np.diag(arr) if arr.ndim == 1 else arr


def vector_arg(value, length: int, default=0.0) -> np.ndarray:
    if value is None:
        return np.full(length, default, dtype=float)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != length:
        raise ValueError(f"Expected vector of length {length}, got {arr.size}.")
    return arr


def infinite_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, discrete: bool = True):
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    if discrete:
        P = solve_discrete_are(A, B, Q, R)
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    else:
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.solve(R, B.T @ P)
    return K, P


def finite_horizon_lqr(
    A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, S: np.ndarray, horizon: int
):
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    S = np.asarray(S, dtype=float)
    horizon = int(horizon)
    nx, nu = B.shape
    P = np.zeros((horizon + 1, nx, nx), dtype=float)
    K = np.zeros((horizon, nu, nx), dtype=float)
    P[horizon] = S
    for k in range(horizon - 1, -1, -1):
        gain_mat = R + B.T @ P[k + 1] @ B
        K[k] = np.linalg.solve(gain_mat, B.T @ P[k + 1] @ A)
        P[k] = Q + A.T @ P[k + 1] @ A - A.T @ P[k + 1] @ B @ K[k]
        P[k] = 0.5 * (P[k] + P[k].T)
    return K, P


def lift_dynamics(A: np.ndarray, B: np.ndarray, horizon: int):
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    nx, nu = B.shape
    bar_A = np.zeros(((horizon + 1) * nx, nx), dtype=float)
    bar_B = np.zeros(((horizon + 1) * nx, horizon * nu), dtype=float)
    bar_A[:nx] = np.eye(nx)
    for k in range(horizon):
        row = slice((k + 1) * nx, (k + 2) * nx)
        previous = slice(k * nx, (k + 1) * nx)
        bar_A[row] = A @ bar_A[previous]
        bar_B[row] = A @ bar_B[previous]
        bar_B[row, k * nu : (k + 1) * nu] = B
    return bar_A, bar_B
