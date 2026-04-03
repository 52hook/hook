import numpy as np
import math
from scipy.optimize import minimize
from matplotlib import pyplot as plt
import time

# Load the data
data = np.load('data_w_real_steer_70.npy')
X = data[:, :-2]
# X[1,:] = -X[1,:] # flip y axis
U = data[:, -2:]
dt = 0.05

# Vehicle dynamics model
def vehicle_dynamics(x, u, p):
    kf, kr, lf, lr, m, Iz = p
    Lk = lf * kf - lr * kr
    x_next = np.zeros_like(x)
    x_next[0] = x[0] + (x[3] * math.cos(x[2]) - x[4] * math.sin(x[2])) * dt
    x_next[1] = x[1] + (x[4] * math.cos(x[2]) + x[3] * math.sin(x[2])) * dt
    x_next[2] = x[2] + x[5] * dt
    x_next[3] = x[3] + dt * u[0]
    x_next[4] = (m * x[3] * x[4] + dt * Lk * x[5] - dt * kf * u[1] * x[3] - dt * m * (x[3] ** 2) * x[5]) / (m * x[3] - dt * (kf + kr))
    x_next[5] = (Iz * x[3] * x[5] + dt * Lk * x[4] - dt * lf * kf * u[1] * x[3]) / (Iz * x[3] - dt * (lf ** 2 * kf + lr ** 2 * kr))
    return x_next

# Residual function
def residuals(p, X, U):
    res = np.zeros(X.shape[0] - 1)
    for i in range(X.shape[0] - 1):
        x_next = vehicle_dynamics(X[i], U[i], p)
        res[i] = np.linalg.norm(X[i + 1] - x_next)
    return np.sum(res)

# Initial parameters
# p0 = np.array([-128916, -85944, 1.08, 1.81, 1412, 1536.7])
p0 = np.array([-120000, -90000, 1.50, 1.39, 1700, 2700])
# p0 = np.array([-4.248e+04, -9.548e+04,  1.339e+00,  1.568e+00,  1.614e+03, 1.538e+03])
# kf = -120232.69753427865
# kr = -72253.17399326632
# lf = 1.3694773400676927
# lr = 1.500522659932285
# m = 1614.008272336204
# Iz = 1826.9103372232407
# p0 = np.array([kf, kr, lf, lr, m, Iz])

# ===== 约束条件（SLSQP 约定: fun(p) >= 0）=====
# 轴距约束: lf + lr = 2.89m（CARLA Model 3）
con_wb = {'type': 'eq', 'fun': lambda p: p[2] + p[3] - 2.89}
# 侧偏刚度为负: kf <= 0, kr <= 0
con_kf = {'type': 'ineq', 'fun': lambda p: -p[0]}          # -kf >= 0 → kf <= 0
con_kr = {'type': 'ineq', 'fun': lambda p: -p[1]}          # -kr >= 0 → kr <= 0
# 前后轴距重心距不低于 0.8m（物理约束，防止退化）
con_lf_lo = {'type': 'ineq', 'fun': lambda p: p[2] - 0.8}  # lf >= 0.8m
con_lr_lo = {'type': 'ineq', 'fun': lambda p: p[3] - 0.8}  # lr >= 0.8m
# 车重: 1000 <= m <= 2200 kg
con_m_lo = {'type': 'ineq', 'fun': lambda p: p[4] - 1000}   # m >= 1000
con_m_hi = {'type': 'ineq', 'fun': lambda p: 2200 - p[4]}   # m <= 2200
# 转动惯量: 500 <= Iz <= 5000 kg·m²
con_Iz_lo = {'type': 'ineq', 'fun': lambda p: p[5] - 500}   # Iz >= 500
con_Iz_hi = {'type': 'ineq', 'fun': lambda p: 5000 - p[5]}  # Iz <= 5000

# Perform optimization
tick = time.time()
result = minimize(residuals, p0, args=(X, U), method='SLSQP', options={'maxiter': 500},
                 constraints=[con_wb, con_kf, con_kr, con_lf_lo, con_lr_lo,
                              con_m_lo, con_m_hi, con_Iz_lo, con_Iz_hi])
tock = time.time()
print(f'Optimization time: {tock - tick} seconds')

# Estimated parameters
print(result)
kf_est, kr_est, lf_est, lr_est, m_est, Iz_est = result.x
print('Estimated parameters:')
print(f'kf = {kf_est}')
print(f'kr = {kr_est}')
print(f'lf = {lf_est}')
print(f'lr = {lr_est}')
print(f'm = {m_est}')
print(f'Iz = {Iz_est}')

# Test the accuracy of the estimated parameters
X_est = np.zeros_like(X)
X_est[0] = X[0]
# receiding horizon test
for i in range(X.shape[0] - 1):
    X_est[i + 1] = vehicle_dynamics(X_est[i], U[i], result.x)
print(f'Average error: {np.mean(np.linalg.norm(X - X_est, axis=1))}')
print(f'Maximum error: {np.max(np.linalg.norm(X - X_est, axis=1))}')

fig = plt.figure()
sub1 = fig.add_subplot(221)
plt.plot(X[:,0]-X_est[:,0], label='Accumulated Error of x')
plt.plot(X[:,1]-X_est[:,1], label='Accumulated Error of y')
plt.legend()
sub2 = fig.add_subplot(222)
plt.plot(X[:,0], X[:,1], label='Ground truth of route')
plt.plot(X_est[:,0], X_est[:,1], label='Estimated of route (Accumulated)')
plt.legend()
# test using pure estimated data
for i in range(X.shape[0] - 1):
    # if i % 10 == 0:
    #     X_est[i + 1] = vehicle_dynamics(X[i], U[i], result.x)
    # else:
    #     X_est[i + 1] = vehicle_dynamics(X_est[i], U[i], result.x)
    X_est[i + 1] = vehicle_dynamics(X[i], U[i], result.x)
print(f'Average error: {np.mean(np.linalg.norm(X - X_est, axis=1))}')
print(f'Maximum error: {np.max(np.linalg.norm(X - X_est, axis=1))}')
sub3 = fig.add_subplot(223)
plt.plot(X[:,0]-X_est[:,0], label='Sliding Window-based Error of x')
plt.plot(X[:,1]-X_est[:,1], label='Sliding Window-based Error of y')
plt.legend()
sub4 = fig.add_subplot(224)
plt.plot(X[:,0], X[:,1], label='Ground truth of route')
plt.plot(X_est[:,0], X_est[:,1], label='Estimated of route (Sliding Window)')
plt.legend()
plt.show()

data_test = np.load("data_w_real_steer_70_test.npy")
X_test = data_test[:, 0:6]
U_test = data_test[:, 6:8]
X_est_test = np.zeros_like(X_test)
X_est_test[0] = X_test[0]
for i in range(X_test.shape[0] - 1):
    X_est_test[i + 1] = vehicle_dynamics(X_est_test[i], U_test[i], result.x)
print(f'Average error: {np.mean(np.linalg.norm(X_test - X_est_test, axis=1))}')
print(f'Maximum error: {np.max(np.linalg.norm(X_test - X_est_test, axis=1))}')

fig = plt.figure()
sub1 = fig.add_subplot(221)
plt.plot(X_test[:,0]-X_est_test[:,0], label='Accumulated Error of x')
plt.plot(X_test[:,1]-X_est_test[:,1], label='Accumulated Error of y')
plt.legend()
sub2 = fig.add_subplot(222)
plt.plot(X_test[:,0], X_test[:,1], label='Ground truth of route')
plt.plot(X_est_test[:,0], X_est_test[:,1], label='Estimated route (Accumulated)')
plt.legend()
# test using pure estimated data
for i in range(X_test.shape[0] - 1):
    # if i % 10 == 0:
    #     X_est[i + 1] = vehicle_dynamics(X[i], U[i], result.x)
    # else:
    #     X_est[i + 1] = vehicle_dynamics(X_est[i], U[i], result.x)
    X_est_test[i + 1] = vehicle_dynamics(X_test[i], U_test[i], result.x)
print(f'Average error: {np.mean(np.linalg.norm(X_test - X_est_test, axis=1))}')
print(f'Maximum error: {np.max(np.linalg.norm(X_test - X_est_test, axis=1))}')
sub3 = fig.add_subplot(223)
plt.plot(X_test[:,0]-X_est_test[:,0], label='Sliding Window-based Error of x')
plt.plot(X_test[:,1]-X_est_test[:,1], label='Sliding Window-based Error of y')
plt.legend()
sub4 = fig.add_subplot(224)
plt.plot(X_test[:,0], X_test[:,1], label='Ground truth of route')
plt.plot(X_est_test[:,0], X_est_test[:,1], label='Estimated of route (Sliding Window)')
plt.legend()
plt.show()


