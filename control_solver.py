# 아래와 같은 다이내믹시스템을 정의함
# 1)  x^{'}_1 = x_2 라 함. 즉 x_1를 시간 t에 대해서 한 번 미분한 것을 x_2라 함
# 이 때 x_1은 position이라 하고 x_2는 velocity라 할 수 있음
# 2) x^{'}_2 = u 라 함. 여기서 u는 시스템으로의 입력을 의미. 즉, 입력은 velocity의 변화량임. u는 가속도 또는 'force divided by mass'라고 해석할 수 있음
# 3) 이 때 z(t) = x_1(t) 라는 tracking objective를 설정. 우리의 목표는 시간 t가 흘러감에 따라 z(t)를 r=1이라는 목표치로 최대한 가까이 가게 만드는 것임
# 4) 시스템에 가해지는 입력값 u는 시간에 따라 달라지는 함수로 u(t)로 쓸 수 있음. 이 때 u(t)의 절대값은 0.1보다 작거나 같음
# 아래와 같은 추가적인 제약조건을 가정
# i) x_1(t)는 모든 시간 t에 대해서 1.01보다 작거나 같음
# ii) x_2(t)의 절대값은 0.2보다 작거나 같음
# Define the following dynamic system:
# 1) Let x1' = x2. That is, x1 is the derivative with respect to time t. Here, x1 is referred to as the position, and x2 as the velocity.
# 2) Let x2' = u. Here, u represents the input to the system.
# 3) At this point, define the tracking objective z(t) = x1(t).
# 4) The input u applied to the system varies with time.
# Assume the following additional constraints:
# i) x1(t) is less than or equal to 1.01 for all times t.
# ii) The absolute value of x2(t) is less than or equal to 0.2.

import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cx

#>>>> parameter setting
# Time span for the simulation
t_span = [0, 50]
num_steps = 1500
t_eval = np.linspace(*t_span, num_steps)
delta_t = (t_span[1] - t_span[0]) / num_steps
u_values = []  # List to store u values
x2_values = []  # List to store velocity (x2) values
x2 = 0  # Initial velocity
x1_values = []  # List to store position (x1) values
x1 = 0  # Initial position
target_x1 = 1  # Target position
#<<<< parameter setting

#>>>> controller
def control_mpc(x1, x2):
    # MPC parameters
    prediction_horizon = 10
    r = target_x1  # Target position

    # Define the system matrices
    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    nx, nu = B.shape

    # Define the state and control input variables for MPC
    x = cx.Variable((nx, prediction_horizon + 1))
    # x_next = Variable((nx, prediction_horizon + 1))
    u = cx.Variable((nu, prediction_horizon))
    # Define the cost function and constraints
    cost = 0
    constraints = [x[:,0] == np.array([x1, x2])]  # Initial state constraint

    for t in range(prediction_horizon):
        cost += cx.sum_squares(x[:,t] - r) + cx.sum_squares(u[:,t])  # Cost function
        constraints += [x[:,t+1] == x[:,t] + A @ x[:,t] + B @ u[:,t],  # System dynamics
                        cx.abs(u[:,t]) <= 0.1,  # Input constraints
                        cx.abs(x[1,t]) <= 0.2]  # State constraints for x2

    # Formulate and solve the MPC problem
    problem = cx.Problem(cx.Minimize(cost), constraints)
    problem.solve()

    # Extract the optimal control input
    optimal_u = u[:,0].value

    # Print the optimal control input
    # print(type(optimal_u))
    # print(optimal_u)
    return optimal_u[0]

# PID state initialization (assuming this part of the code is executed only once at the start)
integral_error = 0  # Integral of the error
previous_error = 0  # Previous error for derivative term
# delta_t_pid = 1  # Time step, adjust based on your simulation step size
def control_pid(x1, x2):
    global integral_error, previous_error  # Use the global keyword to modify the variables outside the function
    # PID parameters
    Kp = 7  # Proportional gain 시스템이 더 빨리 목표에 도달하도록 하려면 Kp를 증가
    Ki = 0.05  # Integral gain 시스템의 안정성을 높이려면 Ki를 증가
    Kd = 0.1  # Derivative gain 오버슈트를 줄이려면 Kd를 증가

    # Define the constraints
    max_u = 0.1
    min_u = -0.1
    max_x1 = target_x1 * 1.01
    max_x2 = 0.2
    min_x2 = -0.2

    # Implementing a PID control strategy
    error = target_x1 - x1

    # Proportional term
    P = Kp * error

    # Integral term (discrete approximation)
    integral_error += error * delta_t
    I = Ki * integral_error

    # Derivative term (discrete approximation)
    D = Kd * (error - previous_error) / delta_t
    previous_error = error  # Update previous error for the next step

    # Compute control input u based on PID controller
    u = P + I + D
    u = np.clip(u, min_u, max_u)  # Ensure u is within the constraints

    # Ensure x1 and x2 constraints are not violated
    # (Optional: add logic to handle constraints for x1 and x2 if needed)
    # Adjust control input u based on x1 and x2 constraints (indirectly)
    # If x1 is close to its maximum value, decrease u
    if x1 >= max_x1:
        u = min(u, 0)

    # If x2 is close to its velocity limits, adjust u to decelerate
    if x2 >= max_x2:
        u = min(u, min_u)
    elif x2 <= min_x2:
        u = max(u, max_u)
    return u

# Define the control strategy (max-min control)
def control_maxmin(x1, x2):
    # Define the constraints
    max_u = 0.1
    min_u = -0.1
    max_x1 = 1.01
    max_x2 = 0.2
    min_x2 = -0.2

    # Implementing a simple control strategy: Adjust u to steer x1 towards the target
    # while respecting the constraints on x1, x2, and u.
    target_x1 = 1  # Target position
    error = target_x1 - x1

    # Simple proportional controller with saturation
    u = np.clip(error, min_u, max_u)

    # update x2 based on u

    # Ensure x1 and x2 constraints are not violated
    # if x1 >= max_x1 or abs(x2) >= max_x2:
    #     u = 0  # Stop the system if constraints are violated
    return u
#<<<< controller

#>>>> system dynamics
# Initial conditions (x1 = position, x2 = velocity)
x0 = [0, 0]
u_system = 0
for step in range(num_steps):
    # if step >= 500:
    #     target_x1 = 1.5
    u_values.append(u_system)

    # Integrate u to update velocity
    x2 += u_system * delta_t
    x2_values.append(x2)

    # Integrate x2 to update position (x1)
    x1 += x2 * delta_t
    x1_values.append(x1)
    
    # feedback control
    control_strategy = control_mpc
    res = control_strategy(x1, x2)
    if res is None:
        pass
    else:
        u_system = res
#<<<< system dynamics

#>>>> plot
# Plot all three: Input (u), Velocity (x2), and Position (x1) values
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t_eval, u_values, label='u (Input)', color='green')
plt.title('Input (u) over Time')
plt.xlabel('Time (s)')
plt.ylabel('Input (u)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t_eval, x2_values, label='x2 (Velocity)', color='blue')
plt.title('Velocity (x2) over Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (x2)')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t_eval, x1_values, label='x1 (Position)', color='purple')
plt.title('Position (x1) over Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (x1)')
plt.grid(True)

plt.tight_layout()
plt.show()
#<<<< plot
