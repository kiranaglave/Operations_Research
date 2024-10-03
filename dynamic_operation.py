import numpy as np


# Drone parameters
mq = 0.6  # mass of the quadrotor (Kg)
L = 0.2159  # arm length (m)
g = 9.81  # acceleration due to gravity (m/s^2)
ms = 0.410  # mass of the central sphere (Kg)
R = 0.0503513  # radius of the sphere (m)
mprop = 0.00311  # mass of the propeller (Kg)
mm = 0.036 + mprop  # mass of the motor + propeller (Kg)

# Moments of inertia
Jx = (2 * ms * R**2) / 5 + 2 * L**2 * mm
Jy = (2 * ms * R**2) / 5 + 2 * L**2 * mm
Jz = (2 * ms * R**2) / 5 + 4 * L**2 * mm

# Initial state
initial_state = np.zeros(12)

# Current state (u, px, v, py, w, pz, thetadot, theta, phidot, phi, phydot, phy)
current_state = np.zeros(12) 


# Target state (pxtarget, pytarget, pztarget, pt_target)
target_state = np.array([0, px_target, 0, py_target, 0, pz_target, 0, 0, 0, 0, 0, 0])


def solve_ip(time, initial_state):
    # Placeholder for the solver implementation
    pass
def non_linear(time, current_state, target_state, K):
    #Computes the non-linear dynamics of the drone system.
    
    #time: current time
    #current_state: current state of the system [u, px, v, py, w, pz, thetadot, theta, phidot, phi, phydot, phy]
    #target_state: target state of the system
    #K: feedback gain matrix for the control system
    
   #Returns the derivatives of the current state (dx).
   
# Extract current state variables using np.array
    u, u_dot, v, v_dot, w, w_dot, p, p_dot, q, q_dot,r, r_dot= np.array(current_state)

 # Control law (feedback control based on state error)
    control = -K @ (current_state - target_state)

 # Adding gravity effect to the control (thrust in the z-direction)
    control[0] += mq * g
    
    # Thrust force
    F = control[0]

  
    # Linear velocity dynamics (u_dot, v_dot, w_dot)
    u_dot = r*v - q*w - g*np.sin(p)
    v_dot = p*w - r*u + g*np.cos(p)*np.sin(q)
    w_dot = q*u - p*v + g*np.cos(p)*np.cos(q) - F/mq

    # Angular velocity dynamics (p_dot, q_dot, r_dot)
    p_dot = (Jy - Jz) / Jx * q * r + control[1] / Jx  # control[1] corresponds to torque τ_φ
    q_dot = (Jz - Jx) / Jy * p * r + control[2] / Jy  # control[2] corresponds to torque τ_θ
    r_dot = (Jx - Jy) / Jz * p * q + control[3] / Jz  # control[3] corresponds to torque τ_ψ

    # Pack the derivatives into a list to return
    dx = np.array([u_dot, u, v_dot, v, w_dot, w, p_dot, p, q_dot,q, r_dot, r])

    return dx


def linear(time, current_state, A, B, K, target_state):
    """
    Computes the linear dynamics of the drone system.
    
    time: current time
    current_state: current state of the system (x)
    A: system matrix for linear dynamics
    B: control matrix for linear dynamics
    K: feedback gain matrix for the control system
    target_state: target state of the system
    
    Returns the derivatives of the current state (dx).
    """
    
    # Control law (feedback control based on state error)
    control = -K @ (current_state - target_state)
    
    # Linear state update: dx = Ax + B * control
    dx = A @ current_state + B @ control
    
    return dx