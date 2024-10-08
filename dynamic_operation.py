import numpy as np


from scipy.integrate import solve_ivp

from Constraints import mq, g, Jx, Jy, Jz, K, A, B # Import the constants

# Initial state
initial_state = np.zeros(12)

# Current state (u, px, v, py, w, pz, phidot, phi, thetadot,theta, phsidot, phsi)
current_state = np.zeros(12)
u, px, v, py, w, pz, p,phi,q,theta,r,psi= np.array(current_state)

# Target state (pxtarget, pytarget, pztarget are the onces only changing)
target_state = np.zeros(12)


# Define time span for the simulation
time_span= [0, 10]

# Control law (feedback control based on state error)
control = -K @ (current_state - target_state)


def non_linear(current_state, target_state, control):
    #Computes the non-linear dynamics of the drone system.
    
    #time: current time
    #current_state: current state of the system [u, px, v, py, w, pz, thetadot, theta, phidot, phi, phydot, phy]
    #target_state: target state of the system
    #K: feedback gain matrix for the control system
    
   #Returns (dx).
    
 

 # Adding gravity effect to the control
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
    dx = np.array([u_dot,u,v_dot,v,w_dot,w, p_dot,p, q_dot,q,r_dot,r])

    return dx


def linear(current_state, A, B, control, target_state):
    """
    Computes the linear dynamics of the drone system.
    
    time: current time
    current_state: current state of the system (x)
    A: system matrix for linear dynamics
    B: control matrix for linear dynamics
    K: feedback gain matrix for the control system
    target_state: target state of the system
    
    Returns dx
    """
    # Linear state update: dx = Ax + B * control
    dx = A @ current_state + B @ control
    
    return dx

# Solve ODEs
non_linear_derivative = solve_ivp(non_linear, time_span, current_state, args=[target_state])
linear_derivative = solve_ivp(linear, time_span, current_state, args=[target_state])