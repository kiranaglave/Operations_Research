import numpy as np
def main():
    
# Initial state
initial_state = np.zeros(12)

# Current state (u, px, v, py, w, pz, phidot, phi, thetadot,theta, phsidot, phsi)
current_state = np.zeros(12)
u, px, v, py, w, pz, p,phi,q,theta,r,psi= np.array(current_state)

# Target state (pxtarget, pytarget, pztarget, pt_target)
target_state = np.array([0, px_target, 0, py_target, 0, pz_target, 0, 0, 0, 0, 0, 0])

time_span= [0, 10]


