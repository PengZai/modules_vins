import numpy as np

def invert_se3(T):
    """Invert a 4x4 transformation matrix (SE(3)): [R t; 0 1]"""
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

# Input matrix
T = np.array([
    [0.0238743541600432, -0.999707744440396, 0.00360642510766516, 0.138922870923538],
    [-0.00736968896588375, -0.00378431903190059, -0.999965147452649, -0.177101909101325],
    [0.999687515506770, 0.0238486947027063, -0.00745791352160211, -0.126685267545513],
    [0.0,0.0,0.0,1.0]
])

T_inv = invert_se3(T)
np.set_printoptions(precision=8, suppress=True)
print("Inverse matrix:\n", T_inv)
