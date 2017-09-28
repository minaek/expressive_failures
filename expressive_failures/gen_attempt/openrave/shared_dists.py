import numpy as np

"""
Distance Function 1: negative squared L2-norm
"""
def dist_l2(v1, v2):
    return np.linalg.norm(v1 - v2)**2

"""
Distance Function 2: dot product with projections
"""
def dist_proj(v1, v2, k=3):
    # v2 should refer to difference between q_d and q_s
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)

    if v1_norm < 1e-10:
        dist = 0
    else:
        init_proj = np.dot(v1,v2)/v2_norm
        cos_theta = np.dot(v1,v2)/(v1_norm*v2_norm)
        dist = init_proj*(cos_theta**(k-1))
    return -dist
