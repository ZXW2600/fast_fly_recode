import casadi as ca
# Quaternion Multiplication
def quat_mult(q1, q2):
    ans = ca.vertcat(q2[0, :] * q1[0, :] - q2[1, :] * q1[1, :] - q2[2, :] * q1[2, :] - q2[3, :] * q1[3, :],  # noqa
                     q2[0, :] * q1[1, :] + q2[1, :] * q1[0, :] - q2[2, :] * q1[3, :] + q2[3, :] * q1[2, :],  # noqa
                     q2[0, :] * q1[2, :] + q2[2, :] * q1[0, :] + q2[1, :] * q1[3, :] - q2[3, :] * q1[1, :],  # noqa
                     q2[0, :] * q1[3, :] - q2[1, :] * q1[2, :] + q2[2, :] * q1[1, :] + q2[3, :] * q1[0, :])  # noqa
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1, v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)),
                    ca.vertcat(q1[0, :], -q1[1, :], -q1[2, :], -q1[3, :]))
    return ca.vertcat(ans[1, :], ans[2, :], ans[3, :])  # to covert to 3x1 vec
