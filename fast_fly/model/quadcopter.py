from typing import Any, Optional
import casadi as ca
import numpy as np
import yaml
from fast_fly.utils.logger import getLogger

# Quaternion Multiplication


def quat_mult(q1, q2):
    ans = ca.vertcat(q2[0, :] * q1[0, :] - q2[1, :] * q1[1, :] - q2[2, :] * q1[2, :] - q2[3, :] * q1[3, :],
                     q2[0, :] * q1[1, :] + q2[1, :] * q1[0, :] -
                     q2[2, :] * q1[3, :] + q2[3, :] * q1[2, :],
                     q2[0, :] * q1[2, :] + q2[2, :] * q1[0, :] +
                     q2[1, :] * q1[3, :] - q2[3, :] * q1[1, :],
                     q2[0, :] * q1[3, :] - q2[1, :] * q1[2, :] + q2[2, :] * q1[1, :] + q2[3, :] * q1[0, :])
    return ans

# Quaternion-Vector Rotation


def rotate_quat(q1, v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(
        q1[0, :], -q1[1, :], -q1[2, :], -q1[3, :]))
    return ca.vertcat(ans[1, :], ans[2, :], ans[3, :])  # to covert to 3x1 vec


class ConfigReader:

    def __init__(self, yaml_path) -> None:
        self.logger=getLogger("ConfigReader")
        with open(yaml_path, 'r') as f:
            self.yaml_cfg = yaml.load(f, Loader=yaml.FullLoader)
        self.logger.info(f"load params from file {yaml_path}")

    def load_param(self, param_name, default: Optional[Any] = None):
        if param_name in self.yaml_cfg:
            value = self.yaml_cfg[param_name]
            self.logger.info(f"Read value from file {param_name} : {default}")

        elif default is not None:
            value = default
            self.logger.warn(f"Using default value for {param_name} : {default}")
        else:
            self.logger.error(f"Parameter {param_name} not found in config file")
            raise ValueError(
                f"Parameter {param_name} not found in config file")
        return value


class QuadrotorModel(object):
    def __init__(self, cfg_f):
        self.logger=getLogger("QuadrotorModel")
        self.logger.info(f"read prams from file {cfg_f}")
        config = ConfigReader(cfg_f)
        
        # load param from file
        self._m = config.load_param("mass", default=1.0)
        self._arm_l = config.load_param("arm_length", default=0.23)
        self._c_tau = config.load_param("torque_coeff", default=0.0133)

        self._G = config.load_param("gravity", default=9.81)

        self._J = np.diag(config.load_param(
            "inertia", default=[0.01, 0.01, 0.02]))
        self._J_inv = np.linalg.inv(self._J)
        self._D = np.diag(config.load_param(
            "drag_coeff", default=[0.1, 0.1, 0.1]))

        self._v_xy_max = config.load_param("v_xy_max", default=ca.inf)
        self._v_z_max = config.load_param("v_z_max", default=ca.inf)
        self._omega_xy_max = config.load_param('omega_xy_max', default=ca.inf)
        self._omega_z_max = config.load_param("omega_z_max", default=ca.inf)
        self._T_max = config.load_param("thrust_max", default=ca.inf)
        self._T_min = config.load_param("thrust_min", default=0)


        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1, -1, -1, -1,
                      -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1, 1, 1, 1,
                      self._omega_xy_max, self._omega_xy_max, self._omega_z_max]

        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]
