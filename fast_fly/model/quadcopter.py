from typing import Any, Optional
import casadi as ca
import numpy as np
import yaml
from fast_fly.utils.logger import getLogger
from fast_fly.utils.geometery import quat_mult, rotate_quat
from fast_fly.utils.math import constrain
from fast_fly.utils.integral import EulerIntegral, RK4
from fast_fly.utils.yaml_helper import ConfigReader
from fast_fly.utils.cacados import create_casados_integrator
from acados_template import AcadosModel


class State:
    X_pos = slice(0, 3)
    X_vel = slice(3, 6)
    X_quat = slice(6, 10)
    X_omega = slice(10, 13)
    U = slice(0, 4)


class QuadrotorModel(object):
    def __init__(self, cfg_f):
        self.logger = getLogger("QuadrotorModel")
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
        self._X_init = [0, 0, 0,  # noqa
                                    0, 0, 0,  # noqa
                                    1, 0, 0, 0,  # noqa
                                    0, 0, 0]  # noqa
        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]

    #
    #   T1    T3
    #     \  /
    #      \/
    #      /\
    #     /  \
    #   T4    T2
    #
    #

    def f_dynamics(self):
        """dynamics function for quadrotor

        Returns:
            _type_: casdia.Function
        """
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')  # noqa
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        T1, T2, T3, T4 = ca.SX.sym('T1'), ca.SX.sym('T2'), ca.SX.sym('T3'), ca.SX.sym('T4')  # noqa

        taux = self._arm_l/np.sqrt(2)*(T1+T4-T2-T3)
        tauy = self._arm_l/np.sqrt(2)*(T1+T3-T2-T4)
        tauz = self._c_tau*(T3+T4-T1-T2)
        thrust = (T1+T2+T3+T4)

        tau = ca.veccat(taux, tauy, tauz)
        w = ca.veccat(wx, wy, wz)
        w_dot = self._J_inv@(tau - ca.cross(w, self._J@w))

        fdrag = rotate_quat(ca.veccat(qw, qx, qy, qz), self._D @
                            rotate_quat(ca.veccat(qw, -qx, -qy, -qz), ca.veccat(vx, vy, vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * (-thrust/self._m) - fdrag[0],
            2 * (qy * qz - qw * qx) * (-thrust/self._m) - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) *
            (-thrust/self._m) + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy),
            w_dot[0],
            w_dot[1],
            w_dot[2]
        )

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz,
                       wx, wy, wz)
        U = ca.vertcat(T1, T2, T3, T4)

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx

    def d_dynamics(self, dt):
        """discrete time dynamics function for quadrotor with constant dt

        Args:
            dt (float): _description_

        Returns:
            Function: _description_
        """
        f = self.f_dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))

        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l

        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])

    def d_dynamics_dt(self):
        """discrete time dynamics function for quadrotor with varying dt

        Returns:
            Function: _description_
        """
        f = self.f_dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')

        # X1 = RK4(f, X0, U, dt, 1)
        X1 = EulerIntegral(f, X0, U, dt, 1)

        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l

        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

    def set_up_cacados(self):
        self.ca_model = self.cacados_model()
        integrator_opts = {
            "type": "implicit",
            "collocation_scheme": "radau",
            "num_stages": 6,
            "num_steps": 3,
            "newton_iter": 10,
            "tol": 1e-6,
        }
        self.casados_integrator = create_casados_integrator(
            self.ca_model, integrator_opts, dt=0.1)

    def cacados_model(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')  # noqa
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        T1, T2, T3, T4 = ca.SX.sym('T1'), ca.SX.sym('T2'), ca.SX.sym('T3'), ca.SX.sym('T4')  # noqa

        taux = self._arm_l/np.sqrt(2)*(T1+T4-T2-T3)
        tauy = self._arm_l/np.sqrt(2)*(T1+T3-T2-T4)
        tauz = self._c_tau*(T3+T4-T1-T2)
        thrust = (T1+T2+T3+T4)

        tau = ca.veccat(taux, tauy, tauz)
        w = ca.veccat(wx, wy, wz)
        w_dot = self._J_inv@(tau - ca.cross(w, self._J@w))

        fdrag = rotate_quat(ca.veccat(qw, qx, qy, qz), self._D @
                            rotate_quat(ca.veccat(qw, -qx, -qy, -qz), ca.veccat(vx, vy, vz)))

        f_expl = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * (-thrust/self._m) - fdrag[0],
            2 * (qy * qz - qw * qx) * (-thrust/self._m) - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) *
            (-thrust/self._m) + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy),
            w_dot[0],
            w_dot[1],
            w_dot[2]
        )

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz,
                       wx, wy, wz)
        U = ca.vertcat(T1, T2, T3, T4)

        X_dot_sym = ca.SX.sym('X_dot', f_expl.shape[0])
        model = AcadosModel()
        model.f_impl_expr = X_dot_sym-f_expl
        model.f_expl_expr = f_expl
        model.x = X
        model.u = U
        model.xdot = X_dot_sym
        model.name = 'quadcopter'
        return model

    def cacados_discrete_constrain(self, X0, U0, dt):
        self.casados_integrator(X0, U0)
