import csv
from typing import Optional
import numpy as np
import casadi as ca
from fast_fly.planner.time_allocate import TimeAllocater
from fast_fly.model.quadcopter import QuadrotorModel, State
from fast_fly.utils.logger import getLogger, Logger
from fast_fly.utils.math import QuadraticError
from fast_fly.planner.optimal_helper import OptimalConstraints, OptimalObject, OptimalProblem, OptimalQuadraticObject, OptimalVariables


class dtConverter:
    def __init__(self, Dt: np.ndarray, time_samples: list):
        self.Dt = Dt
        self.time_samples = time_samples
        self.time_list = []
        self.dt_list = []
        if isinstance(Dt, list):
            Dt = np.array(Dt)
        if Dt.shape[0] != len(time_samples):
            raise ValueError("Dt shape not match time samples")

        for i, n_sample in enumerate(time_samples):
            dT_i = Dt[i]
            for j in range(n_sample):
                self.dt_list.append(dT_i)

        self.dt = np.array(self.dt_list)

        self.time_list = np.cumsum(self.dt, axis=0)

    def getdt(self):
        return self.dt_list

    def getTime(self):
        return self.time_list


class Trajectory:
    def __init__(self, state: OptimalVariables, input: OptimalVariables, dT: OptimalVariables, time_samples: list) -> None:
        self.state = state.getResult()
        self.input = input.getResult()
        self.dT = dT.getResult()
        self.dt_list = []
        if self.dT is None:
            self.dT = dT.getInitState()
        for i, n_sample in enumerate(time_samples):
            dT_i = self.dT[i]
            for j in range(n_sample):
                self.dt_list.append(dT_i)

    def save(self, csv_path):
        csv_writer = csv.writer(open(csv_path, "w"))
        csv_writer.writerow(["t", "dt", "p_x", "p_y", "p_z",
                             "v_x", "v_y", "v_z",
                             "q_w", "q_x", "q_y", "q_z",
                             "w_x", "w_y", "w_z", "u_1", "u_2", "u_3", "u_4"])
        time = 0
        for state, input, dt in zip(self.state, self.input, self.dt_list):
            time += dt
            csv_writer.writerow([time, dt]+state.tolist()+input.tolist())


class DronePathPlanner(Logger):
    def __init__(self, quadcopter: QuadrotorModel, time_allocater: TimeAllocater, waypoints: list[tuple[float, float, float,] | list[float]], loop: bool, ) -> None:
        super().__init__("DronePathPlanner")
        self.quadcopter = quadcopter
        # self.quadcopter.set_up_cacados()

        self.time_allocater = time_allocater
        self.waypoints = waypoints
        self.debug(f"waypoints: {waypoints}")
        self.loop = loop
        self.waypoint_tol = 0.5

        # 动力学方程
        # ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])
        self.f_dynamics = self.quadcopter.d_dynamics_dt()
        self.dim_sym_x = self.f_dynamics.size1_in(0)
        self.dim_sym_u = self.f_dynamics.size1_in(1)

        self.num_nodes = self.time_allocater.getNumSamples()
        self.num_waypoints = self.time_allocater.getNumSegments()
        self.sample_list = self.time_allocater.getSamples()

        self.sym_vec_dts = ca.SX.sym("dt", self.num_nodes)
        self.sym_vec_DTs = ca.SX.sym("dT", self.num_waypoints)
        self.sym_mat_x = ca.SX.sym("Xs", self.dim_sym_x, self.num_nodes)
        self.sym_mat_u = ca.SX.sym("Us", self.dim_sym_u, self.num_nodes)
        self.sym_mat_waypoints_p = ca.SX.sym("WPs_p", 3, self.num_waypoints)
        self.sym_init_pos = ca.SX.sym("init_p", 3)
        # self.sym_scalar_waypoints_tol = ca.SX.sym("wp_tol")

        self.opt_State = OptimalVariables("state", default_init=self.quadcopter._X_init,
                                          default_ub=self.quadcopter._X_ub, default_lb=self.quadcopter._X_lb)
        self.param_State = OptimalVariables(
            "state_p", default_init=self.quadcopter._X_init, param=True)
        self.opt_Input = OptimalVariables("input", default_init=[0.0, 0.0, -9.81, 0.0],
                                          default_ub=self.quadcopter._U_ub, default_lb=self.quadcopter._U_lb)
        self.opt_dT = OptimalVariables(
            "dT", default_init=0.1, default_ub=0.5, default_lb=0.01)

        self.waypoint_pos = OptimalVariables(
            "waypoint_pos", default_init=self.waypoints, param=True)

        self.opt_dynamic_constraints = OptimalConstraints(
            "dynamic", default_lb=[0]*self.dim_sym_x, default_ub=[0]*self.dim_sym_x)
        self.opt_dynamic_obj = OptimalQuadraticObject(
            "dynamic", ca.diag([1]*self.dim_sym_x))

        self.opt_waypoint_tol_constraints = OptimalConstraints(
            "waypoint_tol", default_lb=[0]*3, default_ub=[self.waypoint_tol**2]*3)

        self.opt_waypoint_tol_obj = OptimalQuadraticObject(
            "waypoint_tol", weight=ca.diag([1, 1, 1]))

        self.opt_minco_obj = OptimalQuadraticObject(
            "minco", weight=ca.diag([0.01, 0.01, 0.01]))

        self.opt_min_t_obj = OptimalObject("min_t", weight=1)

    def setup(self):
        if self.loop:
            self.setup_loop()
        else:
            self.setup_nonloop()

    def setup_loop(self):
        self.info("setup loop problem")

        # check if data length match
        if len(self.waypoints) != len(self.sample_list)+1:
            self.error(f"waypoints and sample list not match, "
                       f"waypoints:{len(self.waypoints)} sample list:{len(self.sample_list)}")

        # walk through all the segments to add state and input to the problem
        for index_node in range(self.num_nodes):
            # first state and last state don't need optimize
            self.opt_State.addState(self.sym_mat_x[:, index_node])
            self.opt_Input.addState(self.sym_mat_u[:, index_node])

        for index_segment in range(self.num_waypoints):
            self.opt_dT.addState(self.sym_vec_DTs[index_segment])
            self.waypoint_pos.addState(
                self.sym_mat_waypoints_p[:, index_segment], init=self.waypoints[index_segment])

        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            current_segment_start = next_segment_start
            next_segment_start += segment_samples
            segment_start = self.waypoints[segment_index]
            segment_end = self.waypoints[segment_index+1]
            dT = self.opt_dT[segment_index]
            waypoint_start_state = self.sym_mat_x[:, current_segment_start]
            self.opt_waypoint_tol_obj.addToObjective(
                error=waypoint_start_state[State.X_pos]-segment_start
            )
            self.opt_waypoint_tol_constraints.addConstraint(
                waypoint_start_state[State.X_pos]-segment_start
            )
            # add dynamic constrain to the problem
            for index_path in range(current_segment_start, next_segment_start):
                state_now = self.sym_mat_x[:, index_path]
                input_now = self.sym_mat_u[:, index_path]

                # construst min time objective
                self.opt_min_t_obj.addToObjective(dT)
                # construst min control objective
                self.opt_minco_obj.addToObjective(error=state_now[State.X_omega])  # noqa

                if index_path == self.num_nodes-1:
                    self.debug(f"last nodes {index_path}, skip dynamic constrain")  # noqa
                    state_next = self.sym_mat_x[:, 0]
                    self.debug(f"add dynamic constrain from state {index_path} to {0}")  # noqa

                else:
                    state_next = self.sym_mat_x[:, index_path+1]
                    self.debug(f"add dynamic constrain from state {index_path} to {index_path+1}")  # noqa

                self.opt_dynamic_constraints.addConstraint(
                    self.f_dynamics(state_now, input_now, dT)-state_next
                )
                self.opt_dynamic_obj.addToObjective(
                    self.f_dynamics(state_now, input_now, dT)-state_next
                )

        self.info("setup problem finished")

    def setup_nonloop(self):
        self.info("setup non loop problem")
        self.debug("init pos to first waypoint")

        init_state = self.quadcopter._X_init
        init_state[State.X_pos] = self.waypoints[0]
        end_state = self.quadcopter._X_init
        end_state[State.X_pos] = self.waypoints[-1]

        # check if data length match
        if len(self.waypoints) != len(self.sample_list)+1:
            self.error(f"waypoints and sample list not match, "
                       f"waypoints:{len(self.waypoints)} sample list:{len(self.sample_list)}")

        # walk through all the segments to add state and input to the problem
        for index_node in range(self.num_nodes):
            # first state and last state don't need optimize
            if index_node == 0:
                self.param_State.addState(
                    self.sym_mat_x[:, index_node], init=init_state)
            elif index_node == self.num_nodes-1:
                self.param_State.addState(
                    self.sym_mat_x[:, index_node], init=end_state)

            else:
                self.opt_State.addState(self.sym_mat_x[:, index_node])

            self.opt_Input.addState(self.sym_mat_u[:, index_node])

        for index_segment in range(self.num_waypoints):
            self.opt_dT.addState(self.sym_vec_DTs[index_segment])
            self.waypoint_pos.addState(
                self.sym_mat_waypoints_p[:, index_segment], init=self.waypoints[index_segment])

        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            current_segment_start = next_segment_start
            next_segment_start += segment_samples
            segment_start = self.waypoints[segment_index]
            segment_end = self.waypoints[segment_index+1]
            dT = self.opt_dT[segment_index]

            # add dynamic constrain to the problem
            for index_path in range(current_segment_start, next_segment_start):
                state_now = self.sym_mat_x[:, index_path]
                input_now = self.sym_mat_u[:, index_path]

                # construst min time objective
                self.opt_min_t_obj.addToObjective(dT)
                # construst min control objective
                self.opt_minco_obj.addToObjective(error=state_now[State.X_omega])  # noqa

                if index_path == self.num_nodes-1:
                    self.debug(f"last nodes {index_path}, skip dynamic constrain")  # noqa
                    continue
                state_next = self.sym_mat_x[:, index_path+1]

                self.debug(f"add dynamic constrain from state {index_path} to {index_path+1}")  # noqa
                self.opt_dynamic_constraints.addConstraint(
                    self.f_dynamics(state_now, input_now, dT)-state_next
                )
                self.opt_dynamic_obj.addToObjective(
                    self.f_dynamics(state_now, input_now, dT)-state_next
                )

        self.info("setup problem finished")

    def getWarmupProblem(self):
        self.info("setup warmup problem")
        warmup = OptimalProblem(name="warmup")
        warmup.addOptimalVariables(self.opt_State)
        warmup.addOptimalVariables(self.opt_Input)
        warmup.addParameters(self.opt_dT)
        warmup.addParameters(self.waypoint_pos)
        warmup.addParameters(self.param_State)
        warmup.addObjective(self.opt_minco_obj)
        warmup.addObjective(self.opt_dynamic_obj)
        warmup.addObjective(self.opt_waypoint_tol_obj)
        self.info("setup warmup problem finished")
        return warmup

    def saveWarmup(self, csv_path):
        self.warmup_traj = Trajectory(
            self.opt_State, self.opt_Input, self.opt_dT, self.sample_list)
        self.warmup_traj.save(csv_path)

    def saveOptimal(self, csv_path):
        self.opt_traj = Trajectory(
            self.opt_State, self.opt_Input, self.opt_dT, self.sample_list)
        self.opt_traj.save(csv_path)

    def getOptimalProblem(self):
        self.info("setup optimal problem")

        opt = OptimalProblem(name="opt", use_last_result=True)
        opt.addOptimalVariables(self.opt_State)
        opt.addOptimalVariables(self.opt_Input)
        opt.addOptimalVariables(self.opt_dT)

        opt.addParameters(self.waypoint_pos)

        opt.addConstraints(self.opt_dynamic_constraints)
        opt.addConstraints(self.opt_waypoint_tol_constraints)

        # opt.addObjective(self.minco_obj)
        # opt.addObjective(self.opt_dynamic_obj)
        opt.addObjective(self.opt_min_t_obj)
        self.info("setup optimal problem finished")

        return opt
