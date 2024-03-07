from typing import Optional
import numpy as np
import casadi as ca
from fast_fly.planner.time_allocate import TimeAllocater
from fast_fly.model.quadcopter import QuadrotorModel, State
from fast_fly.utils.logger import getLogger, Logger
from fast_fly.utils.math import QuadraticError


class OptimalVariables(Logger):
    def __init__(self, name, default_init=None, default_ub=None, default_lb=None, param=False) -> None:
        super().__init__(name)
        self.state_list = []
        self.state_init_list = []
        self.state_result_list: Optional[ca.DM] = None
        self.state_ub_list = []
        self.state_lb_list = []
        self.state_dim = 0
        self.state_len = 0
        self.name = name
        self.default_init = default_init
        self.default_ub = default_ub
        self.default_lb = default_lb
        self.param = param

    def addState(self, symbo, init=None, ub=None, lb=None):
        if init is None:
            if self.default_init is None:
                self.error(
                    "init value is not set, and default value is not set")
                raise ValueError("init value is not set")
            init = self.default_init
        if not self.param:
            if ub is None:
                if self.default_ub is None:
                    self.error(
                        "upper bound value is not set, and default value is not set")
                    raise ValueError("upper bound value is not set")
                ub = self.default_ub
            if lb is None:
                if self.default_lb is None:
                    self.error(
                        "lower bound value is not set, and default value is not set")
                    raise ValueError("lower bound value is not set")
                lb = self.default_lb

        self.state_dim = symbo.size1()
        self.state_len += 1
        # self.error(f"state dim:{self.state_dim}")
        self.state_list.append(symbo)
        self.state_init_list.append(init)
        self.state_ub_list.append(ub)
        self.state_lb_list.append(lb)

    def getSymbolicState(self):
        return self.state_list

    def getInitState(self):
        return self.state_init_list

    def getUpBound(self):
        return self.state_ub_list

    def getLowBound(self):
        return self.state_lb_list

    def setResult(self, result: ca.DM):
        self.state_result_list = result.reshape((-1, self.state_dim))
        self.info(f"result shape:{self.state_result_list.shape} init shape {
            len(self.state_init_list)} {len(self.state_init_list[0])}")

    def getResult(self):
        return self.state_result_list.toarray()

    def state(self, index: int):
        return self.state_list[index]

    def __getitem__(self, index: int):
        return self.state_list[index]

    def len(self):
        return len(self.state_list)


class OptimalConstraints(Logger):
    def __init__(self, name, default_ub=None, default_lb=None) -> None:
        super().__init__(f"OptCst_{name}")
        self.g_list = []
        self.g_ub_list = []
        self.g_lb_list = []

        self.default_ub = default_ub
        self.default_lb = default_lb

        self.name = name

    def addConstraint(self, constraint, ub=None, lb=None):
        if ub is None:
            if self.default_ub is None:
                self.error("upper bound value is not set")
                raise ValueError("upper bound value is not set")
            ub = self.default_ub
        if lb is None:
            if self.default_lb is None:
                self.error("lower bound value is not set")
                raise ValueError("lower bound value is not set")
            lb = self.default_lb
        self.g_list.append(constraint)
        self.g_ub_list.append(ub)
        self.g_lb_list.append(lb)


class OptimalQuadraticObject(Logger):
    def __init__(self, name, weight) -> None:
        super().__init__(f"OPQobj{name}")
        self.name = name
        self.weight = weight
        self.obj = 0

    def addToObjective(self, error):
        self.obj += error.T@self.weight@error

    def setObject(self, error):
        self.obj = error.T@self.weight@error


class OptimalObject(Logger):
    def __init__(self, name, weight) -> None:
        super().__init__(name)
        self.weight = weight
        self.obj = 0

    def addToObjective(self, error):
        self.obj += self.weight*error

    def setObject(self, error):
        self.obj = self.weight@error


def flatten(xss):
    if not isinstance(xss, list):
        return [xss]
    elif not isinstance(xss[0], list):
        return xss
    return [x for xs in xss for x in xs]


class OptimalProblem(Logger):
    def __init__(self, name) -> None:
        self.name = name
        super().__init__(f"OP {name}")
        self.opt_var_dict: dict[OptimalVariables] = {}
        self.opt_cst_dict: dict[OptimalConstraints] = {}
        self.opt_obj_dict: dict[OptimalObject] = {}
        self.opt_param_dict: dict[OptimalVariables] = {}
        self.opt_option = {}

        self.result_var_slice = {}

    def addOptimalVariables(self, vars: OptimalVariables):
        self.opt_var_dict[vars.name] = vars

    def addParameters(self, params: OptimalVariables):
        self.opt_param_dict[params.name] = params

    def addConstraints(self, constraints: OptimalConstraints):
        self.opt_cst_dict[constraints.name] = constraints

    def addObjective(self, obj: OptimalQuadraticObject):
        self.opt_obj_dict[obj.name] = obj

    def setOptimalOption(self, option: dict):
        self.opt_option = option

    def solve(self, warmup_problem=None):
        self.info("start solve")
        self.x = []
        self.x0 = []
        self.lbx = []
        self.ubx = []
        for key, vars in self.opt_var_dict.items():
            slice_start = len(self.x0)
            vars: OptimalVariables
            self.x += vars.getSymbolicState()
            self.x0 += flatten(vars.getInitState())
            self.lbx += flatten(vars.getLowBound())
            self.ubx += flatten(vars.getUpBound())
            slice_end = len(self.x0)
            self.result_var_slice[key] = slice(slice_start, slice_end)

        self.p = []
        self.p0 = []
        for key, parms in self.opt_param_dict.items():
            parms: OptimalVariables
            self.p += parms.getSymbolicState()
            print(parms.getInitState())

            self.p0 += flatten(parms.getInitState())

        self.p0 = self.p0

        self.f = 0
        for key, obj in self.opt_obj_dict.items():
            obj: OptimalQuadraticObject
            self.f += obj.obj

        self.g = []
        self.lbg = []
        self.ubg = []
        for key, cst in self.opt_cst_dict.items():
            cst: OptimalConstraints
            self.g += cst.g_list
            self.lbg += cst.g_lb_list
            self.ubg += cst.g_ub_list

        self.lbg = np.array(self.lbg).flatten()
        self.ubg = np.array(self.ubg).flatten()

        self.nlp_equation = {
            "x": ca.veccat(*self.x),
            "f": self.f,
            "g": ca.veccat(*self.g),
            "p": ca.veccat(*self.p)
        }

        self.info("x shape: "+str(self.nlp_equation["x"].shape))
        self.info("g shape: "+str(self.nlp_equation["g"].shape))
        self.info("p shape: "+str(self.nlp_equation["p"].shape))

        self.nlp_solver = ca.nlpsol(
            "opt", "ipopt", self.nlp_equation, self.opt_option)
        self.info("x0 shape: "+str(len(self.x0)))
        self.info("p0 shape: "+str(len(self.p0)))
        self.info("lbx shape: "+str(len(self.lbx)))
        self.info("ubx shape: "+str(len(self.ubx)))
        res = self.nlp_solver(
            x0=self.x0,
            p=self.p0,
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
        )

        for key, vars in self.opt_var_dict.items():
            x = res["x"]
            self.info(f"result x shape:{x.shape} slice {
                self.result_var_slice[key]}")

            vars.setResult(res["x"][self.result_var_slice[key]])
        self.info("start solve finshed")


class DronePathPlanner(Logger):
    def __init__(self, quadcopter: QuadrotorModel, time_allocater: TimeAllocater, waypoints: list[tuple[float, float, float,] | list[float]], loop: bool, ) -> None:
        super().__init__("DronePathPlanner")
        self.quadcopter = quadcopter
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
            "dynamic", default_lb=[0] * self.dim_sym_x, default_ub=[0] * self.dim_sym_x)
        self.opt_dynamic_obj = OptimalQuadraticObject(
            "dynamic", ca.diag([1]*self.dim_sym_x))

        self.waypoint_tol_constraints = OptimalConstraints(
            "waypoint_tol", default_lb=0, default_ub=self.waypoint_tol)

        self.waypoint_tol_obj = OptimalQuadraticObject(
            "waypoint_tol", weight=ca.diag([1, 1, 1]))

        self.minco_obj = OptimalQuadraticObject(
            "minco", weight=ca.diag([0.01, 0.01, 0.01]))

        self.min_t_obj = OptimalObject("min_t", weight=1)

    def setup(self):
        if self.loop:
            self.setup_loop()
        else:
            self.setup_nonloop()

    def setup_loop(self):
        self.error("To be imple")
        raise Exception("To be imple")

    def func_per_segment(self, segment_index, segment_samples,
                         current_segment_start_index, next_segment_start_index,
                         segment_start, segment_end,
                         flag_init, flag_end):
        pass

    def function_per_node(self, node_index, init_flag, end_flag):
        pass

    def walk_through_path(self, func_per_segment, function_per_node):
        # walk through all the segments
        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            current_segment_start = next_segment_start
            next_segment_start += segment_samples
            segment_start = self.waypoints[segment_index]
            segment_end = self.waypoints[segment_index+1]

            # add all state to the problem
            func_per_segment(self, segment_index, segment_samples,
                             segment_start, segment_end, 
                             current_segment_start,next_segment_start,
                             current_segment_start == 0, next_segment_start == self.num_nodes)
            for index_path in range(current_segment_start, next_segment_start):

                function_per_node(self,index_path, index_path == 0,
                                  index_path == self.num_nodes-1)

    def setup_nonloop(self):
        self.info("setup non loop problem")
        self.debug("init pos to first waypoint")

        # check if data length match
        if len(self.waypoints) != len(self.sample_list)+1:
            self.error(f"waypoints and sample list not match, "
                       f"waypoints:{len(self.waypoints)} sample list:{len(self.sample_list)}")

        def add_dT_perseg(self: DronePathPlanner, segment_index, segment_samples,
                          current_segment_start_index, next_segment_start_index,
                          segment_start, segment_end,
                         flag_init, flag_end):  # noqa
            self.opt_dT.addState(self.sym_vec_DTs[segment_index])
            self.debug(f"segment start {current_segment_start_index} :{segment_start} "
                       f"end {current_segment_start_index} :{segment_end}")
            if flag_init:
                self.debug("add init pos to problem")
            if flag_end:
                self.debug("add end pos to problem")

        def add_state_input_per_node(self: DronePathPlanner, node_index, init_flag, end_flag):
            self.opt_State.addState(self.sym_mat_x[:, node_index])
            self.opt_Input.addState(self.sym_mat_u[:, node_index])
            self.debug(f"add state {node_index} to problem")

        self.walk_through_path(
            func_per_segment=add_dT_perseg, function_per_node=add_state_input_per_node
        )
        # walk through all the segments
        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            current_segment_start = next_segment_start
            next_segment_start += segment_samples
            segment_start = self.waypoints[segment_index]
            segment_end = self.waypoints[segment_index+1]

            self.debug(f"segment start {current_segment_start}:{segment_start} "
                       f"end {next_segment_start}:{segment_end}")
            if current_segment_start == 0:
                flag_init = True
                self.debug("add init pos to problem")
            if next_segment_start == self.num_nodes:
                flag_end = True
                self.debug("add end pos to problem")

            # add all state to the problem
            self.opt_dT.addState(self.sym_vec_DTs[segment_index])
            for index_path in range(current_segment_start, next_segment_start):
                self.opt_State.addState(self.sym_mat_x[:, index_path])
                self.opt_Input.addState(self.sym_mat_u[:, index_path])
                self.debug(f"add state {index_path} to problem")

        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            current_segment_start = next_segment_start
            next_segment_start += segment_samples
            segment_start = self.waypoints[segment_index]
            segment_end = self.waypoints[segment_index+1]
            dT = self.opt_dT[segment_index]
            # add dynamic constrain to the problem
            for index_path in range(current_segment_start, next_segment_start):
                if index_path == self.num_nodes-1:
                    self.debug(f"last nodes {
                               index_path}, skip dynamic constrain")
                    continue
                state_now = self.opt_State[index_path]
                state_next = self.opt_State[index_path+1]
                input_now = self.opt_Input[index_path]
                self.debug(f"add dynamic constrain from state {
                           index_path} to {index_path+1}")

        for segment_index, segment_samples in enumerate(self.sample_list):
            self.opt_dT.addState(self.sym_vec_DTs[segment_index])
            for index_per_segment in range(segment_samples):
                index_path = next_segment_start+index_per_segment
                self.opt_State.addState(self.sym_mat_x[:, index_path])
                self.opt_Input.addState(self.sym_mat_u[:, index_path])

            self.waypoint_pos.addState(
                self.sym_mat_waypoints_p[:, segment_index], init=self.waypoints[segment_index])
            self.error(self.waypoints[segment_index])
            next_segment_start += segment_samples
        # self.info(f"param:{self.waypoint_pos.getInitState()}")
        next_segment_start = 0
        for segment_index, segment_samples in enumerate(self.sample_list):
            for index_per_segment in range(segment_samples):
                index_path = next_segment_start+index_per_segment
                # min time objective
                self.min_t_obj.addToObjective(self.opt_dT[segment_index])

                # min angle speed objective as minco
                self.minco_obj.addToObjective(
                    error=self.opt_State[index_path][State.X_omega])

                if index_path+1 < self.num_nodes:
                    self.opt_dynamic_constraints.addConstraint(QuadraticError(
                        self.f_dynamics(self.opt_State[index_path], self.opt_Input[index_path], self.opt_dT[segment_index])-self.opt_State[index_path+1])
                    )
                    self.opt_dynamic_obj.addToObjective(
                        self.f_dynamics(self.opt_State[index_path], self.opt_Input[index_path],
                                        self.opt_dT[segment_index])-self.opt_State[index_path+1]
                    )

            self.waypoint_tol_obj.addToObjective(
                error=self.opt_State[next_segment_start][State.X_pos]-self.waypoint_pos[segment_index])
            self.waypoint_tol_constraints.addConstraint(
                constraint=QuadraticError(
                    error=self.opt_State[next_segment_start][State.X_pos]-self.waypoint_pos[segment_index]),
            )
            next_segment_start += segment_samples
        self.info("setup problem finished")

    def getWarmupProblem(self):
        self.info("setup warmup problem")
        warmup = OptimalProblem(name="warmup")
        warmup.addOptimalVariables(self.opt_State)
        warmup.addOptimalVariables(self.opt_Input)
        warmup.addParameters(self.opt_dT)
        warmup.addParameters(self.waypoint_pos)
        warmup.addObjective(self.minco_obj)
        warmup.addObjective(self.opt_dynamic_obj)
        warmup.addObjective(self.waypoint_tol_obj)
        self.info("setup warmup problem finished")

        return warmup

    def getOptimalProblem(self):
        self.info("setup optimal problem")

        opt = OptimalProblem(name="opt")
        opt.addOptimalVariables(self.opt_State)
        opt.addOptimalVariables(self.opt_Input)
        opt.addOptimalVariables(self.opt_dT)

        opt.addParameters(self.waypoint_pos)

        opt.addConstraints(self.opt_dynamic_constraints)
        opt.addConstraints(self.waypoint_tol_constraints)

        # opt.addObjective(self.minco_obj)
        # opt.addObjective(self.opt_dynamic_obj)
        opt.addObjective(self.min_t_obj)
        self.info("setup optimal problem finished")

        return opt
