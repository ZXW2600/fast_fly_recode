from typing import Optional
import numpy as np
import casadi as ca
from fast_fly.utils.logger import Logger
from fast_fly.utils.math import QuadraticError

from fast_fly.utils.utils import flatten


class OptimalVariables(Logger):
    def __init__(self, name, default_init=None, default_ub=None, default_lb=None, param=False,headers=None) -> None:
        super().__init__(name)
        self.state_list = []
        self.state_init_list = []
        self.state_result_list: Optional[np.ndarray] = None
        self.state_ub_list = []
        self.state_lb_list = []
        self.state_dim = 0
        self.state_len = 0
        self.name = name
        self.default_init = default_init
        self.default_ub = default_ub
        self.default_lb = default_lb
        self.param = param
        self.headers=headers


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
        self.debug(f"add state {symbo} init {init} ub {ub} lb {lb}")

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
        self.debug(f"set result {self.state_result_list}")
        # self.info(f"result shape:{self.state_result_list.shape} init shape {
        #     len(self.state_init_list)} {len(self.state_init_list[0])}")

    def getResult(self):
        return self.state_result_list
        # return self.state_result_list.tolist()

    def result(self, i):
        return self.state_result_list[i, :]

    def state(self, index: int):
        return self.state_list[index]

    def __getitem__(self, index: int):
        return self.state_list[index]

    def len(self):
        return len(self.state_list)

    def __len__(self):
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
        super().__init__(f"OPObj{name}")
        self.name = name
        self.weight = weight
        self.obj = 0

    def addToObjective(self, error):
        self.obj += self.weight*error

    def setObject(self, error):
        self.obj = self.weight@error


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
            if len(vars) == 0:
                self.exception(f"var {key} is empty")

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
            self.g += flatten(cst.g_list)
            self.lbg += flatten(cst.g_lb_list)
            self.ubg += flatten(cst.g_ub_list)

        # self.lbg = np.array(self.lbg).flatten()
        # self.ubg = np.array(self.ubg).flatten()

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
            x: ca.DM = res["x"]
            self.debug(f"{key}: {x.toarray()[self.result_var_slice[key]]}")
            self.debug(f"result x shape:{x.shape} slice {self.result_var_slice[key]}")  # noqa

            vars.setResult(x.toarray()[self.result_var_slice[key]])
        self.info("start solve finshed")
