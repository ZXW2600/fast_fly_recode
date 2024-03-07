from fast_fly.planner.optimal_helper import OptimalConstraints, OptimalQuadraticObject, OptimalVariables, OptimalObject, OptimalProblem
import casadi as ca
import numpy as np


def test_opt_problem():

    opt_problem = OptimalProblem("test")

    sym_x = ca.SX.sym("x", 3, 10)
    sym_y = ca.SX.sym("y", 2, 10)

    opt_var_x = OptimalVariables("x", default_init=[
        0, 0, 0], default_lb=[-10, -10, -10], default_ub=[10, 10, 10])
    opt_var_y = OptimalVariables("y", default_init=[
        0, 0], default_lb=[-10, -10], default_ub=[10000, 10000])

    opt_constr1 = OptimalConstraints("cst1", default_lb=-1, default_ub=1)
    opt_constr2 = OptimalConstraints("cst2", default_lb=-1, default_ub=1)

    obj = OptimalQuadraticObject("test_obj", weight=1)

    for i in range(10):
        opt_var_x.addState(sym_x[:, i])
        opt_var_y.addState(sym_y[:, i])

        opt_constr1.addConstraint(
            opt_var_x[i], ub=[i, i*2, i*3], lb=[0.5*i, i, i*1.5])
        opt_constr2.addConstraint(
            opt_var_y[i], ub=[i*10, i*10], lb=[i*10, i*10])
        
        obj.addToObjective(opt_var_x[i])

    opt_problem.addOptimalVariables(opt_var_x)
    opt_problem.addOptimalVariables(opt_var_y)
    opt_problem.addConstraints(opt_constr1)
    opt_problem.addConstraints(opt_constr2)
    opt_problem.addObjective(obj)
    # opt_problem.setOptimalOption({"max_iter": 1000})
    opt_problem.solve()
    print(opt_var_x.result(0))
    print(opt_var_y.result(0))


test_opt_problem()
