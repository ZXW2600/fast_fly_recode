from casados_integrator import CasadosIntegrator
from acados_template import AcadosSim
from casadi import *
from datetime import datetime

def create_casados_integrator(model, integrator_opts, dt=0.1, use_cython=True, integrator_type="IRK", code_reuse=False):
    sim = AcadosSim()
    sim.model = model

    # set simulation time
    sim.solver_options.T = dt

    # set options

    sim.solver_options.sens_forw = True
    sim.solver_options.sens_algebraic = False
    sim.solver_options.sens_hess = True
    sim.solver_options.sens_adj = True
    # if integrator_opts["type"] == "implicit":
    if integrator_type == "GNSF":
        sim.solver_options.integrator_type = "GNSF"
        sim.solver_options.sens_hess = False
    elif integrator_type == "RK4":
        sim.solver_options.integrator_type = "ERK"
    else:
        sim.solver_options.integrator_type = "IRK"
    # elif integrator_opts["type"] == "explicit":
    #     sim.solver_options.integrator_type = "ERK"
    # else:
    #     raise Exception("integrator_opts['type'] must be explicit or implicit.")

    if integrator_type == "RK4":
        sim.solver_options.num_stages = 4
        sim.solver_options.num_steps = 1
    else:
        sim.solver_options.num_stages = integrator_opts["num_stages"]
        sim.solver_options.num_steps = integrator_opts["num_steps"]

    sim.solver_options.newton_iter = integrator_opts["newton_iter"]

    if integrator_opts["collocation_scheme"] == "radau":
        sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"
    elif integrator_opts["collocation_scheme"] == "legendre":
        sim.solver_options.collocation_type = "GAUSS_LEGENDRE"
    else:
        raise Exception(
            "integrator_opts['collocation_scheme'] must be radau or legendre."
        )

    sim.solver_options.newton_tol = (
        integrator_opts["tol"] / integrator_opts["num_steps"]
    )
    sim.code_export_directory = f'c_generated_code_{model.name}_{sim.solver_options.integrator_type}'

    # create
    casados_integrator = CasadosIntegrator(sim, use_cython=use_cython, code_reuse=code_reuse)

    # if integrator_opts['type'] == 'implicit':
    #     casados_integrator.acados_integrator.set('xdot', np.zeros(casados_integrator.nx))

    return casados_integrator