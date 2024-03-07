import csv
from fast_fly.planner.cpc_dt import DronePathPlanner, OptimalVariables
from fast_fly.utils.gate import GateConfig
from fast_fly.model.quadcopter import QuadrotorModel
from fast_fly.planner.time_allocate import AvrTimeAllocater
import fast_fly
import os
package_path = os.path.dirname(fast_fly.__file__)

if not os.path.exists("result"):
    os.makedirs("result")
csv_file = "result/test_cpc.csv"

csv_writer = csv.writer(open(csv_file, "w"))
csv_writer.writerow(["time", "x", "y", "z", "vx", "vy",
                    "vz", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw"])


def test_planner():
    yaml_path = os.path.join(os.path.dirname(
        package_path), "fast_fly/config/gates_n7.yaml")
    gate_cfg = GateConfig(yaml_path)
    assert gate_cfg.n_gate == 7
    test_model = QuadrotorModel(
        "/home/zxw2600/Workspace_Disk/traj_opt_ws/cpc/fast_fly_recode/fast_fly/config/quad_real.yaml")
    time_alloc = AvrTimeAllocater(
        gate_cfg.pos_list, gate_cfg.loop, 20, 0.5)
    planner = DronePathPlanner(
        quadcopter=test_model,
        time_allocater=time_alloc,
        waypoints=gate_cfg.pos_list,
        loop=gate_cfg.loop,        
    )
    opt_option = {
            # "verbose": False,
            "ipopt.tol": 1e-4,
            # 'ipopt.acceptable_tol': 1e-3,
            "ipopt.max_iter": 1000,
            # 'ipopt.warm_start_init_point': 'yes',
            # 'ipopt.print_level': 0,
        }
    planner.setup()
    warmup = planner.getWarmupProblem()
    warmup.setOptimalOption(opt_option)
    warmup.solve()
    state:OptimalVariables=warmup.opt_var_dict["state"]
    for state_i in state.getResult():
        csv_writer.writerow(state_i)
    planner.getOptimalProblem()


test_planner()
