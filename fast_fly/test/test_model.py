from fast_fly.model.quadcopter import QuadrotorModel
import logging
logging.basicConfig(level=logging.INFO)

def test_model():
    test_model=QuadrotorModel("/home/zxw2600/Workspace_Disk/traj_opt_ws/cpc/fast_fly_recode/fast_fly/config/quad_real.yaml")
    
test_model()
while True:
    pass