from fast_fly.utils.yaml_helper import ConfigReader
from fast_fly.utils.logger import getLogger

class Mission:
    def __init__(self,loop:bool) -> None:
        self.loop = loop

    def add_waypoint(self, pos, yaw,pos_tol):
        pass
    
    def add_init_pose(self, pos, yaw):
        pass

    def add_aim_pose(self, pos, yaw):
        pass


class GateConfig:
    def __init__(self, yaml_path) -> None:
        self.logger = getLogger("GateConfig")
        self.logger.info(f"read prams from file {yaml_path}")
        self.config = ConfigReader(yaml_path)

        self.waypoints = self.config.load_param("waypoints")
        self.pos_list = [w[:3] for w in self.waypoints]
        self.yaw_list = [w[3] for w in self.waypoints]

        self.distance_tol = self.config.load_param("distance_tol")
        self.loop = self.config.load_param("loop")

        if len(self.pos_list) != len(self.yaw_list):
            self.logger.error("pos and yaw list length not equal")
            raise ValueError

        self.n_gate = len(self.pos_list)
