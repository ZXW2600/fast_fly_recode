import numpy as np
from fast_fly.utils.logger import getLogger
from fast_fly.planner.trajectory_generation import MinimimSnapTraj

class TimeAllocater:
    def __init__(self, waypoints: list[tuple[float, float, float,] | list[float]], loop: bool) -> None:
        self.logger = getLogger("TimeAllocater")
        self.N_sample_list: list[int] = []
        self.time_list: list[list[float]] = []
        self.dt_list: list[list[float]] = []
        self.flag_loop = loop
        self.waypoints = waypoints

    def allocate_sample(self) -> None:
        pass

    def allocate_time(self):
        pass

    def getSamples(self):
        return self.N_sample_list

    def getDts(self):
        return self.dt_list

    def getTimes(self):
        """get sample start time list for each segment

        Returns:
            list[float]: _description_
        """
        return self.time_list

    def calculate_init_state(self, init_state: np.ndarray, init_time: float):
        pass

    def getNumSamples(self):
        return sum(self.N_sample_list)

    def getNumSegments(self):
        return len(self.N_sample_list)


class AvrTimeAllocater(TimeAllocater):
    def __init__(self, waypoints: list[tuple[float, float, float] | list[float]], loop: bool,
                 avr_speed: float, length_per_sample: float) -> None:
        super().__init__(waypoints, loop)
        self.avr_speed = avr_speed
        self.l_per_n = length_per_sample
        self.allocate_sample()
        self.allocate_time()
        self.logger.debug(f"gates {self.waypoints}")

    def allocate_sample(self) -> None:
        waypoints = self.waypoints
        for i in range(len(waypoints)-1):
            p1 = np.array(waypoints[i])
            p2 = np.array(waypoints[i+1])
            l = np.linalg.norm(p2-p1)
            self.N_sample_list.append(int(l/self.l_per_n))
        self.logger.debug(f"sample list:{self.N_sample_list}")

    def allocate_time(self):
        last_segment_end_t = 0

        for i in range(len(self.N_sample_list)):
            self.time_list.append(
                [last_segment_end_t+j*self.l_per_n/self.avr_speed for j in range(self.N_sample_list[i])])
            self.dt_list.append(
                [self.l_per_n/self.avr_speed for j in range(self.N_sample_list[i])])
            last_segment_end_t += self.N_sample_list[i] * \
                self.l_per_n/self.avr_speed

class MinSnapAllocater(TimeAllocater):
    def __init__(self, waypoints: list[tuple[float, float, float] | list[float]], loop: bool,
                 avr_speed: float, length_per_sample: float) -> None:
        super().__init__(waypoints, loop)
        self.avr_speed = avr_speed
        self.l_per_n = length_per_sample
        self.allocate_sample()
        self.allocate_time()
        self.logger.debug(f"gates {self.waypoints}")
    
    