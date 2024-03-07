import cppimport.import_hook
import fast_fly
import sys, os

package_path = os.path.dirname(fast_fly.__file__)
sys.path += [package_path,package_path+"/planner/trajectory_generation"]


from traj_min_snap import SnapOpt, Trajectory, Piece
from traj_utils import allocateTime
import numpy
import matplotlib.pyplot as plt


class MinimimSnapTraj:
    def __init__(self):
        self.snapopt = SnapOpt()
        self.minSnapTraj = Trajectory()
        self.iSS = numpy.zeros((3, 4), dtype=numpy.float64)
        self.fSS = numpy.zeros((3, 4), dtype=numpy.float64)

    def generate(self, waypoints, vel: float, acc: float):
        self.dts = allocateTime(waypoints.T, vel, acc)
        self.iSS[:, 0] = waypoints[0, :].T
        self.fSS[:, 0] = waypoints[-1, :].T

        self.n_waypoints = waypoints.shape[0]

        self.snapopt.reset(self.iSS, self.fSS, self.n_waypoints - 1)
        self.snapopt.generate(waypoints[1:-1, :].T, self.dts)
        self.snapopt.getTraj(self.minSnapTraj)

    def getPos(self, t):
        return self.minSnapTraj.getPos(t)

    def getVel(self, t):
        return self.minSnapTraj.getVel(t)

    def getAcc(self, t):
        return self.minSnapTraj.getAcc(t)

    def getWaypointNum(self):
        return self.n_waypoints

    def getTrajectoryNum(self):
        return self.n_waypoints - 1

    def getPosCoeffMat(self, n, normalized=False):
        return self.minSnapTraj.get(n).getCoeffMat(normalized)

    def getVelCoeffMat(self, n, normalized=False):
        return self.minSnapTraj.get(n).getVelCoeffMat(normalized)

    def getVAccCoeffMat(self, n, normalized=False):
        return self.minSnapTraj.get(n).getAccCoeffMat(normalized)

    def getDurations(self):
        return self.minSnapTraj.getDurations()

    def getTrajectoryOrder(self):
        return 7



if __name__ == "__main__":
    traj = MinimimSnapTraj()
    traj.generate(
        numpy.array([[0, 0, 0], [100, 50, 9], [10, 15, 10], [10, 20, 10]]), 5, 1
    )

