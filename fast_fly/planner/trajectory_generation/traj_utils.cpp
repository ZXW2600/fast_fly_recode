// cppimport
/*<%
import pybind11
cfg['compiler_args'] = ['-std=c++11']
cfg['include_dirs'] = [pybind11.get_include(), pybind11.get_include(True),"/usr/include/eigen3/"]
%>*/
#include <Eigen/Eigen>
#include <pybind11/eigen.h>
#include <iostream>
#include <pybind11/pybind11.h>

using namespace Eigen;

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
        durations[0] = durations[0] * 0.7;
        if (N > 1)
            durations[1] = durations[1] * 0.7;
    }
    // TODO:: 完善时间分配标准
    return durations;
}

namespace py = pybind11;
PYBIND11_MODULE(traj_utils, m)
{
    m.def("allocateTime", &allocateTime);
}
