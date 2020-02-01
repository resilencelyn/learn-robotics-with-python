#ifndef ROBOTICS_TOOLBOX_H
#define ROBOTICS_TOOLBOX_H
#include <Eigen/Eigen>

namespace rtb{

    Eigen::Matrix3d EulerToMatrix(const double, const double, const double);

    Eigen::Vector3d MatrixToEuler(Eigen::Matrix3d);

}

#endif // ROBOTICS_TOOLBOX_H
