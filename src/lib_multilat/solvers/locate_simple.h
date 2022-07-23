#pragma once

#include <vector3d.h>
#include <basics.h>

#include <Eigen/Dense>

#include <vector>


namespace PseudorangeMultilateration {

    Vector3d locate_simple(const std::vector<LocatorData>& locators)
    {
        const int n = static_cast<int>(locators.size());
        Eigen::MatrixX3d position_mat(n, 3);

        for (int i = 0; i < n; ++i) {
            position_mat.row(i) << locators[i].position.x(), locators[i].position.y(), locators[i].position.z();
        }
        position_mat *= 2;

        Eigen::VectorXd value_column(n);
        for (int i = 0; i < n; ++i)
        {
            value_column(i) = locators[i].position.dot_square() - locators[i].distance * locators[i].distance;
        }

        Eigen::MatrixXd subtraction_mat(n - 1, n);
        subtraction_mat.setZero();
        subtraction_mat.topLeftCorner(n - 1, 1) = Eigen::MatrixXd::Constant(n - 1, 1, -1);
        subtraction_mat.topRightCorner(n - 1, n - 1) = Eigen::MatrixXd::Identity(n - 1, n - 1);

        Eigen::MatrixXd subbed_position_mat = subtraction_mat * position_mat;
        Eigen::MatrixXd subbed_position_mat_t = subbed_position_mat.transpose();
        Eigen::VectorXd subbed_value_column = subtraction_mat * value_column;
        Eigen::VectorXd result = (subbed_position_mat_t * subbed_position_mat).ldlt()
                .solve(subbed_position_mat_t * subbed_value_column);

        return { result(0), result(1), result(2) };
    }
}
