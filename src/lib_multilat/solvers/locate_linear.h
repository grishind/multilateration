#pragma once

#include <vector3d.h>
#include <basics.h>

#include <Eigen/Dense>

#include <vector>

namespace PseudorangeMultilateration {

    Vector3d locate_linear(const std::vector<LocatorData>& locators)
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

        Eigen::MatrixXd subtraction_mat(((n - 1) * n) / 2, n);
        subtraction_mat.setZero();
        int row = 0;
        for (int i = 0; i < n - 1; ++i) {
            const int id_size = n - 1 - i;
            subtraction_mat.block(row, i, id_size, 1)
                = Eigen::MatrixXd::Constant(id_size, 1, -1);
            subtraction_mat.block(row, i + 1, id_size, id_size)
                = Eigen::MatrixXd::Identity(id_size, id_size);
            row += id_size;
        }

        Eigen::MatrixXd eq_mat = subtraction_mat * position_mat;
        Eigen::MatrixXd eq_mat_t = eq_mat.transpose();
        Eigen::VectorXd eq_col = subtraction_mat * value_column;
        Eigen::VectorXd result = (eq_mat_t * eq_mat).ldlt().solve(eq_mat_t * eq_col);

        return { result(0), result(1), result(2) };
    }
}
