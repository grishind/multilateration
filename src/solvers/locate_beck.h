#pragma once

#include <vector3d.h>
#include <basics.h>

#include <Eigen/Dense>

#include <vector>


namespace PseudorangeMultilateration {

    class BeckSolver {
    public:
        explicit BeckSolver(const std::vector<LocatorData>& locators)
                : position_mat(locators.size(), 4)
                , value_column(locators.size())
                , proj_mat (4, 4)
                , q_vec(4)
        {
            const int n = static_cast<int>(locators.size());
            for (int i = 0; i < n; ++i) {
                position_mat.row(i) << 2 * locators[i].position.x(),
                                       2 * locators[i].position.y(),
                                       2 * locators[i].position.z(),
                                       -1;
            }

            for (int i = 0; i < n; ++i)
            {
                value_column(i) = locators[i].position.dot_square() - locators[i].distance * locators[i].distance;
            }

            proj_mat.setZero();
            proj_mat.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);

            q_vec << 0, 0, 0, -1./2.;
        }

        Eigen::Vector4d getZVec(double lambda) {
            Eigen::MatrixXd trans = position_mat.transpose();
            Eigen::MatrixXd mat = trans * position_mat + lambda * proj_mat;
            Eigen::MatrixXd inv = mat.inverse();
            Eigen::Vector4d vec = trans * value_column - lambda * q_vec;
            return inv * vec;
        }

        static double target(const Eigen::Vector4d& vec) {
            return vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2) - vec(3);
        }

        double target(double lambda) {
            auto z_vec = getZVec(lambda);
            return target(z_vec);
        }

        double bisection(double left) {
            double length = 1;
            while (target(left + length) > 0) {
                length *= 2;
            }
            double right = left + length;
            while (right - left > 1e-5) {
                const double mid = left + (right - left) / 2;
                if (target(mid) > 0) {
                    left = mid;
                }
                else {
                    right = mid;
                }
            }
            return left;
        }

        Vector3d solve() {
            const double lambda = bisection(0);
            const auto z_vec = getZVec(lambda);
            return { z_vec(0), z_vec(1), z_vec(2) };
        }

    private:
        Eigen::MatrixX4d position_mat;
        Eigen::VectorXd value_column;
        Eigen::MatrixX4d proj_mat;
        Eigen::Vector4d q_vec;
    };

    Vector3d locate_beck(const std::vector<LocatorData>& locators)
    {
        BeckSolver beck_solver(locators);
        return beck_solver.solve();
    }
}
