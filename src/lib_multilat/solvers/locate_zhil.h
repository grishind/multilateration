#pragma once

#include <vector3d.h>
#include <basics.h>
#include <solvers/locate_bancroft.h>

#include <Eigen/Dense>

#include <vector>


namespace PseudorangeMultilateration {
    Vector3d locate_zhil(const std::vector<LocatorData>& locators)
    {
        const int n = static_cast<int>(locators.size());
        Eigen::MatrixX3d N(n, 3);

        for (int i = 0; i < n; ++i)
            N.row(i) << locators[i].position.x(), locators[i].position.y(), locators[i].position.z();

        const Eigen::MatrixX3d M = N * (N.transpose() * N).inverse();
        const double Bx = M.col(0).sum();
        const double By = M.col(1).sum();
        const double Bz = M.col(2).sum();
        const double a = (Bx * Bx + By * By + Bz * Bz) / 4;
        Eigen::VectorXd R0(n), Lr(n);

        for (int i = 0; i < n; ++i)
        {
            R0(i) = locators[i].distance;
            Lr(i) = (locators[i].position.x() * locators[i].position.x() + locators[i].position.y() * locators[i].position.y() + locators[i].position.z() * locators[i].position.z() - R0(i) * R0(i)) / 2;
        }

        const double Ax = M.col(0).dot(Lr);
        const double Ay = M.col(1).dot(Lr);
        const double Az = M.col(2).dot(Lr);
        const double b = Ax * Bx + Ay * By + Az * Bz - 1;
        const double c = Ax * Ax + Ay * Ay + Az * Az;
        double D = b * b - 4 * a * c; //a > 0 && c > 0: both real roots have the same sign

        if (D < 0.)
            D = 0.;

        const double L01 = (-b - std::sqrt(D)) / (2 * a);

        if (L01 < 0)
            return Vector3d(); //negative roots: incorrect input data

        const Eigen::Vector3d X01 = M.transpose() * (Lr.array() + L01 / 2).matrix();
        const Vector3d X1(X01(0), X01(1), X01(2));
        const double L02 = (-b + std::sqrt(D)) / (2 * a);
        const Eigen::Vector3d X02 = M.transpose() * (Lr.array() + L02 / 2).matrix();
        const Vector3d X2(X02(0), X02(1), X02(2));

        if (!is_surface(X1, 0.))
        {
            if (is_surface(X2, 0.))
                return X2;

            return Vector3d(); //incorrect input data
        }

        if (!is_surface(X2, 0.))
            return X1;

        return choose_by_standard_deviation(n, N, X01, X02, R0, X1, X2, L01, L02);
    }
}
