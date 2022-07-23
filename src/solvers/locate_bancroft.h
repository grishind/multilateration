#pragma once

#include <vector3d.h>
#include <basics.h>

#include <Eigen/Dense>

#include <vector>


namespace PseudorangeMultilateration {

    bool is_surface(const Vector3d& position, double sigma)
    {
        return true; //needs to be improved somehow
    }

    const Vector3d& choose_by_standard_deviation(const int n, const Eigen::MatrixX3d& N, const Eigen::Vector3d& X01, const Eigen::Vector3d& X02, const Eigen::VectorXd& R0, const Vector3d& X1, const Vector3d& X2, const double L01, const double L02)
    {
        Eigen::VectorXd R01(n), R02(n), S01(n), S02(n); // N is a matrix of the size N(n, 3)
        double S1 = 0., S2 = 0.;                        //R0 is a column of the size n

        for (int i = 0; i < n; ++i)                     //const double L01, const double L02 are only needed for std::cout tests
        {
            R01(i) = (N(i, 0) - X01(0)) * (N(i, 0) - X01(0)) + (N(i, 1) - X01(1)) * (N(i, 1) - X01(1)) + (N(i, 2) - X01(2)) * (N(i, 2) - X01(2));
            R02(i) = (N(i, 0) - X02(0)) * (N(i, 0) - X02(0)) + (N(i, 1) - X02(1)) * (N(i, 1) - X02(1)) + (N(i, 2) - X02(2)) * (N(i, 2) - X02(2));
            S01(i) = R01(i) / (R0(i) * R0(i)) - 1;
            S02(i) = R02(i) / (R0(i) * R0(i)) - 1;
            S1 += S01(i) * S01(i);
            S2 += S02(i) * S02(i);
        }

        //std::cout << X1 << '\n' << L01 << '\n' << R01.transpose() << '\n' << S01.transpose() << '\n' << std::sqrt(S1) << '\n' << '\n';
        //std::cout << X2 << '\n' << L02 << '\n' << R02.transpose() << '\n' << S02.transpose() << '\n' << std::sqrt(S2) << '\n' << '\n';

        return S1 > S2 ? X2 : X1;
    }


    double minkowski4d(const Eigen::Vector4d& a, const Eigen::Vector4d& b)
    {
        return a(0) * b(0) + a(1) * b(1) + a(2) * b(2) - a(3) * b(3);
    }

    Vector3d locate_bancroft(const std::vector<LocatorData>& locators)
    {
        const int n = static_cast<int>(locators.size());
        Eigen::Matrix4Xd A_transpose(4, n);
        Eigen::VectorXd r(n), i0(n);

        for (int i = 0; i < n; ++i)
        {
            A_transpose.col(i) << locators[i].position.x(), locators[i].position.y(), locators[i].position.z(), locators[i].distance;
            r(i) = minkowski4d(A_transpose.col(i), A_transpose.col(i)) / 2;
            i0(i) = 1.;
        }

        const Eigen::MatrixX4d A = A_transpose.transpose();
        const Eigen::Matrix4Xd B = (A_transpose * A_transpose.transpose()).inverse() * A_transpose;
        const Eigen::Vector4d u = B * i0;
        const Eigen::Vector4d v = B * r;
        const double E = minkowski4d(u, u);
        const double F = minkowski4d(u, v) - 1;
        const double G = minkowski4d(v, v);
        double D4 = F * F - E * G; //E lambda^2 + 2F lambda + G = 0

        if (D4 < 0.)
            D4 = 0.;

        const double lambda01 = (-F - std::sqrt(D4)) / E;
        const Eigen::Vector4d X01 = lambda01 * u + v;
        const Vector3d X1(X01(0), X01(1), X01(2));
        const double lambda02 = (-F + std::sqrt(D4)) / E;
        const Eigen::Vector4d X02 = lambda02 * u + v;
        const Vector3d X2(X02(0), X02(1), X02(2));

        if (!is_surface(X1, 0.))
        {
            if (is_surface(X2, 0.))
                return X2;

            return Vector3d(); //incorrect input data
        }

        if (!is_surface(X2, 0.))
            return X1;

        return choose_by_standard_deviation(n, A.leftCols<3>(), X01.head<3>(), X02.head<3>(), A.rightCols<1>(), X1, X2, lambda01, lambda02);
    }

}
