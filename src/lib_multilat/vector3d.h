#pragma once

#include <cmath>
#include <iostream>

class Vector3d {
public:
    double x_[3];

    constexpr Vector3d()
            : x_{0} {}

    constexpr Vector3d(double x, double y, double z)
            : x_{x, y, z} {}

    constexpr explicit Vector3d(const double *const x)
            : x_{x[0], x[1], x[2]} {}

    constexpr explicit Vector3d(const float *const x)
            : x_{static_cast<double>(x[1]),
                 static_cast<double>(x[2]),
                 static_cast<double>(x[3])
    } {}

    constexpr Vector3d(const Vector3d &) = default;

    constexpr Vector3d &operator=(const Vector3d &) = default;

    [[nodiscard]] constexpr double x() const { return x_[0]; }

    [[nodiscard]] constexpr double y() const { return x_[1]; }

    [[nodiscard]] constexpr double z() const { return x_[2]; }

    constexpr double operator[](size_t i) const { return x_[i]; }

    [[nodiscard]] double dot_square() const {
        return x_[0] * x_[0] + x_[1] * x_[1] + x_[2] * x_[2];
    }

    [[nodiscard]] double modulus() const {
        return std::sqrt(dot_square());
    }

    Vector3d operator*(double scalar) const {
        return {
            x_[0] * scalar,
            x_[1] * scalar,
            x_[2] * scalar,
        };
    }

    Vector3d operator+(const Vector3d& vec) const {
        return {
                x_[0] + vec.x_[0],
                x_[1] + vec.x_[1],
                x_[2] + vec.x_[2],
        };
    }

    Vector3d operator-() const {
        return {
                - x_[0],
                - x_[1],
                - x_[2],
        };
    }

    Vector3d operator-(const Vector3d& vec) const {
        return *this + (- vec);
    }

    Vector3d operator/(double scalar) const {
        return *this * (1. / scalar);
    }

    Vector3d &operator*=(double scalar) {
        *this = *this * scalar;
        return *this;
    }

    Vector3d &operator/=(double scalar) {
        *this = *this / scalar;
        return *this;
    }

    Vector3d &operator+=(const Vector3d &vec) {
        *this = *this + vec;
        return *this;
    }

    Vector3d &operator-=(const Vector3d &vec) {
        *this = *this - vec;
        return *this;
    }

    [[nodiscard]] double distance(const Vector3d& vec) const {
        return (*this - vec).modulus();
    }


    Vector3d &normalize() {
        return *this /= modulus();
    }

    Vector3d &rotate(const Vector3d &v, double alpha) {
        const double m = v.modulus();
        const double a = v.x() / m;
        const double b = v.y() / m;
        const double c = v.z() / m;

        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        const double x = x_[0] * (a * a + (1.0 - a * a) * ca) +
                         x_[1] * (a * b * (1.0 - ca) - c * sa) +
                         x_[2] * (a * c * (1.0 - ca) + b * sa);

        const double y = x_[0] * (a * b * (1.0 - ca) + c * sa) +
                         x_[1] * (b * b + (1.0 - b * b) * ca) +
                         x_[2] * (b * c * (1.0 - ca) - a * sa);

        const double z = x_[0] * (a * c * (1.0 - ca) - b * sa) +
                         x_[1] * (b * c * (1.0 - ca) + a * sa) +
                         x_[2] * (c * c + (1.0 - c * c) * ca);

        x_[0] = x;
        x_[1] = y;
        x_[2] = z;
        return *this;
    }

    [[nodiscard]] Vector3d rotate_x(double alpha) const {
        const double ca = std::cos(alpha);
        const double sa = std::cos(alpha);
        return {
            x_[0],
            x_[1] * ca - x_[2] * sa,
            x_[1] * sa + x_[2] * ca
        };
    }

    [[nodiscard]] Vector3d rotate_y(double alpha) const {
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);
        return {
            x_[0] * ca + x_[2] * sa,
            x_[1],
            x_[2] * ca - x_[0] * sa
        };
    }

    [[nodiscard]] Vector3d rotate_z(double alpha) const {
        const double ca = std::cos(alpha);
        const double sa = std::cos(alpha);
        return {
            x_[0] * ca - x_[1] * sa,
            x_[0] * sa + x_[1] * ca,
            x_[2]
        };
    }

    Vector3d &rotate_x(double alpha) {
        const double ca = std::cos(alpha);
        const double sa = std::cos(alpha);
        const double y = x_[1] * ca - x_[2] * sa;
        const double z = x_[1] * sa + x_[2] * ca;
        x_[1] = y;
        x_[2] = z;
        return *this;
    }

    Vector3d &rotate_y(double alpha) {
        const double ca = std::cos(alpha);
        const double sa = std::cos(alpha);
        const double x = x_[0] * ca + x_[2] * sa;
        const double z = x_[2] * ca - x_[0] * sa;
        x_[0] = x;
        x_[2] = z;
        return *this;
    }

    Vector3d &rotate_z(double alpha) {
        const double ca = std::cos(alpha);
        const double sa = std::cos(alpha);
        const double x = x_[0] * ca - x_[1] * sa;
        const double y = x_[0] * sa + x_[1] * ca;
        x_[0] = x;
        x_[1] = y;
        return *this;
    }
};

std::ostream &operator<<(std::ostream &os, const Vector3d &vec) {
    os << "(" << vec.x() << ", "
              << vec.y() << ", "
              << vec.z()
       << ")";
    return os;
}

Vector3d cross_prod(const Vector3d &v1, const Vector3d &v2) {
    return {v1.y() * v2.z() - v1.z() * v2.y(),
            v1.z() * v2.x() - v1.x() * v2.z(),
            v1.x() * v2.y() - v1.y() * v2.x()};
}

Vector3d operator*(const double scalar, const Vector3d& vec) {
    return vec * scalar;
}
