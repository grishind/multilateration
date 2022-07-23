#pragma once

#include <vector3d.h>
#include <basics.h>
#include <solvers/locate_simple.h>

#include <Eigen/Dense>

#include <vector>


namespace PseudorangeMultilateration {

    Vector3d locate_grad(const std::vector<LocatorData>& locators) {
        constexpr int iter_count = 2000;
        constexpr double first_step = 3;
        constexpr double step_scale = 0.99;

        const auto size = static_cast<double>(locators.size());
        Vector3d target = locate_simple(locators);
        double step = first_step;
        for (int iter = 0; iter < iter_count; ++iter) {
            Vector3d delta;
            for (const auto & locator : locators) {
                const double dist = target.distance(locator.position);
                const double err = dist - locator.distance;
                const double max_dist = std::max(locator.distance, dist);
                const double scale = step * err / max_dist;
                const Vector3d loc_dir = locator.position - target;
                Vector3d diff = loc_dir * scale;
                delta = delta + diff;
            }
            delta = delta * (1.0 / size);
            step *= step_scale;
            target = target + delta;
        }
        return target;
    }

    Vector3d locate_grad_slow(const std::vector<LocatorData>& locators) {
        constexpr int iter_count = 200000;
        constexpr double first_step = 3;
        constexpr double step_scale = 0.9999;

        const auto size = static_cast<double>(locators.size());
        Vector3d target;//{6500,6500,6500};
        double step = first_step;
        for (int iter = 0; iter < iter_count; ++iter) {
            Vector3d delta;
            for (const auto & locator : locators) {
                const double dist = target.distance(locator.position);
                const double err = dist - locator.distance;
                const double max_dist = std::max(locator.distance, dist);
                const double scale = step * err / max_dist;
                const Vector3d loc_dir = locator.position - target;
                Vector3d diff = loc_dir * scale;
                delta = delta + diff;
            }
            delta = delta * (1.0 / size);
            step *= step_scale;
            target = target + delta;
        }
        return target;
    }
}
