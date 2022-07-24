#pragma once

#include <vector3d.h>
#include <basics.h>

#include <random>

namespace PseudorangeMultilateration {
    class RandomSceneGenerator {
    public:
        explicit RandomSceneGenerator(int seed)
                : random_engine(seed)
        {
        }

        [[nodiscard]] Vector3d GeneratePointInCube(const Cube& cube) {
            std::uniform_real_distribution<double> distr(0, 1);
            const Vector3d portion_vector = {
                    distr(random_engine),
                    distr(random_engine),
                    distr(random_engine),
            };
            return Interpolate(cube.low, cube.high, portion_vector);
        }

        [[nodiscard]] vector<Vector3d> GeneratePointsInCube(int locators_count, const Cube& locators_domain) {
            vector<Vector3d> points(locators_count);
            for (auto& point : points) {
                point = GeneratePointInCube(locators_domain);
            }
            return points;
        }

        [[nodiscard]] Scene GenerateScene(int locators_count, const Cube& locators_domain, const Cube& target_domain) {
            const vector<Vector3d> locators = GeneratePointsInCube(locators_count, locators_domain);
            const Vector3d target = GeneratePointInCube(target_domain);
            return { locators, target };
        }

        [[nodiscard]] vector<Scene> GenerateScenes(int scenes_count,
                                                   int locators_count,
                                                   const Cube& locators_domain,
                                                   const Cube& target_domain) {
            vector<Scene> scenes;
            scenes.reserve(scenes_count);
            for (size_t i = 0; i < scenes_count; ++i) {
                scenes.push_back(GenerateScene(locators_count, locators_domain, target_domain));
            }
            return scenes;
        }

        [[nodiscard]] double PickRandomNumberOnSegment(double left, double right) {
            std::uniform_real_distribution<double> uniform(left, right);
            return uniform(random_engine);
        }

        [[nodiscard]] double PickRandomLongitude() {
            const double pi = std::acos(-1);
            return PickRandomNumberOnSegment(0, 2 * pi);
        }

        [[nodiscard]] Vector3d PickRandomUnitVector() {
            const double longitude = PickRandomLongitude();
            const double height = PickRandomNumberOnSegment(-1, 1);
            const double latitude_radius = std::sqrt(1 - height * height);
            return {
                latitude_radius * std::cos(longitude),
                latitude_radius * std::sin(longitude),
                height
            };
        }

        [[nodiscard]] Vector3d NoisifyPoint(const Vector3d& point, double deviation) {
            std::normal_distribution<double> distr(0, deviation);
            const Vector3d unit = PickRandomUnitVector();
            const double radius = distr(random_engine);
            return point + radius * unit;
        }

        [[nodiscard]] vector<Vector3d> NoisifyPoints(const vector<Vector3d>& points, double deviation) {
            vector<Vector3d> noisy_points;
            std::transform(points.begin(), points.end(), std::back_inserter(noisy_points),
                           [this, deviation](const Vector3d& point) {
                               return NoisifyPoint(point, deviation);
                           });
            return noisy_points;
        }

        [[nodiscard]] Scenario CreateNoisyScenario(const Scene& scene, double deviation) {
            const vector<double> distances = ComputeDistances(scene);
            const vector<Vector3d> noisy_observers = NoisifyPoints(scene.observers, deviation);
            return {
                    CreateLocators(noisy_observers, distances),
                    scene.target
            };
        }

        [[nodiscard]] vector<Scenario> CreateNoisyScenarios(const Scene& original, double deviation, int count) {
            vector<Scenario> scenarios;
            for (int i = 0; i < count; ++i) {
                scenarios.push_back(CreateNoisyScenario(original, deviation));
            }
            return scenarios;
        }

        [[nodiscard]] vector<Scenario> CreateNoisyScenarios(const Scene& original,
                                                            const vector<double>& deviations) {
            vector<Scenario> scenarios;
            scenarios.reserve(deviations.size());
            for (double deviation : deviations) {
                scenarios.push_back(CreateNoisyScenario(original, deviation));
            }
            return scenarios;
        }

    private:
        std::mt19937 random_engine;
    };
}
