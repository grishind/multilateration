#pragma once

#include <basics.h>
#include <scenegenerator.h>
#include <solver.h>
#include <solvers/locate_bancroft.h>
#include <solvers/locate_beck.h>
#include <solvers/locate_grad.h>
#include <solvers/locate_simple.h>
#include <solvers/locate_zhil.h>
#include <utils.h>

#include <algorithm>
#include <iostream>
#include <string>

namespace PseudorangeMultilateration {
    Scene GenerateDiagonalCubicScene(RandomSceneGenerator& generator, int locators_count) {
        const Cube locators_domain = diagonalCube(6800, 7200);
        const Cube target_domain = diagonalCube(6300, 6500);
        return generator.GenerateScene(
                locators_count,
                locators_domain,
                target_domain
        );
    }

    Scene GenerateOrbitalScene(int planes_count, int sat_per_plane) {
        const double earth_radius = 6371;
        const Vector3d target{ 0, 0, earth_radius };
        return {};
    }

    vector<double> CollectMethodErrors(RandomSceneGenerator& generator,
                                       const Solver& solver,
                                       int locators_count,
                                       int scenes_count = 100) {
        const Cube locators_domain = diagonalCube(6800, 7200);
        const Cube target_domain = diagonalCube(6300, 6500);
        const vector<Scene> scenes = generator.GenerateScenes(
                scenes_count,
                locators_count,
                locators_domain,
                target_domain
        );
        vector<double> errors;
        std::transform(scenes.begin(), scenes.end(), std::back_inserter(errors),
                       [&solver](const Scene& scene) {
                           return solver.FindLocationError(scene);
                       });
        return errors;
    }

    Statistics CollectMethodErrorStatistics(RandomSceneGenerator& generator,
                                            const Solver& solver,
                                            int locators_count,
                                            int scenes_count = 100) {
        const vector<double> errors = CollectMethodErrors(generator, solver, locators_count, scenes_count);
        return CollectStatistics(errors);
    }

    void TestMethodError(RandomSceneGenerator& generator, const Solver& solver) {
        for (int locators_count = 3; locators_count <= 20; locators_count++) {
            const Statistics stats = CollectMethodErrorStatistics(generator, solver, locators_count);
            std::cout << stats << '\n';
        }
    }

    vector<double> ComputeSelection(double min_val, double max_val, int steps) {
        vector<double> selection;
        selection.reserve(steps + 1);
        const double min_log = std::log(min_val);
        const double max_log = std::log(max_val);
        const double diff_log = max_log - min_log;
        for (int i = 0; i <= steps; ++i) {
            const double portion = static_cast<double>(i) / steps;
            const double cur_log = min_log + portion * diff_log;
            const double value = std::exp(cur_log);
            selection.push_back(value);
        }
        return selection;
    }

    Statistics CollectNoiseErrorStatistics(const vector<Scenario>& scenarios,
                                           const Solver& solver) {
        const vector<double> errors = solver.FindPositioningErrors(scenarios);
        const Statistics stat = CollectStatistics(errors);
        return stat;
    }

    Statistics CollectNoiseErrorStatistics(RandomSceneGenerator& generator,
                                           Solver& solver,
                                           const Scene& scene,
                                           double deviation) {
        const vector<Scenario> scenarios = generator.CreateNoisyScenarios(scene, deviation, 100);
        return CollectNoiseErrorStatistics(scenarios, solver);
    }


    vector<Statistics> TestNoiseError(RandomSceneGenerator& generator,
                                      Solver& solver,
                                      const Scene& scene,
                                      const vector<double>& deviations)
    {
        vector<Statistics> stats;
        for (double deviation : deviations) {
            std::cout << "dev: " << deviation << '\n';
            const Statistics stat = CollectNoiseErrorStatistics(generator, solver, scene, deviation);
            stats.push_back(stat);
        }
        return stats;
    }

    vector<Statistics> TestNoiseError(RandomSceneGenerator& generator,
                                      Solver& solver,
                                      int locators_count,
                                      double min_dev,
                                      double max_dev,
                                      int steps)
    {
        const Scene scene = GenerateDiagonalCubicScene(generator, locators_count);
        const vector<double> deviations = ComputeSelection(min_dev, max_dev, steps);
        return TestNoiseError(generator, solver, scene, deviations);
    }

    vector<NoisySelection> CreateNoisySelection(int count,
                                                const Scene& scene,
                                                const vector<double>& deviations,
                                                RandomSceneGenerator& generator) {
        vector<NoisySelection> noisy_selection;
        for (double dev : deviations) {
            const vector<Scenario> scenarios = generator.CreateNoisyScenarios(scene, dev, count);
            noisy_selection.push_back({scenarios, dev});
        }
        return noisy_selection;
    }

    [[nodiscard]] vector<StatisticsPoint> CompareDeviationErrorsInMethods(int locators_count,
                                                            const vector<Solver>& solvers,
                                                            RandomSceneGenerator& generator) {
        const Scene scene = GenerateDiagonalCubicScene(generator, locators_count);
        const vector<double> deviations = ComputeSelection(1e-3, 2000, 100);
        const vector<NoisySelection> noisy_selection = CreateNoisySelection(1000, scene, deviations, generator);

        vector<StatisticsPoint> stat_graph(noisy_selection.size());
        {
            int counter = 0;
            #pragma omp parallel for default(none) shared(counter, noisy_selection, solvers, stat_graph, std::cout)
            for (size_t i = 0; i < noisy_selection.size(); ++i) {
                const auto& selection = noisy_selection.at(i);
                vector<NamedStatistics> stats;
                for (const auto& solver : solvers) {
                    const std::string solver_name = solver.GetDesignator();
                    stats.push_back({
                                            CollectNoiseErrorStatistics(selection.scenarios, solver),
                                            solver_name
                    });
                }
                stat_graph.at(i) = {
                        selection.deviation,
                        stats
                };
                counter++;
                std::cout << counter << "/" << noisy_selection.size() << "\n";
            }
        }
        return stat_graph;
    }

    Vector3d SolveSimpleCase(const Solver& solver) {
        Scene scene = {
                {
                        {1, 1, 0},
                        {1, -1, 0},
                        {-1, 1, 0},
                        {-1, -1, 0},
                        {0, 0, -1}
                },
                {0, 0, 1}
        };
        return solver.LocateInScene(scene);
    }
}



