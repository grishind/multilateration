#pragma once

#include <utility>

#include <basics.h>
#include <scenegenerator.h>

namespace PseudorangeMultilateration {

    class Solver {
    public:
        Solver(std::function<Vector3d(const std::vector<LocatorData>&)> locate,
               std::string designator)
                : locate(std::move(locate))
                , designator(std::move(designator))
        {
        }

        [[nodiscard]] std::string GetDesignator() const {
            return designator;
        }

        [[nodiscard]] Vector3d LocateInScene(const Scene& scene) const {
            const vector<LocatorData> locators = CreateLocators(scene);
            return locate(locators);
        }

        [[nodiscard]] double FindLocationError(const Scene& scene) const {
            return scene.target.distance(LocateInScene(scene));
        }

        [[nodiscard]] vector<double> FindLocationErrorsAcrossScenes(const vector<Scene>& scenes) const {
            vector<double> errors;
            for (const Scene& scene : scenes) {
                errors.push_back(FindLocationError(scene));
            }
            return errors;
        }

        [[nodiscard]] double FindPositioningError(const Scenario& scenario) const {
            return scenario.latent.distance(locate(scenario.locators));
        }

        [[nodiscard]] vector<double> FindPositioningErrors(const vector<Scenario>& scenarios) const {
            vector<double> errors;
            std::transform(scenarios.begin(), scenarios.end(), std::back_inserter(errors),
                           [this](const Scenario& scenario) {
                return FindPositioningError(scenario);
            });
            return errors;
        }

        [[nodiscard]] double FindPositioningError(const Scene& scene,
                                                  double deviation,
                                                  RandomSceneGenerator& generator) const {
            Scenario scenario = generator.CreateNoisyScenario(scene, deviation);
            return FindPositioningError(scenario);
        }

        [[nodiscard]] vector<double> FindPositioningErrors(const Scene& scene,
                                                           double deviation,
                                                           int iterations,
                                                           RandomSceneGenerator& generator) const
        {
            vector<double> errors(iterations);
            for (double& error : errors) {
                error = FindPositioningError(scene, deviation, generator);
            }
            return errors;
        }

    private:
        std::function<Vector3d(const std::vector<LocatorData>&)> locate;
        std::string designator;
    };
}
