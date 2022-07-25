#include <statistics.h>
#include <solvers/locate_bancroft.h>
#include <solvers/locate_beck.h>
#include <solvers/locate_grad.h>
#include <solvers/locate_linear.h>
#include <solvers/locate_simple.h>
#include <solvers/locate_zhil.h>

#include <fstream>
#include <filesystem>

using namespace PseudorangeMultilateration;

void PrintStatisticsPoints(const std::vector<StatisticsPoint>& stat_graph, std::ostream& ostream) {
    for (const auto& point : stat_graph) {
        ostream << point.deviation << " ";
        for (const auto& stat : point.stats) {
            ostream << stat.stat.avg / point.deviation << " ";
        }
        ostream << '\n';
    }
}

int main() {
    RandomSceneGenerator generator(666);
    const vector<Solver> solvers{
           { locate_zhil, "zhil" },
           { locate_linear, "linear" },
           { locate_simple, "simple" },
           //{ locate_bancroft, "bancroft" },
           //{ locate_grad, "grad" },
           //{ locate_beck, "beck" },
    };
    //TestMethodError(generator, eigen_solver);
    //TestNoiseError(generator, eigen_solver, 3, 0, 200, 100);

    const Solver simple(locate_simple, "simple");
    const Solver bancfort(locate_bancroft, "bancroft");
    const Solver beck(locate_beck, "beck");

    std::cout << SolveSimpleCase(beck) << '\n';

    //std::string filename = "~/multilat/graph.txt";
    /*std::string filename = "zhil_simple_bancfort_4loc_800km_log100.txt";
    if (std::filesystem::exists(filename)) {
        std::cout << "Output file \"" << filename << "\"  already exists!\n";
        return 0;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "Unable to open file \"" << filename << "\" !\n";
        return 0;
    }
     */

    std::string base_filename = "zhil_lin_simp_";
    std::string tail_filename = "loc_1200km_log100.txt";
    //for (int locators_count : {3, 4, 5, 6, 7, 10, 20, 100}) {
    for (int locators_count : { 100 }) {
        std::string dyn_filename = base_filename;
        dyn_filename += std::to_string(locators_count);
        dyn_filename += tail_filename;
        std::ofstream file(dyn_filename);
        if (!file.is_open()) {
            std::cout << "Unable to open file \"" << dyn_filename << "\" !\n";
            continue;
        }
        const vector<StatisticsPoint> stat_graph = CompareDeviationErrorsInMethods(locators_count, solvers, generator);
        PrintStatisticsPoints(stat_graph, file);
    }

    return 0;
}



