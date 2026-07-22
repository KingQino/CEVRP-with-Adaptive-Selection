#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "MA.hpp"
#include "case.hpp"
#include "command_line.hpp"
#include "parameters.hpp"
#include "stats.hpp"

namespace fs = std::filesystem;

namespace {

constexpr std::size_t MAX_TRIALS = 10;

double run_algorithm(const Parameters& baseParameters, int runSeed) {
    Parameters parameters = baseParameters;
    parameters.seed = runSeed;
    const fs::path instancePath =
        fs::path(parameters.dataPath) / parameters.instance;
    Case instance(instancePath.string(), runSeed);
    MA algorithm(&instance, parameters);
    algorithm.run();
    return algorithm.verifiedBest->get_lower_cost();
}

fs::path aggregate_stats_path(const Parameters& parameters) {
    const std::string instanceName = fs::path(parameters.instance).stem().string();
    return fs::path(parameters.statsPath)
        / instanceName
        / ("stats." + instanceName + ".txt");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        CommandLine commandLine(argc, argv);
        if (commandLine.help_requested()) {
            CommandLine::display_help();
            return 0;
        }

        Parameters parameters;
        commandLine.parse_parameters(parameters);
        parameters.validate();

        const std::size_t trialCount = parameters.enableMultithreading
            ? MAX_TRIALS
            : 1;
        std::vector<double> performance(trialCount, 0.0);

        if (!parameters.enableMultithreading) {
            performance[0] = run_algorithm(parameters, parameters.seed);
        } else {
            std::vector<std::thread> threads;
            std::vector<std::exception_ptr> failures(trialCount);
            threads.reserve(trialCount);
            for (std::size_t trial = 0; trial < trialCount; ++trial) {
                threads.emplace_back([&, trial]() {
                    try {
                        // With seed 0 this preserves the traditional seeds 1..10.
                        const int runSeed = parameters.seed
                            + static_cast<int>(trial)
                            + 1;
                        performance[trial] = run_algorithm(parameters, runSeed);
                    } catch (...) {
                        failures[trial] = std::current_exception();
                    }
                });
            }
            for (auto& thread : threads) {
                thread.join();
            }
            for (const auto& failure : failures) {
                if (failure != nullptr) {
                    std::rethrow_exception(failure);
                }
            }
        }

        if (parameters.enableLogging) {
            const fs::path statsFile = aggregate_stats_path(parameters);
            StatsInterface::create_directories_if_not_exists(
                statsFile.parent_path().string());
            StatsInterface::stats_for_multiple_trials(
                statsFile.string(),
                performance);
        }

        double objective = performance.front();
        if (parameters.enableMultithreading) {
            objective = StatsInterface::calculate_population_metrics(performance).average;
        }
        std::cout << std::fixed << std::setprecision(5) << objective << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << '\n';
        return 1;
    }
}
