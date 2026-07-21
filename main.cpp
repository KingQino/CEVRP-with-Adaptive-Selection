//
// Created by Yinghao Qin on 21/12/2023.
//

#include <iostream>
#include <thread>
#include <cstdlib>
#include <algorithm>
#include <cctype>

#include "include/case.hpp"
#include "include/MA.hpp"
#include "include/stats.hpp"

using namespace std;

#define MAX_TRIALS 10

const string DATA_PATH = "../data/";

bool parse_local_search_intensity(
    const std::string& value,
    LocalSearchIntensity& intensity) {
    std::string normalized = value;
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });

    if (normalized == "weak") {
        intensity = LocalSearchIntensity::Weak;
        return true;
    }
    if (normalized == "medium") {
        intensity = LocalSearchIntensity::Medium;
        return true;
    }
    if (normalized == "strong") {
        intensity = LocalSearchIntensity::Strong;
        return true;
    }
    return false;
}

std::string generateStatsFilePath(const std::string& filepath) {
    // Extract instance name from filepath
    std::filesystem::path filePath(filepath);
    std::string instanceName = filePath.stem().string();

    // Generate directory path and file path
    std::string directoryPath = "../" + StatsInterface::statsPath + "/" + instanceName;
    std::string statsFilePath = directoryPath + "/stats." + instanceName + ".txt";

    return statsFilePath;
}

int main(int argc, char *argv[]) {
    int run;

    if (argc < 4) {
        cerr << "Usage: " << argv[0]
             << " <problem_instance_filename>"
             << " <stop_criteria: 1 for max-evals, 2 for max-time>"
             << " <multithreading: 1 for yes>"
             << " [local_search_intensity: weak|medium|strong]" << endl;
        return 1;
    }

    string filename(argv[1]);
    string filepath = DATA_PATH + filename;
    int isMaxEvals = std::stoi(argv[2]);
    int isActivateMultiThreading = std::stoi(argv[3]);
    LocalSearchIntensity localSearchIntensity = LocalSearchIntensity::Strong;
    if (argc >= 5
        && !parse_local_search_intensity(argv[4], localSearchIntensity)) {
        cerr << "Invalid local-search intensity: " << argv[4]
             << ". Expected weak, medium, or strong." << endl;
        return 1;
    }

    std::vector<double> perfOfTrials(MAX_TRIALS);
    if (isActivateMultiThreading == 1) {
        std::vector<std::thread> threads;

        // Define a function to perform the threaded work
        auto thread_function = [&](int run) {
            Case* instance = new Case(filepath, run);
            MA* ma = new MA(instance, run, isMaxEvals);
            ma->localSearchIntensity = localSearchIntensity;

            ma->run();

            perfOfTrials[run - 1] = ma->verifiedBest->get_lower_cost();

            delete ma;
            delete instance;
        };

        // Launch threads
        for (run = 2; run <= MAX_TRIALS; ++run) {
            threads.emplace_back(thread_function, run);
        }
        thread_function(1);

        // Wait for threads to finish
        for (auto& thread : threads) {
            thread.join();
        }
    } else {
        for (run = 1; run <= MAX_TRIALS; run++) {
            Case* instance = new Case(filepath, run);
            MA* ma = new MA(instance, run, isMaxEvals);
            ma->localSearchIntensity = localSearchIntensity;

            ma->run();

            perfOfTrials[run - 1] = ma->verifiedBest->get_lower_cost();

            delete ma;
            delete instance;
        }
    }

    StatsInterface::stats_for_multiple_trials(generateStatsFilePath(filepath), perfOfTrials);

    return 0;
}
