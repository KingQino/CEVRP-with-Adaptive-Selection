#ifndef CEVRP_YINGHAO_STATS_HPP
#define CEVRP_YINGHAO_STATS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "../include/utils.hpp"

namespace fs = std::filesystem;


struct PopulationMetrics {
    double min{};
    double max{};
    double avg{};
    double std{};
    std::size_t size{}; // population size
    std::size_t dumbSize{}; // the number of infeasible individuals, initialized to 0
};

class StatsInterface {
public:
    static const std::string statsPath;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    std::chrono::duration<double> duration;

    std::ofstream logEvolution;
    std::ofstream logSolution;

    static PopulationMetrics calculate_population_metrics(const std::vector<double>& data) ;
    static bool create_directories_if_not_exists(const std::string& directoryPath);
    static void stats_for_multiple_trials(const std::string& filePath, const std::vector<double>& data); // open a file, save the statistical info, and then close it
    virtual void open_log_for_evolution() = 0; // open a file
    virtual void flush_row_into_evol_log() = 0; // flush the evolution info into the file
    virtual void close_log_for_evolution() = 0; // close the file
    virtual void save_log_for_solution() = 0; // open a file, save the solution, and close it

    virtual ~StatsInterface() = default;
};



#endif // CEVRP_YINGHAO_STATS_HPP