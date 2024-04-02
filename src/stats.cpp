#include "../include/stats.hpp"

using namespace std;


const std::string StatsInterface::statsPath = "stats";

PopulationMetrics StatsInterface::calculate_population_metrics(const std::vector<double> &data) {
    PopulationMetrics metrics;

    // Filter out infeasible data
    std::vector<double> feasibleData;
    for (double value : data) {
        if (value <= INFEASIBLE) {
            feasibleData.push_back(value);
        }
    }

    metrics.size = feasibleData.size();
    metrics.dumbSize = data.size() - feasibleData.size();

    if (feasibleData.empty()) {
        metrics.min = 0.0;
        metrics.max = 0.0;
        metrics.avg = 0.0;
        metrics.std = 0.0;
    } else {
        // Calculate min, max, mean, and standard deviation for feasible data
        metrics.min = *std::min_element(feasibleData.begin(), feasibleData.end());
        metrics.max = *std::max_element(feasibleData.begin(), feasibleData.end());

        double sum = 0.0;
        for (double value : feasibleData) {
            sum += value;
        }
        metrics.avg = sum / static_cast<double>(feasibleData.size());

        double sumSquaredDiff = 0.0;
        for (double value : feasibleData) {
            double diff = value - metrics.avg;
            sumSquaredDiff += diff * diff;
        }
        metrics.std = (feasibleData.size() == 1) ? 0.0 : std::sqrt(sumSquaredDiff / static_cast<double>(feasibleData.size() - 1));
    }

    return metrics;
}

bool StatsInterface::create_directories_if_not_exists(const string &directoryPath) {
    if (!fs::exists(directoryPath)) {
        try {
            fs::create_directories(directoryPath);
            std::cout << "Directory created successfully: " << directoryPath << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return false;
        }
    } else {
        std::cout << "Directory already exists: " << directoryPath << std::endl;
        return true;
    }
}

void StatsInterface::stats_for_multiple_trials(const std::string& filePath, const std::vector<double>& data) {
    std::ofstream logStats;

    logStats.open(filePath);

    std::ostringstream oss;
    for (auto& perf:data) {
        oss << fixed << setprecision(2) << perf << endl;
    }
    PopulationMetrics metric = calculate_population_metrics(data);
    oss << "Mean " << metric.avg << "\t \tStd Dev " << metric.std << "\t " << endl;
    oss << "Min: " << metric.min << "\t " << endl;
    oss << "Max: " << metric.max << "\t " << endl;
    logStats << oss.str() << flush;

    logStats.close();
}
