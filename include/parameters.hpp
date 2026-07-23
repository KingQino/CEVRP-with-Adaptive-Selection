#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <string>

#include "leader.hpp"

struct Parameters {
    std::string dataPath;
    std::string statsPath;
    std::string instance = "E-n22-k4.evrp";
    bool enableLogging = false;
    int stopCriteria = 1;
    bool enableMultithreading = false;
    int seed = 0;

    int popSize = 100;
    double mutationProb = 0.5;
    double mutationIndProb = 0.2;
    int tournamentSize = 2;
    LocalSearchIntensity localSearchIntensity = LocalSearchIntensity::Strong;
    bool enableLocalSearchLearning = false;
    double parentPoolRatio = 0.10;
    double qualityRatio = 0.50;
    double verifiedUpperRatio = 0.05;
    double pureImmigrantRatio = 0.10;
    double gamma = 1.02;

    void validate() const;
};

#endif
