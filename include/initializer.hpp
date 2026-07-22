#ifndef INITIALIZER_HPP
#define INITIALIZER_HPP

#include <random>
#include <vector>

class Case;

struct SplitWorkspace {
    std::vector<int> predecessors;
    std::vector<double> costs;
    std::vector<int> giantTour;
};

class Initializer {
public:
    static std::vector<std::vector<int>> split_giant_tour(
        const std::vector<int>& giantTour,
        Case& instance);
    static std::vector<std::vector<int>> split_giant_tour(
        const std::vector<int>& giantTour,
        Case& instance,
        SplitWorkspace& workspace);

    static std::vector<std::vector<int>> build_with_random_split(
        Case& instance,
        std::mt19937& randomEngine);
    static std::vector<std::vector<int>> build_with_random_split(
        Case& instance,
        std::mt19937& randomEngine,
        SplitWorkspace& workspace);

    static std::vector<std::vector<int>> build_with_clustering(
        const Case& instance,
        std::mt19937& randomEngine);

    static std::vector<std::vector<int>> build_with_direct_encoding(
        const Case& instance,
        std::mt19937& randomEngine);
};

#endif
