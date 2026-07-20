#ifndef INITIALIZER_HPP
#define INITIALIZER_HPP

#include <random>
#include <vector>

class Case;

class Initializer {
public:
    static std::vector<std::vector<int>> split_giant_tour(
        const std::vector<int>& giantTour,
        Case& instance);

    static std::vector<std::vector<int>> build_with_random_split(
        Case& instance,
        std::default_random_engine& randomEngine);

    static std::vector<std::vector<int>> build_with_clustering(
        const Case& instance,
        std::default_random_engine& randomEngine);

    static std::vector<std::vector<int>> build_with_direct_encoding(
        const Case& instance,
        std::default_random_engine& randomEngine);
};

#endif
