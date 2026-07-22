#ifndef LEADER_HPP
#define LEADER_HPP

#include <random>

class Case;
class Individual;

enum class LocalSearchIntensity {
    Skip,
    Weak,
    Medium,
    Strong,
};

struct LocalSearchResult {
    int moveLimit{};
    int acceptedMoves{};
    int neighborhoodCalls{};
    double evalsUsed{};
    double relativeUpperImprovement{};
    bool reachedLocalOptimum{};
};

class Leader {
public:
    static void improve_with_three_neighborhood_vnd(Individual& individual, Case& instance);
    static void improve_with_three_neighborhood_rvnd(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine);
    static void improve_with_five_neighborhood_rvnd(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine);
    static void improve_with_seven_neighborhood_rvnd(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine);
    static void improve_with_seven_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine);
    static LocalSearchResult improve_with_seven_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchIntensity intensity);
};

#endif
