#ifndef LEADER_HPP
#define LEADER_HPP

#include <array>
#include <cstddef>
#include <random>
#include <utility>
#include <vector>

class Case;
class Individual;

enum class LocalSearchOperator {
    NodeShift,
    InterRouteRelocate,
    IntraRouteSwap,
    InterRouteSwap,
    SwapStar,
    TwoOpt,
    TwoOptStarHeadToHead,
    TwoOptStarHeadToTail,
    Count,
};

constexpr std::size_t LOCAL_SEARCH_OPERATOR_COUNT =
    static_cast<std::size_t>(LocalSearchOperator::Count);

struct LocalSearchOperatorStats {
    int calls{};
    int accepts{};
    double evals{};
    double upperGain{};
    int gammaCrosses{};
};

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
    std::array<LocalSearchOperatorStats, LOCAL_SEARCH_OPERATOR_COUNT>
        operatorStats{};
};

struct LocalSearchWorkspace {
    struct InsertionCandidate {
        double cost{};
        int insertionIndex{-1};
    };

    using TopThreeInsertions = std::array<InsertionCandidate, 3>;

    std::vector<int> routeOrder;
    std::vector<std::pair<int, int>> routePairs;
    std::vector<int> firstRouteBuffer;
    std::vector<int> secondRouteBuffer;
    std::vector<TopThreeInsertions> firstCustomersIntoSecond;
    std::vector<TopThreeInsertions> secondCustomersIntoFirst;
    std::vector<double> firstRemovalCosts;
    std::vector<double> secondRemovalCosts;
};

class Leader {
public:
    static const char* operator_name(LocalSearchOperator localSearchOperator);
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
    static LocalSearchResult improve_with_seven_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchIntensity intensity,
        LocalSearchWorkspace& workspace);
    static void improve_with_eight_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine);
    static LocalSearchResult improve_with_eight_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchIntensity intensity);
    static LocalSearchResult improve_with_eight_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchIntensity intensity,
        LocalSearchWorkspace& workspace);
    static LocalSearchResult improve_with_eight_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchIntensity intensity,
        LocalSearchWorkspace& workspace,
        double gammaUpperBound);
};

#endif
