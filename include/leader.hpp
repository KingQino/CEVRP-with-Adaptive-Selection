#ifndef LEADER_HPP
#define LEADER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
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
static_assert(
    LOCAL_SEARCH_OPERATOR_COUNT <= 16,
    "local-search operator mask exceeds its storage");
constexpr std::size_t LOCAL_SEARCH_OPERATOR_MASK_COUNT =
    1U << LOCAL_SEARCH_OPERATOR_COUNT;
constexpr std::uint16_t LOCAL_SEARCH_ALL_OPERATOR_MASK =
    static_cast<std::uint16_t>(
        LOCAL_SEARCH_OPERATOR_MASK_COUNT - 1U);

struct LocalSearchOperatorSelectionEntry {
    std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>
        cumulativeProbabilities{};
    std::size_t activeCount{};
};

struct LocalSearchOperatorSelectionTable {
    std::array<
        LocalSearchOperatorSelectionEntry,
        LOCAL_SEARCH_OPERATOR_MASK_COUNT> entries{};
};

struct LocalSearchOperatorStats {
    int calls{};
    int accepts{};
    std::uint64_t distanceCalls{};
    double upperGain{};
    int gammaCrosses{};
};

struct LocalSearchAcceptedMove {
    LocalSearchOperator localSearchOperator{
        LocalSearchOperator::NodeShift};
    double upperGain{};
};

enum class LocalSearchIntensity {
    Skip,
    Weak,
    Medium,
    BoundedStrong,
    Strong,
};

struct LocalSearchResult {
    int moveLimit{};
    int acceptedMoves{};
    int neighborhoodCalls{};
    std::uint64_t distanceCallsUsed{};
    double relativeUpperImprovement{};
    bool reachedLocalOptimum{};
    bool hitMoveLimit{};
    bool hitDistanceCallLimit{};
    std::array<LocalSearchOperatorStats, LOCAL_SEARCH_OPERATOR_COUNT>
        operatorStats{};
    std::vector<LocalSearchAcceptedMove> acceptedMoveEvents;
};

struct LocalSearchSession {
    bool initialized{};
    int totalAcceptedMoves{};
    int totalNeighborhoodCalls{};
    std::uint64_t totalDistanceCalls{};
    std::uint16_t activeOperatorMask{};
    std::vector<LocalSearchOperator> activeOperators;
};

struct LocalSearchWorkspace {
    struct InsertionCandidate {
        double cost{};
        int insertionIndex{-1};
    };

    using TopThreeInsertions = std::array<InsertionCandidate, 3>;

    struct ActiveRoutePairPool {
        std::vector<std::pair<int, int>> pairs;
        std::vector<unsigned char> membership;
        std::vector<std::uint64_t> observedRouteVersions;
        std::uint64_t topologyVersion{};
        int routeCount{};
        bool directed{};
    };

    std::vector<int> routeOrder;
    std::vector<int> firstRouteBuffer;
    std::vector<int> secondRouteBuffer;
    std::vector<TopThreeInsertions> firstCustomersIntoSecond;
    std::vector<TopThreeInsertions> secondCustomersIntoFirst;
    std::vector<double> firstRemovalCosts;
    std::vector<double> secondRemovalCosts;
    // Failed route scans use versions; failed pair scans leave the active pool.
    std::vector<std::uint64_t> routeVersions;
    std::array<
        std::vector<std::uint64_t>,
        LOCAL_SEARCH_OPERATOR_COUNT> failedRouteVersions;
    std::array<ActiveRoutePairPool, LOCAL_SEARCH_OPERATOR_COUNT>
        activeRoutePairPools;
    std::size_t routePairCacheStride{};
    std::uint64_t nextRouteVersion{1};
    std::uint64_t routeTopologyVersion{1};
    std::array<int, 2> changedRoutes{-1, -1};
    int changedRouteCount{};
    bool allRoutesChanged{};
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
    static void begin_eight_neighborhood_rvnd_one_move_session(
        Individual& individual,
        LocalSearchSession& session,
        LocalSearchWorkspace& workspace);
    static LocalSearchOperatorSelectionTable
    build_operator_selection_table(
        const std::array<
            double,
            LOCAL_SEARCH_OPERATOR_COUNT>& operatorSelectionWeights,
        double operatorUniformExplorationRate);
    static LocalSearchResult continue_eight_neighborhood_rvnd_one_move_session(
        Individual& individual,
        Case& instance,
        std::mt19937& randomEngine,
        LocalSearchSession& session,
        int cumulativeMoveLimit,
        LocalSearchWorkspace& workspace,
        double gammaUpperBound,
        std::uint64_t cumulativeDistanceCallLimit =
            std::numeric_limits<std::uint64_t>::max(),
        const LocalSearchOperatorSelectionTable*
            operatorSelectionTable = nullptr,
        std::mt19937* operatorSelectionEngine = nullptr);
    static int move_limit_for_intensity(
        const Individual& individual,
        const Case& instance,
        LocalSearchIntensity intensity);
    static std::uint64_t bounded_strong_distance_call_limit(
        std::uint64_t weakDistanceCalls);
};

#endif
