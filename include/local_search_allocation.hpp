#ifndef LOCAL_SEARCH_ALLOCATION_HPP
#define LOCAL_SEARCH_ALLOCATION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "leader.hpp"
#include "reproduction.hpp"

class Case;
class Individual;

enum class LocalSearchPolicy {
    Static,
    RandomMixed,
    ContextualMixed,
};

struct LocalSearchAllocationContext {
    double qualityGap{};
    double adjacencyDistance{};
    double budgetProgress{};
    double gammaMargin{};
    double successRate{};
    double normalizedEfficiency{};
};

class LinearUcbRanker {
public:
    static constexpr std::size_t FEATURE_COUNT = 9;
    using FeatureVector = std::array<double, FEATURE_COUNT>;

    LinearUcbRanker();

    void reset();
    [[nodiscard]] double score(
        const LocalSearchAllocationContext& context) const;
    void update(
        const LocalSearchAllocationContext& context,
        double reward);

private:
    using Matrix =
        std::array<std::array<double, FEATURE_COUNT>, FEATURE_COUNT>;

    Matrix gram{};
    FeatureVector targets{};

    [[nodiscard]] static FeatureVector features(
        const LocalSearchAllocationContext& context);
    [[nodiscard]] static FeatureVector solve(
        const Matrix& matrix,
        const FeatureVector& target);
    [[nodiscard]] static double dot(
        const FeatureVector& first,
        const FeatureVector& second);
};

class ContextualLocalSearchAllocator {
public:
    void reset();

    [[nodiscard]] std::vector<double> score_medium(
        const std::vector<LocalSearchAllocationContext>& contexts) const;
    [[nodiscard]] std::vector<double> score_strong(
        const std::vector<LocalSearchAllocationContext>& contexts) const;

    void update_medium(
        const LocalSearchAllocationContext& context,
        double reward);
    void update_strong(
        const LocalSearchAllocationContext& context,
        double reward);

private:
    LinearUcbRanker mediumRanker;
    LinearUcbRanker strongRanker;
};

struct LocalSearchAllocationStats {
    int selections{};
    int terminalCount{};
    int acceptedMoves{};
    int neighborhoodCalls{};
    std::uint64_t distanceCalls{};
    double upperGain{};
    int gammaCrosses{};
    int parentPoolHits{};
    int globalUpperUpdates{};
    int verifiedUpdates{};
    double reward{};
};

struct AllocatedLocalSearchRecord {
    std::shared_ptr<Individual> individual;
    LocalSearchSession session;
    LocalSearchIntensity terminalIntensity{LocalSearchIntensity::Weak};
    LocalSearchResult weakResult;
    LocalSearchResult mediumResult;
    LocalSearchResult strongResult;
    LocalSearchAllocationContext mediumContext;
    LocalSearchAllocationContext strongContext;
    double costBeforeWeak{};
    double costAfterWeak{};
    double costAfterMedium{};
    double costAfterStrong{};
    bool selectedForMedium{};
    bool selectedForStrong{};
    bool parentPoolHit{};
    bool verifiedUpdate{};
    double relativeVerifiedImprovement{};
};

struct LocalSearchAllocationRun {
    std::vector<AllocatedLocalSearchRecord> records;
    std::array<LocalSearchAllocationStats, 3> stats{};
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT> operatorStats{};
};

class LocalSearchAllocationRunner {
public:
    static LocalSearchAllocationRun run(
        const std::vector<std::shared_ptr<Individual>>& population,
        const std::shared_ptr<Individual>& unchangedLowerElite,
        Case& instance,
        int generation,
        LocalSearchPolicy policy,
        const ParentCandidate& upperReference,
        double triggerUpperBound,
        double budgetProgress,
        std::mt19937& localSearchEngine,
        std::mt19937& allocationEngine,
        std::vector<LocalSearchWorkspace>& workspaces,
        const ContextualLocalSearchAllocator& allocator);

    static void assign_parent_pool_feedback(
        LocalSearchAllocationRun& run,
        const std::vector<ParentCandidate>& parentPool);
    static void assign_verified_feedback(
        LocalSearchAllocationRun& run,
        const std::shared_ptr<Individual>& verifiedIndividual,
        double previousLowerCost,
        double newLowerCost);
    static void finalize_feedback(
        LocalSearchAllocationRun& run,
        double referenceUpperCost,
        LocalSearchPolicy policy,
        ContextualLocalSearchAllocator& allocator);

    static std::size_t intensity_index(
        LocalSearchIntensity intensity);
    static const char* intensity_name(
        LocalSearchIntensity intensity);
};

const char* local_search_policy_name(LocalSearchPolicy policy);

#endif
