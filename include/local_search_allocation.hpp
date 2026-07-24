#ifndef LOCAL_SEARCH_ALLOCATION_HPP
#define LOCAL_SEARCH_ALLOCATION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "leader.hpp"
#include "reproduction.hpp"

class Case;
class Individual;

enum class LocalSearchPolicy {
    Static,
    RandomMixed,
    OnlineIndividual,
};

struct LocalSearchAllocationContext {
    double qualityGap{};
    double adjacencyDistance{};
    double budgetProgress{};
    double gammaMargin{};
    double upperStagnation{};
    double lowerStagnation{};
    double populationDispersion{};
    double recentGammaRate{};
    double probeSuccessRate{};
    double probeRelativeGain{};
    double probeEfficiency{};
    double probeCost{};
};

struct LinearUcbEstimate {
    double prediction{};
    double uncertainty{};
};

class LinearUcbModel {
public:
    static constexpr std::size_t FEATURE_COUNT = 14;
    using FeatureVector = std::array<double, FEATURE_COUNT>;

    LinearUcbModel();

    void reset();
    [[nodiscard]] LinearUcbEstimate estimate(
        const LocalSearchAllocationContext& context) const;
    void update(
        const LocalSearchAllocationContext& context,
        double target);

private:
    using Matrix =
        std::array<std::array<double, FEATURE_COUNT>, FEATURE_COUNT>;

    Matrix inverseGram{};
    FeatureVector targets{};
    FeatureVector coefficients{};

    [[nodiscard]] static FeatureVector features(
        const LocalSearchAllocationContext& context);
    [[nodiscard]] static FeatureVector multiply(
        const Matrix& matrix,
        const FeatureVector& vector);
    [[nodiscard]] static double dot(
        const FeatureVector& first,
        const FeatureVector& second);
};

struct LocalSearchIntensityDecision {
    LocalSearchIntensity intensity{LocalSearchIntensity::Weak};
    double score{};
    bool exploratory{};
};

class OnlineIntensityLearner {
public:
    static constexpr std::size_t LOWER_ARCHIVE_CAPACITY = 10;

    void reset();

    [[nodiscard]] LocalSearchIntensityDecision select(
        const LocalSearchAllocationContext& context,
        int generation,
        std::mt19937& randomEngine) const;
    void update(
        const LocalSearchAllocationContext& context,
        LocalSearchIntensity intensity,
        double utility,
        double costUnits);
    [[nodiscard]] std::vector<std::pair<const Individual*, double>>
    update_lower_archive(
        const std::vector<std::shared_ptr<Individual>>& completeSolutions);

    [[nodiscard]] double score(
        const LocalSearchAllocationContext& context,
        LocalSearchIntensity intensity) const;
    [[nodiscard]] int observation_count(
        LocalSearchIntensity intensity) const;

private:
    struct LowerArchiveEntry {
        std::vector<int> chromosome;
        double lowerCost{};
    };

    std::array<LinearUcbModel, 3> utilityModels;
    std::array<LinearUcbModel, 3> logCostModels;
    std::array<int, 3> observationCounts{};
    std::vector<LowerArchiveEntry> lowerArchive;
};

struct LocalSearchAllocationStats {
    int selections{};
    int forcedLocalOptima{};
    int exploratorySelections{};
    int acceptedMoves{};
    int neighborhoodCalls{};
    std::uint64_t distanceCalls{};
    double upperGain{};
    int gammaCrosses{};
    int parentUses{};
    int lowerArchiveEntries{};
    double utility{};
    double costUnits{};
    double selectionScore{};
};

struct AllocatedLocalSearchRecord {
    std::shared_ptr<Individual> individual;
    LocalSearchSession session;
    LocalSearchIntensity terminalIntensity{LocalSearchIntensity::Weak};
    LocalSearchResult weakResult;
    LocalSearchResult continuationResult;
    LocalSearchResult totalResult;
    LocalSearchAllocationContext context;
    double costBeforeWeak{};
    double costAfterWeak{};
    double costAfterTerminal{};
    double selectionScore{};
    double parentCredit{};
    double lowerCredit{};
    double utility{};
    double costUnits{1.0};
    int parentUseCount{};
    bool forcedLocalOptimum{};
    bool exploratorySelection{};
    bool crossedGamma{};
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
        double upperStagnation,
        double lowerStagnation,
        double populationDispersion,
        double recentGammaRate,
        std::mt19937& localSearchEngine,
        std::mt19937& allocationEngine,
        std::vector<LocalSearchWorkspace>& workspaces,
        const OnlineIntensityLearner& learner);

    static void assign_parent_use_feedback(
        LocalSearchAllocationRun& run,
        const std::vector<ParentCandidate>& parentPool,
        const std::vector<int>& parentUseCounts);
    static void assign_lower_archive_feedback(
        LocalSearchAllocationRun& run,
        const std::vector<std::shared_ptr<Individual>>& completeSolutions,
        OnlineIntensityLearner& learner);
    static void finalize_feedback(
        LocalSearchAllocationRun& run,
        LocalSearchPolicy policy,
        OnlineIntensityLearner& learner);

    static std::size_t intensity_index(
        LocalSearchIntensity intensity);
    static const char* intensity_name(
        LocalSearchIntensity intensity);
};

const char* local_search_policy_name(LocalSearchPolicy policy);

#endif
