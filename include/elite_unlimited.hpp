#ifndef ELITE_UNLIMITED_HPP
#define ELITE_UNLIMITED_HPP

#include <cstdint>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "leader.hpp"

class Case;
class Individual;
struct LocalSearchAllocationRun;
struct ParentCandidate;

struct EliteUnlimitedStats {
    int eligibleCandidates{};
    int triggers{};
    int chunks{};
    int localOptimumStops{};
    int lowEfficiencyStops{};
    int budgetStops{};
    int chunkCapStops{};
    std::uint64_t normalDistanceCalls{};
    std::uint64_t distanceCalls{};
    int acceptedMoves{};
    int neighborhoodCalls{};
    double upperGain{};
    int gammaCrosses{};
    int parentUses{};
    int lowerArchiveEntries{};
    int verifiedImprovements{};
    double probeEfficiency{};
    double chunkEfficiencyRatio{};
    int chunkEfficiencySamples{};
    long double endingCreditDistanceCalls{};
};

enum class EliteContinuationStopReason {
    None,
    LocalOptimum,
    LowEfficiency,
    Budget,
    ChunkCap,
};

struct EliteUnlimitedRun {
    std::shared_ptr<Individual> individual;
    LocalSearchResult result;
    int eligibleCandidates{};
    int chunks{};
    double upperCostBefore{};
    double upperCostAfter{};
    double probeEfficiency{};
    double chunkEfficiencyRatio{};
    int chunkEfficiencySamples{};
    int parentUses{};
    int lowerArchiveEntries{};
    int verifiedImprovements{};
    bool triggered{};
    bool crossedGamma{};
    EliteContinuationStopReason stopReason{
        EliteContinuationStopReason::None};
};

class EliteUnlimitedController {
public:
    static constexpr int WARMUP_GENERATIONS = 10;
    static constexpr long double CREDIT_RATIO = 0.10L;
    static constexpr int MAX_CHUNKS_PER_TRIGGER = 4;
    static constexpr double MIN_EFFICIENCY_RATIO = 0.25;

    void reset();

    EliteUnlimitedRun run(
        LocalSearchAllocationRun& allocationRun,
        Case& instance,
        int generation,
        std::uint64_t normalDistanceCalls,
        double triggerUpperBound,
        std::mt19937& randomEngine,
        LocalSearchWorkspace& workspace);

    static void assign_parent_use_feedback(
        EliteUnlimitedRun& run,
        const std::vector<ParentCandidate>& parentPool,
        const std::vector<int>& parentUseCounts);
    static void assign_lower_archive_feedback(
        EliteUnlimitedRun& run,
        const std::vector<std::pair<const Individual*, double>>& credits);
    static EliteUnlimitedStats make_stats(
        const EliteUnlimitedRun& run,
        std::uint64_t normalDistanceCalls,
        long double endingCreditDistanceCalls);

    [[nodiscard]] long double credit_distance_calls() const;

private:
    long double creditDistanceCalls{};
};

#endif
