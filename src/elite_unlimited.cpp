#include "elite_unlimited.hpp"

#include <algorithm>
#include <stdexcept>

#include "case.hpp"
#include "individual.hpp"
#include "local_search_allocation.hpp"
#include "reproduction.hpp"

namespace {

double result_upper_gain(const LocalSearchResult& result) {
    double gain = 0.0;
    for (const auto& stats : result.operatorStats) {
        gain += stats.upperGain;
    }
    return gain;
}

int result_gamma_crosses(const LocalSearchResult& result) {
    int crosses = 0;
    for (const auto& stats : result.operatorStats) {
        crosses += stats.gammaCrosses;
    }
    return crosses;
}

}  // namespace

void EliteUnlimitedController::reset() {
    creditDistanceCalls = 0.0L;
}

EliteUnlimitedRun EliteUnlimitedController::run(
    LocalSearchAllocationRun& allocationRun,
    Case& instance,
    int generation,
    std::uint64_t normalDistanceCalls,
    double triggerUpperBound,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    creditDistanceCalls +=
        CREDIT_RATIO * static_cast<long double>(normalDistanceCalls);

    EliteUnlimitedRun run;
    std::vector<std::size_t> eligibleCandidateIndices;
    eligibleCandidateIndices.reserve(allocationRun.records.size());
    for (std::size_t index = 0;
         index < allocationRun.records.size();
         ++index) {
        const auto& record = allocationRun.records[index];
        const bool eligible =
            record.terminalIntensity
                == LocalSearchIntensity::BoundedStrong
            && !record.totalResult.reachedLocalOptimum;
        if (!eligible) {
            continue;
        }
        eligibleCandidateIndices.push_back(index);
    }
    run.eligibleCandidates =
        static_cast<int>(eligibleCandidateIndices.size());

    std::stable_sort(
        eligibleCandidateIndices.begin(),
        eligibleCandidateIndices.end(),
        [&](std::size_t first, std::size_t second) {
            return allocationRun.records[first]
                       .individual->get_upper_cost()
                < allocationRun.records[second]
                       .individual->get_upper_cost();
        });

    const std::size_t qualityCandidateCount =
        (eligibleCandidateIndices.size() + 1) / 2;
    run.qualityCandidates =
        static_cast<int>(qualityCandidateCount);

    auto bestCandidate = allocationRun.records.end();
    long double bestEfficiency = -1.0L;
    double bestDistance = -1.0;
    for (std::size_t rank = 0;
         rank < qualityCandidateCount;
         ++rank) {
        auto candidate = allocationRun.records.begin()
            + static_cast<std::ptrdiff_t>(
                eligibleCandidateIndices[rank]);
        const double probeGain =
            result_upper_gain(candidate->continuationResult);
        const std::uint64_t probeDistanceCalls =
            candidate->continuationResult.distanceCallsUsed;
        const long double efficiency =
            probeDistanceCalls > 0
            ? static_cast<long double>(probeGain)
                / static_cast<long double>(probeDistanceCalls)
            : 0.0L;
        const double adjacencyDistance =
            candidate->context.adjacencyDistance;
        if (bestCandidate == allocationRun.records.end()
            || efficiency > bestEfficiency
            || (efficiency == bestEfficiency
                && adjacencyDistance > bestDistance)) {
            bestCandidate = candidate;
            bestEfficiency = efficiency;
            bestDistance = adjacencyDistance;
            run.selectedQualityRank =
                static_cast<int>(rank + 1);
            run.selectedProbeDistanceCalls =
                probeDistanceCalls;
            run.selectedProbeUpperGain = probeGain;
            run.selectedAdjacencyDistance =
                adjacencyDistance;
        }
    }

    if (generation <= WARMUP_GENERATIONS
        || creditDistanceCalls <= 0.0L
        || bestCandidate == allocationRun.records.end()) {
        return run;
    }

    bestCandidate->excludeFromLearnerFeedback = true;
    run.individual = bestCandidate->individual;
    run.upperCostBefore = run.individual->get_upper_cost();
    run.result =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            *run.individual,
            instance,
            randomEngine,
            LocalSearchIntensity::Strong,
            workspace,
            triggerUpperBound);
    run.upperCostAfter = run.individual->get_upper_cost();
    run.triggered = true;
    run.crossedGamma =
        run.upperCostBefore > triggerUpperBound
        && run.upperCostAfter <= triggerUpperBound;
    creditDistanceCalls -=
        static_cast<long double>(run.result.distanceCallsUsed);
    return run;
}

void EliteUnlimitedController::assign_parent_use_feedback(
    EliteUnlimitedRun& run,
    const std::vector<ParentCandidate>& parentPool,
    const std::vector<int>& parentUseCounts) {
    if (!run.triggered) {
        return;
    }
    if (parentPool.size() != parentUseCounts.size()) {
        throw std::invalid_argument(
            "parent use counts must match parent pool size");
    }
    for (std::size_t index = 0; index < parentPool.size(); ++index) {
        if (parentPool[index].source == run.individual.get()) {
            run.parentUses += parentUseCounts[index];
        }
    }
}

void EliteUnlimitedController::assign_lower_archive_feedback(
    EliteUnlimitedRun& run,
    const std::vector<std::pair<const Individual*, double>>& credits) {
    if (!run.triggered) {
        return;
    }
    run.lowerArchiveEntries += static_cast<int>(std::count_if(
        credits.begin(),
        credits.end(),
        [&](const auto& entry) {
            return entry.first == run.individual.get()
                && entry.second > 0.0;
        }));
}

EliteUnlimitedStats EliteUnlimitedController::make_stats(
    const EliteUnlimitedRun& run,
    std::uint64_t normalDistanceCalls,
    long double endingCreditDistanceCalls) {
    EliteUnlimitedStats stats;
    stats.eligibleCandidates = run.eligibleCandidates;
    stats.qualityCandidates = run.qualityCandidates;
    stats.triggers = run.triggered;
    stats.selectedQualityRank =
        run.triggered ? run.selectedQualityRank : 0;
    stats.normalDistanceCalls = normalDistanceCalls;
    stats.distanceCalls = run.result.distanceCallsUsed;
    stats.selectedProbeDistanceCalls =
        run.triggered ? run.selectedProbeDistanceCalls : 0;
    stats.acceptedMoves = run.result.acceptedMoves;
    stats.neighborhoodCalls = run.result.neighborhoodCalls;
    stats.upperGain = result_upper_gain(run.result);
    stats.selectedProbeUpperGain =
        run.triggered ? run.selectedProbeUpperGain : 0.0;
    stats.selectedAdjacencyDistance =
        run.triggered ? run.selectedAdjacencyDistance : 0.0;
    stats.gammaCrosses = result_gamma_crosses(run.result);
    stats.parentUses = run.parentUses;
    stats.lowerArchiveEntries = run.lowerArchiveEntries;
    stats.verifiedImprovements = run.verifiedImprovements;
    stats.endingCreditDistanceCalls = endingCreditDistanceCalls;
    return stats;
}

long double EliteUnlimitedController::credit_distance_calls() const {
    return creditDistanceCalls;
}
