#include "elite_unlimited.hpp"

#include <algorithm>
#include <cmath>
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

void EliteUnlimitedController::set_credit_ratio(double newCreditRatio) {
    if (!std::isfinite(newCreditRatio)
        || newCreditRatio < 0.0
        || newCreditRatio > 1.0) {
        throw std::invalid_argument(
            "elite credit ratio must be in [0, 1]");
    }
    creditRatio = newCreditRatio;
}

double EliteUnlimitedController::credit_ratio() const {
    return creditRatio;
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
        static_cast<long double>(creditRatio)
        * static_cast<long double>(normalDistanceCalls);

    EliteUnlimitedRun run;
    auto bestCandidate = allocationRun.records.end();
    for (auto record = allocationRun.records.begin();
         record != allocationRun.records.end();
         ++record) {
        const bool eligible =
            record->terminalIntensity
                == LocalSearchIntensity::BoundedStrong
            && !record->totalResult.reachedLocalOptimum;
        if (!eligible) {
            continue;
        }
        ++run.eligibleCandidates;
        if (bestCandidate == allocationRun.records.end()
            || record->individual->get_upper_cost()
                < bestCandidate->individual->get_upper_cost()) {
            bestCandidate = record;
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
    stats.triggers = run.triggered;
    stats.normalDistanceCalls = normalDistanceCalls;
    stats.distanceCalls = run.result.distanceCallsUsed;
    stats.acceptedMoves = run.result.acceptedMoves;
    stats.neighborhoodCalls = run.result.neighborhoodCalls;
    stats.upperGain = result_upper_gain(run.result);
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
