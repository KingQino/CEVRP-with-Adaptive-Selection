#include "elite_unlimited.hpp"

#include <algorithm>
#include <limits>
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

double relative_gain_efficiency(
    double upperCostBefore,
    double upperCostAfter,
    std::uint64_t distanceCalls) {
    if (upperCostBefore <= 0.0
        || upperCostAfter >= upperCostBefore
        || distanceCalls == 0) {
        return 0.0;
    }
    return ((upperCostBefore - upperCostAfter) / upperCostBefore)
        / static_cast<double>(distanceCalls);
}

void append_result(
    LocalSearchResult& destination,
    const LocalSearchResult& source) {
    destination.moveLimit = source.moveLimit;
    destination.acceptedMoves += source.acceptedMoves;
    destination.neighborhoodCalls += source.neighborhoodCalls;
    destination.distanceCallsUsed += source.distanceCallsUsed;
    destination.reachedLocalOptimum = source.reachedLocalOptimum;
    destination.hitMoveLimit = source.hitMoveLimit;
    destination.hitDistanceCallLimit =
        source.hitDistanceCallLimit;
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        auto& total = destination.operatorStats[operatorIndex];
        const auto& addition = source.operatorStats[operatorIndex];
        total.calls += addition.calls;
        total.accepts += addition.accepts;
        total.distanceCalls += addition.distanceCalls;
        total.upperGain += addition.upperGain;
        total.gammaCrosses += addition.gammaCrosses;
    }
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
        || bestCandidate == allocationRun.records.end()) {
        return run;
    }

    const std::uint64_t chunkDistanceCallLimit =
        std::max<std::uint64_t>(
            bestCandidate->continuationResult.distanceCallsUsed,
            std::max<std::uint64_t>(
                bestCandidate->weakResult.distanceCallsUsed,
                1));
    if (creditDistanceCalls
        < static_cast<long double>(chunkDistanceCallLimit)) {
        return run;
    }

    bestCandidate->excludeFromLearnerFeedback = true;
    run.individual = bestCandidate->individual;
    run.upperCostBefore = run.individual->get_upper_cost();
    run.probeEfficiency = relative_gain_efficiency(
        bestCandidate->costAfterWeak,
        bestCandidate->costAfterTerminal,
        bestCandidate->continuationResult.distanceCallsUsed);

    LocalSearchSession session;
    Leader::begin_eight_neighborhood_rvnd_one_move_session(
        *run.individual,
        session,
        workspace);
    const int chunkMoveLimit =
        std::max(1, bestCandidate->boundedStrongMoveLimit);
    double referenceEfficiency = run.probeEfficiency;

    while (run.chunks < MAX_CHUNKS_PER_TRIGGER) {
        if (creditDistanceCalls
            < static_cast<long double>(chunkDistanceCallLimit)) {
            run.stopReason =
                EliteContinuationStopReason::Budget;
            break;
        }

        const double chunkUpperCostBefore =
            run.individual->get_upper_cost();
        const int cumulativeMoveLimit =
            session.totalAcceptedMoves + chunkMoveLimit;
        const std::uint64_t cumulativeDistanceCallLimit =
            session.totalDistanceCalls
                > std::numeric_limits<std::uint64_t>::max()
                    - chunkDistanceCallLimit
            ? std::numeric_limits<std::uint64_t>::max()
            : session.totalDistanceCalls
                + chunkDistanceCallLimit;
        const LocalSearchResult chunkResult =
            Leader::continue_eight_neighborhood_rvnd_one_move_session(
            *run.individual,
            instance,
            randomEngine,
            session,
            cumulativeMoveLimit,
            workspace,
            triggerUpperBound,
            cumulativeDistanceCallLimit);
        ++run.chunks;
        append_result(run.result, chunkResult);
        creditDistanceCalls -= static_cast<long double>(
            chunkResult.distanceCallsUsed);

        const double chunkEfficiency =
            relative_gain_efficiency(
                chunkUpperCostBefore,
                run.individual->get_upper_cost(),
                chunkResult.distanceCallsUsed);
        if (referenceEfficiency <= 0.0
            && chunkEfficiency > 0.0) {
            referenceEfficiency = chunkEfficiency;
        }
        const double efficiencyRatio =
            referenceEfficiency > 0.0
            ? chunkEfficiency / referenceEfficiency
            : 0.0;
        run.chunkEfficiencyRatio += efficiencyRatio;
        ++run.chunkEfficiencySamples;

        if (chunkResult.reachedLocalOptimum) {
            run.stopReason =
                EliteContinuationStopReason::LocalOptimum;
            break;
        }
        if (chunkResult.acceptedMoves == 0
            || chunkEfficiency <= 0.0
            || (referenceEfficiency > 0.0
                && efficiencyRatio
                    < MIN_EFFICIENCY_RATIO)) {
            run.stopReason =
                EliteContinuationStopReason::LowEfficiency;
            break;
        }
    }

    if (run.stopReason == EliteContinuationStopReason::None) {
        run.stopReason =
            EliteContinuationStopReason::ChunkCap;
    }
    run.upperCostAfter = run.individual->get_upper_cost();
    run.triggered = true;
    run.crossedGamma =
        run.upperCostBefore > triggerUpperBound
        && run.upperCostAfter <= triggerUpperBound;
    run.result.relativeUpperImprovement =
        run.upperCostBefore > 0.0
        ? (run.upperCostBefore - run.upperCostAfter)
            / run.upperCostBefore
        : 0.0;
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
    stats.chunks = run.chunks;
    stats.localOptimumStops =
        run.stopReason
            == EliteContinuationStopReason::LocalOptimum;
    stats.lowEfficiencyStops =
        run.stopReason
            == EliteContinuationStopReason::LowEfficiency;
    stats.budgetStops =
        run.stopReason
            == EliteContinuationStopReason::Budget;
    stats.chunkCapStops =
        run.stopReason
            == EliteContinuationStopReason::ChunkCap;
    stats.normalDistanceCalls = normalDistanceCalls;
    stats.distanceCalls = run.result.distanceCallsUsed;
    stats.acceptedMoves = run.result.acceptedMoves;
    stats.neighborhoodCalls = run.result.neighborhoodCalls;
    stats.upperGain = result_upper_gain(run.result);
    stats.gammaCrosses = result_gamma_crosses(run.result);
    stats.parentUses = run.parentUses;
    stats.lowerArchiveEntries = run.lowerArchiveEntries;
    stats.verifiedImprovements = run.verifiedImprovements;
    stats.probeEfficiency = run.probeEfficiency;
    stats.chunkEfficiencyRatio =
        run.chunkEfficiencyRatio;
    stats.chunkEfficiencySamples =
        run.chunkEfficiencySamples;
    stats.endingCreditDistanceCalls = endingCreditDistanceCalls;
    return stats;
}

long double EliteUnlimitedController::credit_distance_calls() const {
    return creditDistanceCalls;
}
