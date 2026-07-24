#include "local_search_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include "algorithm_constants.hpp"
#include "case.hpp"
#include "individual.hpp"

namespace {

constexpr double kRidge = 1.0;
constexpr double kExplorationScale = 0.10;
constexpr std::size_t kMixedBatchSize = 10;
constexpr double kMediumSelectionRatio = 0.80;
constexpr double kStrongSelectionRatio = 0.50;

double bounded_nonnegative(double value) {
    const double nonnegative = std::max(0.0, value);
    return nonnegative / (1.0 + nonnegative);
}

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

void add_operator_stats(
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT>& generationStats,
    const LocalSearchResult& result) {
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        auto& destination = generationStats[operatorIndex];
        const auto& source = result.operatorStats[operatorIndex];
        destination.calls += source.calls;
        destination.accepts += source.accepts;
        destination.distanceCalls += source.distanceCalls;
        destination.upperGain += source.upperGain;
        destination.gammaCrosses += source.gammaCrosses;
    }
}

void add_allocation_stats(
    LocalSearchAllocationStats& stats,
    const LocalSearchResult& result) {
    ++stats.selections;
    stats.acceptedMoves += result.acceptedMoves;
    stats.neighborhoodCalls += result.neighborhoodCalls;
    stats.distanceCalls += result.distanceCallsUsed;
    stats.upperGain += result_upper_gain(result);
    stats.gammaCrosses += result_gamma_crosses(result);
}

double raw_efficiency(const LocalSearchResult& result) {
    if (result.distanceCallsUsed == 0) {
        return 0.0;
    }
    return result.relativeUpperImprovement
        / static_cast<double>(result.distanceCallsUsed);
}

LocalSearchAllocationContext make_allocation_context(
    const Individual& individual,
    const ParentCandidate& reference,
    double triggerUpperBound,
    double budgetProgress,
    const LocalSearchResult& probeResult,
    double normalizedEfficiency) {
    const ParentCandidate candidate =
        Reproduction::make_parent_candidate(individual);
    LocalSearchAllocationContext context;
    context.qualityGap = reference.upperCost > 0.0
        ? (candidate.upperCost - reference.upperCost)
            / reference.upperCost
        : 0.0;
    context.adjacencyDistance = Reproduction::adjacency_distance(
        candidate,
        reference);
    context.budgetProgress = budgetProgress;
    context.gammaMargin = triggerUpperBound > 0.0
        ? (candidate.upperCost - triggerUpperBound)
            / triggerUpperBound
        : 0.0;
    context.successRate = probeResult.neighborhoodCalls > 0
        ? static_cast<double>(probeResult.acceptedMoves)
            / static_cast<double>(probeResult.neighborhoodCalls)
        : 0.0;
    context.normalizedEfficiency = normalizedEfficiency;
    return context;
}

double useful_stage_reward(
    double costBefore,
    double costAfter,
    double referenceCost,
    bool crossedGamma,
    bool parentPoolHit,
    bool globalUpperUpdate,
    bool verifiedUpdate,
    double relativeVerifiedImprovement) {
    const double relativeImprovement = costBefore > 0.0
        ? std::max(0.0, (costBefore - costAfter) / costBefore)
        : 0.0;
    const double qualityGap = referenceCost > 0.0
        ? std::max(0.0, (costAfter - referenceCost) / referenceCost)
        : 0.0;
    const double competitiveImprovement =
        relativeImprovement * std::exp(-10.0 * qualityGap);
    return std::clamp(
        competitiveImprovement
            + (crossedGamma ? 0.05 : 0.0)
            + (parentPoolHit ? 0.02 : 0.0)
            + (globalUpperUpdate ? 0.05 : 0.0)
            + (verifiedUpdate ? 0.10 : 0.0)
            + std::clamp(relativeVerifiedImprovement, 0.0, 0.10),
        0.0,
        1.0);
}

}  // namespace

LinearUcbRanker::LinearUcbRanker() {
    reset();
}

void LinearUcbRanker::reset() {
    gram = {};
    targets = {};
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        gram[feature][feature] = kRidge;
    }
}

double LinearUcbRanker::score(
    const LocalSearchAllocationContext& context) const {
    const FeatureVector contextFeatures = features(context);
    const FeatureVector coefficients = solve(gram, targets);
    const FeatureVector projectedContext = solve(gram, contextFeatures);
    const double prediction = dot(coefficients, contextFeatures);
    const double uncertainty = std::sqrt(std::max(
        0.0,
        dot(contextFeatures, projectedContext)));
    return prediction + kExplorationScale * uncertainty;
}

void LinearUcbRanker::update(
    const LocalSearchAllocationContext& context,
    double reward) {
    const FeatureVector contextFeatures = features(context);
    const double boundedReward = std::clamp(reward, 0.0, 1.0);
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            gram[row][column] +=
                contextFeatures[row] * contextFeatures[column];
        }
        targets[row] += contextFeatures[row] * boundedReward;
    }
}

LinearUcbRanker::FeatureVector LinearUcbRanker::features(
    const LocalSearchAllocationContext& context) {
    const double qualityGap = bounded_nonnegative(context.qualityGap);
    const double adjacencyDistance = std::clamp(
        context.adjacencyDistance,
        0.0,
        1.0);
    const double budgetProgress = std::clamp(
        context.budgetProgress,
        0.0,
        1.0);
    return {
        1.0,
        qualityGap,
        adjacencyDistance,
        budgetProgress,
        std::tanh(context.gammaMargin),
        std::clamp(context.successRate, 0.0, 1.0),
        std::clamp(context.normalizedEfficiency, 0.0, 1.0),
        qualityGap * adjacencyDistance,
        budgetProgress * adjacencyDistance,
    };
}

LinearUcbRanker::FeatureVector LinearUcbRanker::solve(
    const Matrix& matrix,
    const FeatureVector& target) {
    std::array<
        std::array<double, FEATURE_COUNT + 1>,
        FEATURE_COUNT> augmented{};
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][FEATURE_COUNT] = target[row];
    }

    for (std::size_t pivot = 0; pivot < FEATURE_COUNT; ++pivot) {
        std::size_t pivotRow = pivot;
        for (std::size_t row = pivot + 1;
             row < FEATURE_COUNT;
             ++row) {
            if (std::fabs(augmented[row][pivot])
                > std::fabs(augmented[pivotRow][pivot])) {
                pivotRow = row;
            }
        }
        if (std::fabs(augmented[pivotRow][pivot]) <= 1e-14) {
            return {};
        }
        if (pivotRow != pivot) {
            std::swap(augmented[pivotRow], augmented[pivot]);
        }

        const double pivotValue = augmented[pivot][pivot];
        for (std::size_t column = pivot;
             column <= FEATURE_COUNT;
             ++column) {
            augmented[pivot][column] /= pivotValue;
        }
        for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
            if (row == pivot) {
                continue;
            }
            const double factor = augmented[row][pivot];
            for (std::size_t column = pivot;
                 column <= FEATURE_COUNT;
                 ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot][column];
            }
        }
    }

    FeatureVector solution{};
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        solution[row] = augmented[row][FEATURE_COUNT];
    }
    return solution;
}

double LinearUcbRanker::dot(
    const FeatureVector& first,
    const FeatureVector& second) {
    double result = 0.0;
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        result += first[feature] * second[feature];
    }
    return result;
}

void ContextualLocalSearchAllocator::reset() {
    mediumRanker.reset();
    strongRanker.reset();
}

std::vector<double> ContextualLocalSearchAllocator::score_medium(
    const std::vector<LocalSearchAllocationContext>& contexts) const {
    std::vector<double> scores;
    scores.reserve(contexts.size());
    for (const auto& context : contexts) {
        scores.push_back(mediumRanker.score(context));
    }
    return scores;
}

std::vector<double> ContextualLocalSearchAllocator::score_strong(
    const std::vector<LocalSearchAllocationContext>& contexts) const {
    std::vector<double> scores;
    scores.reserve(contexts.size());
    for (const auto& context : contexts) {
        scores.push_back(strongRanker.score(context));
    }
    return scores;
}

void ContextualLocalSearchAllocator::update_medium(
    const LocalSearchAllocationContext& context,
    double reward) {
    mediumRanker.update(context, reward);
}

void ContextualLocalSearchAllocator::update_strong(
    const LocalSearchAllocationContext& context,
    double reward) {
    strongRanker.update(context, reward);
}

LocalSearchAllocationRun LocalSearchAllocationRunner::run(
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
    const ContextualLocalSearchAllocator& allocator) {
    LocalSearchAllocationRun run;
    run.records.reserve(population.size());
    const std::size_t requiredWorkspaces = std::min(
        kMixedBatchSize,
        population.size());
    if (workspaces.size() < requiredWorkspaces) {
        workspaces.resize(requiredWorkspaces);
    }

    std::vector<std::shared_ptr<Individual>> searchCandidates;
    searchCandidates.reserve(population.size());
    for (const auto& individual : population) {
        const bool canReuseLowerElite =
            individual == unchangedLowerElite
            && individual->is_upper_locally_optimal()
            && individual->get_lower_cost() < INFEASIBLE_COST;
        if (!canReuseLowerElite) {
            searchCandidates.push_back(individual);
        }
    }

    auto select_records = [&](
        std::vector<std::size_t> eligibleRecords,
        std::size_t selectionCount,
        bool selectStrong) {
        selectionCount = std::min(
            selectionCount,
            eligibleRecords.size());
        std::shuffle(
            eligibleRecords.begin(),
            eligibleRecords.end(),
            allocationEngine);
        if (policy == LocalSearchPolicy::ContextualMixed
            && generation > 1
            && !eligibleRecords.empty()) {
            std::vector<LocalSearchAllocationContext> contexts;
            contexts.reserve(eligibleRecords.size());
            for (const std::size_t recordIndex : eligibleRecords) {
                contexts.push_back(
                    selectStrong
                        ? run.records[recordIndex].strongContext
                        : run.records[recordIndex].mediumContext);
            }
            const std::vector<double> scores = selectStrong
                ? allocator.score_strong(contexts)
                : allocator.score_medium(contexts);
            std::vector<std::size_t> scoreOrder(
                eligibleRecords.size());
            std::iota(
                scoreOrder.begin(),
                scoreOrder.end(),
                0);
            std::stable_sort(
                scoreOrder.begin(),
                scoreOrder.end(),
                [&](std::size_t first, std::size_t second) {
                    return scores[first] > scores[second];
                });
            std::vector<std::size_t> rankedRecords;
            rankedRecords.reserve(eligibleRecords.size());
            for (const std::size_t scoreIndex : scoreOrder) {
                rankedRecords.push_back(
                    eligibleRecords[scoreIndex]);
            }
            eligibleRecords.swap(rankedRecords);
        }
        eligibleRecords.resize(selectionCount);
        return eligibleRecords;
    };

    for (std::size_t batchStart = 0;
         batchStart < searchCandidates.size();
         batchStart += kMixedBatchSize) {
        const std::size_t batchSize = std::min(
            kMixedBatchSize,
            searchCandidates.size() - batchStart);
        const std::size_t recordStart = run.records.size();

        double maximumWeakEfficiency = 0.0;
        for (std::size_t localIndex = 0;
             localIndex < batchSize;
             ++localIndex) {
            AllocatedLocalSearchRecord record;
            record.individual =
                searchCandidates[batchStart + localIndex];
            record.costBeforeWeak =
                record.individual->get_upper_cost();
            Leader::begin_eight_neighborhood_rvnd_one_move_session(
                *record.individual,
                record.session,
                workspaces[localIndex]);
            const int weakLimit =
                Leader::move_limit_for_intensity(
                    *record.individual,
                    instance,
                    LocalSearchIntensity::Weak);
            record.weakResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    weakLimit,
                    workspaces[localIndex],
                    triggerUpperBound);
            record.costAfterWeak =
                record.individual->get_upper_cost();
            record.costAfterMedium = record.costAfterWeak;
            record.costAfterStrong = record.costAfterWeak;
            maximumWeakEfficiency = std::max(
                maximumWeakEfficiency,
                raw_efficiency(record.weakResult));
            add_operator_stats(
                run.operatorStats,
                record.weakResult);
            add_allocation_stats(
                run.stats[
                    intensity_index(LocalSearchIntensity::Weak)],
                record.weakResult);
            run.records.push_back(std::move(record));
        }

        std::vector<std::size_t> mediumEligible;
        for (std::size_t localIndex = 0;
             localIndex < batchSize;
             ++localIndex) {
            const std::size_t recordIndex =
                recordStart + localIndex;
            auto& record = run.records[recordIndex];
            const double normalizedEfficiency =
                maximumWeakEfficiency > 0.0
                ? raw_efficiency(record.weakResult)
                    / maximumWeakEfficiency
                : 0.0;
            record.mediumContext = make_allocation_context(
                *record.individual,
                upperReference,
                triggerUpperBound,
                budgetProgress,
                record.weakResult,
                normalizedEfficiency);
            if (!record.weakResult.reachedLocalOptimum) {
                mediumEligible.push_back(recordIndex);
            }
        }

        const std::size_t mediumTarget =
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batchSize)
                * kMediumSelectionRatio));
        const std::vector<std::size_t> mediumSelected =
            select_records(
                mediumEligible,
                mediumTarget,
                false);
        double maximumMediumEfficiency = 0.0;
        for (const std::size_t recordIndex : mediumSelected) {
            const std::size_t localIndex =
                recordIndex - recordStart;
            auto& record = run.records[recordIndex];
            record.selectedForMedium = true;
            record.terminalIntensity =
                LocalSearchIntensity::Medium;
            const int mediumLimit =
                Leader::move_limit_for_intensity(
                    *record.individual,
                    instance,
                    LocalSearchIntensity::Medium);
            record.mediumResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    mediumLimit,
                    workspaces[localIndex],
                    triggerUpperBound);
            record.costAfterMedium =
                record.individual->get_upper_cost();
            record.costAfterStrong = record.costAfterMedium;
            maximumMediumEfficiency = std::max(
                maximumMediumEfficiency,
                raw_efficiency(record.mediumResult));
            add_operator_stats(
                run.operatorStats,
                record.mediumResult);
            add_allocation_stats(
                run.stats[
                    intensity_index(
                        LocalSearchIntensity::Medium)],
                record.mediumResult);
        }

        std::vector<std::size_t> strongEligible;
        for (const std::size_t recordIndex : mediumSelected) {
            auto& record = run.records[recordIndex];
            const double normalizedEfficiency =
                maximumMediumEfficiency > 0.0
                ? raw_efficiency(record.mediumResult)
                    / maximumMediumEfficiency
                : 0.0;
            record.strongContext = make_allocation_context(
                *record.individual,
                upperReference,
                triggerUpperBound,
                budgetProgress,
                record.mediumResult,
                normalizedEfficiency);
            if (!record.mediumResult.reachedLocalOptimum) {
                strongEligible.push_back(recordIndex);
            }
        }

        const std::size_t strongTarget =
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batchSize)
                * kStrongSelectionRatio));
        const std::vector<std::size_t> strongSelected =
            select_records(
                strongEligible,
                strongTarget,
                true);
        for (const std::size_t recordIndex : strongSelected) {
            const std::size_t localIndex =
                recordIndex - recordStart;
            auto& record = run.records[recordIndex];
            record.selectedForStrong = true;
            record.terminalIntensity =
                LocalSearchIntensity::Strong;
            record.strongResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    -1,
                    workspaces[localIndex],
                    triggerUpperBound);
            record.costAfterStrong =
                record.individual->get_upper_cost();
            add_operator_stats(
                run.operatorStats,
                record.strongResult);
            add_allocation_stats(
                run.stats[
                    intensity_index(
                        LocalSearchIntensity::Strong)],
                record.strongResult);
        }
    }

    return run;
}

void LocalSearchAllocationRunner::assign_parent_pool_feedback(
    LocalSearchAllocationRun& run,
    const std::vector<ParentCandidate>& parentPool) {
    for (const ParentCandidate& parent : parentPool) {
        const auto selectedRecord = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& record) {
                return !record.parentPoolHit
                    && record.individual->get_chromosome()
                        == parent.chromosome;
            });
        if (selectedRecord != run.records.end()) {
            selectedRecord->parentPoolHit = true;
            ++run.stats[
                intensity_index(selectedRecord->terminalIntensity)]
                .parentPoolHits;
        }
    }
}

void LocalSearchAllocationRunner::assign_verified_feedback(
    LocalSearchAllocationRun& run,
    const std::shared_ptr<Individual>& verifiedIndividual,
    double previousLowerCost,
    double newLowerCost) {
    const auto verifiedRecord = std::find_if(
        run.records.begin(),
        run.records.end(),
        [&](const AllocatedLocalSearchRecord& record) {
            return record.individual == verifiedIndividual;
        });
    if (verifiedRecord == run.records.end()) {
        return;
    }

    verifiedRecord->verifiedUpdate = true;
    if (previousLowerCost < INFEASIBLE_COST
        && previousLowerCost > 0.0) {
        verifiedRecord->relativeVerifiedImprovement =
            (previousLowerCost - newLowerCost)
            / previousLowerCost;
    }
    ++run.stats[
        intensity_index(verifiedRecord->terminalIntensity)]
        .verifiedUpdates;
}

void LocalSearchAllocationRunner::finalize_feedback(
    LocalSearchAllocationRun& run,
    double referenceUpperCost,
    LocalSearchPolicy policy,
    ContextualLocalSearchAllocator& allocator) {
    for (auto& record : run.records) {
        ++run.stats[
            intensity_index(record.terminalIntensity)]
            .terminalCount;

        const bool weakCrossedGamma =
            result_gamma_crosses(record.weakResult) > 0;
        const bool mediumCrossedGamma =
            result_gamma_crosses(record.mediumResult) > 0;
        const bool strongCrossedGamma =
            result_gamma_crosses(record.strongResult) > 0;
        const bool weakUpdatedGlobal =
            record.costBeforeWeak >= referenceUpperCost
            && record.costAfterWeak < referenceUpperCost;
        const bool mediumUpdatedGlobal =
            record.selectedForMedium
            && record.costAfterWeak >= referenceUpperCost
            && record.costAfterMedium < referenceUpperCost;
        const bool strongUpdatedGlobal =
            record.selectedForStrong
            && record.costAfterMedium >= referenceUpperCost
            && record.costAfterStrong < referenceUpperCost;

        if (weakUpdatedGlobal) {
            ++run.stats[
                intensity_index(LocalSearchIntensity::Weak)]
                .globalUpperUpdates;
        }
        if (mediumUpdatedGlobal) {
            ++run.stats[
                intensity_index(LocalSearchIntensity::Medium)]
                .globalUpperUpdates;
        }
        if (strongUpdatedGlobal) {
            ++run.stats[
                intensity_index(LocalSearchIntensity::Strong)]
                .globalUpperUpdates;
        }

        const bool weakIsTerminal =
            record.terminalIntensity == LocalSearchIntensity::Weak;
        const bool mediumIsTerminal =
            record.terminalIntensity == LocalSearchIntensity::Medium;
        const bool strongIsTerminal =
            record.terminalIntensity == LocalSearchIntensity::Strong;

        const double weakReward = useful_stage_reward(
            record.costBeforeWeak,
            record.costAfterWeak,
            referenceUpperCost,
            weakCrossedGamma,
            weakIsTerminal && record.parentPoolHit,
            weakUpdatedGlobal,
            weakIsTerminal && record.verifiedUpdate,
            weakIsTerminal
                ? record.relativeVerifiedImprovement
                : 0.0);
        run.stats[
            intensity_index(LocalSearchIntensity::Weak)]
            .reward += weakReward;

        if (record.selectedForMedium) {
            const double mediumReward = useful_stage_reward(
                record.costAfterWeak,
                record.costAfterMedium,
                referenceUpperCost,
                mediumCrossedGamma,
                mediumIsTerminal && record.parentPoolHit,
                mediumUpdatedGlobal,
                mediumIsTerminal && record.verifiedUpdate,
                mediumIsTerminal
                    ? record.relativeVerifiedImprovement
                    : 0.0);
            run.stats[
                intensity_index(LocalSearchIntensity::Medium)]
                .reward += mediumReward;
            if (policy == LocalSearchPolicy::ContextualMixed) {
                allocator.update_medium(
                    record.mediumContext,
                    mediumReward);
            }
        }

        if (record.selectedForStrong) {
            const double strongReward = useful_stage_reward(
                record.costAfterMedium,
                record.costAfterStrong,
                referenceUpperCost,
                strongCrossedGamma,
                strongIsTerminal && record.parentPoolHit,
                strongUpdatedGlobal,
                strongIsTerminal && record.verifiedUpdate,
                strongIsTerminal
                    ? record.relativeVerifiedImprovement
                    : 0.0);
            run.stats[
                intensity_index(LocalSearchIntensity::Strong)]
                .reward += strongReward;
            if (policy == LocalSearchPolicy::ContextualMixed) {
                allocator.update_strong(
                    record.strongContext,
                    strongReward);
            }
        }
    }
}

std::size_t LocalSearchAllocationRunner::intensity_index(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Weak:
            return 0;
        case LocalSearchIntensity::Medium:
            return 1;
        case LocalSearchIntensity::Strong:
            return 2;
        case LocalSearchIntensity::Skip:
            break;
    }
    throw std::logic_error(
        "skip is not a mixed local-search stage");
}

const char* LocalSearchAllocationRunner::intensity_name(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Skip:
            return "skip";
        case LocalSearchIntensity::Weak:
            return "weak";
        case LocalSearchIntensity::Medium:
            return "medium";
        case LocalSearchIntensity::Strong:
            return "strong";
    }
    return "unknown";
}

const char* local_search_policy_name(LocalSearchPolicy policy) {
    switch (policy) {
        case LocalSearchPolicy::Static:
            return "static";
        case LocalSearchPolicy::RandomMixed:
            return "random";
        case LocalSearchPolicy::ContextualMixed:
            return "contextual";
    }
    return "unknown";
}
