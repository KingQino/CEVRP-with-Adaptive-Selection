#include "operator_learning.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include "local_search_allocation.hpp"

namespace {

constexpr double kDiscountFactor = 0.95;
constexpr double kExplorationScale = 0.20;
constexpr double kSoftmaxTemperature = 1.0;
constexpr double kRobustScaleFactor = 1.4826;
constexpr double kMaximumStandardizedSignal = 3.0;
constexpr double kMinimumScale = 1e-12;
constexpr double kMinimumEstimatedCostPerCall = 1.0;

std::size_t operator_index(LocalSearchOperator localSearchOperator) {
    const std::size_t index =
        static_cast<std::size_t>(localSearchOperator);
    if (index >= LOCAL_SEARCH_OPERATOR_COUNT) {
        throw std::logic_error("invalid local-search operator");
    }
    return index;
}

void add_operator_stats(
    LocalSearchOperatorStats& destination,
    const LocalSearchOperatorStats& source) {
    destination.calls += source.calls;
    destination.accepts += source.accepts;
    destination.distanceCalls += source.distanceCalls;
    destination.upperGain += source.upperGain;
    destination.gammaCrosses += source.gammaCrosses;
}

double median(
    std::array<double, LOCAL_SEARCH_OPERATOR_COUNT> values) {
    std::sort(values.begin(), values.end());
    constexpr std::size_t upperMiddle =
        LOCAL_SEARCH_OPERATOR_COUNT / 2;
    return 0.5
        * (values[upperMiddle - 1] + values[upperMiddle]);
}

std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>
robust_standardize(
    const std::array<
        double,
        LOCAL_SEARCH_OPERATOR_COUNT>& values) {
    const double center = median(values);
    std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>
        absoluteDeviations{};
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        absoluteDeviations[index] =
            std::fabs(values[index] - center);
    }

    double scale =
        kRobustScaleFactor * median(absoluteDeviations);
    if (scale < kMinimumScale) {
        double squaredDeviationSum = 0.0;
        for (const double value : values) {
            const double deviation = value - center;
            squaredDeviationSum += deviation * deviation;
        }
        scale = std::sqrt(
            squaredDeviationSum
            / static_cast<double>(
                LOCAL_SEARCH_OPERATOR_COUNT));
    }

    std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>
        standardized{};
    if (scale < kMinimumScale) {
        return standardized;
    }
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        standardized[index] = std::clamp(
            (values[index] - center) / scale,
            -kMaximumStandardizedSignal,
            kMaximumStandardizedSignal);
    }
    return standardized;
}

}  // namespace

const char* operator_selection_policy_name(
    OperatorSelectionPolicy policy) {
    switch (policy) {
        case OperatorSelectionPolicy::Uniform:
            return "uniform";
        case OperatorSelectionPolicy::BudgetAware:
            return "budget_aware";
    }
    return "unknown";
}

void BudgetAwareOperatorScheduler::reset() {
    arms = {};
    scores = {};
    estimatedCostsPerCall.fill(kMinimumEstimatedCostPerCall);
    efficiencies = {};
    targetBudgetShares.fill(
        1.0 / static_cast<double>(LOCAL_SEARCH_OPERATOR_COUNT));
    callSelectionWeights = targetBudgetShares;
    selectionProbabilities = targetBudgetShares;
    recompute_scheduler();
}

const BudgetAwareOperatorScheduler::SelectionWeights&
BudgetAwareOperatorScheduler::call_selection_weights() const {
    return callSelectionWeights;
}

BudgetAwareOperatorScheduler::GenerationStats
BudgetAwareOperatorScheduler::update(
    const LocalSearchAllocationRun& run) {
    GenerationStats generationStats{};
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        generationStats[index].estimatedCostPerCall =
            estimatedCostsPerCall[index];
        generationStats[index].efficiency =
            efficiencies[index];
        generationStats[index].targetBudgetShare =
            targetBudgetShares[index];
        generationStats[index].selectionProbability =
            selectionProbabilities[index];
        arms[index].effectiveObservations *= kDiscountFactor;
        arms[index].calls *= kDiscountFactor;
        arms[index].distanceCalls *= kDiscountFactor;
        arms[index].relativeUpperGain *= kDiscountFactor;
    }

    for (const auto& record : run.records) {
        // Downstream parent/lower rewards belong to intensity allocation.
        // The operator layer learns only its immediate continuation gain.
        const double upperCostScale = std::max(
            std::fabs(record.costAfterWeak),
            kMinimumScale);

        for (std::size_t index = 0;
             index < LOCAL_SEARCH_OPERATOR_COUNT;
             ++index) {
            const auto& operatorStats =
                record.continuationResult.operatorStats[index];
            if (operatorStats.calls == 0) {
                continue;
            }

            add_operator_stats(
                generationStats[index].operatorStats,
                operatorStats);
            generationStats[index].relativeUpperGain +=
                operatorStats.upperGain / upperCostScale;
        }
    }

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double callCount = static_cast<double>(
            generationStats[index].operatorStats.calls);
        if (callCount <= 0.0) {
            continue;
        }

        arms[index].effectiveObservations += 1.0;
        arms[index].calls += callCount;
        arms[index].distanceCalls += static_cast<double>(
            generationStats[index].operatorStats.distanceCalls);
        arms[index].relativeUpperGain +=
            generationStats[index].relativeUpperGain;
    }

    recompute_scheduler();
    return generationStats;
}

double BudgetAwareOperatorScheduler::estimated_cost_per_call(
    LocalSearchOperator localSearchOperator) const {
    return estimatedCostsPerCall[operator_index(localSearchOperator)];
}

double BudgetAwareOperatorScheduler::efficiency(
    LocalSearchOperator localSearchOperator) const {
    return efficiencies[operator_index(localSearchOperator)];
}

double BudgetAwareOperatorScheduler::target_budget_share(
    LocalSearchOperator localSearchOperator) const {
    return targetBudgetShares[operator_index(localSearchOperator)];
}

double BudgetAwareOperatorScheduler::selection_probability(
    LocalSearchOperator localSearchOperator) const {
    return selectionProbabilities[
        operator_index(localSearchOperator)];
}

double BudgetAwareOperatorScheduler::effective_observations(
    LocalSearchOperator localSearchOperator) const {
    return arms[operator_index(localSearchOperator)]
        .effectiveObservations;
}

void BudgetAwareOperatorScheduler::recompute_scheduler() {
    const double totalEffectiveObservations = std::accumulate(
        arms.begin(),
        arms.end(),
        0.0,
        [](double total, const ArmState& arm) {
            return total + arm.effectiveObservations;
        });

    SelectionWeights observedCostsPerCall{};
    std::array<bool, LOCAL_SEARCH_OPERATOR_COUNT> hasCostObservation{};
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        if (arms[index].calls > kMinimumScale) {
            observedCostsPerCall[index] =
                arms[index].distanceCalls / arms[index].calls;
            hasCostObservation[index] = true;
        }
    }

    SelectionWeights observedCosts{};
    std::size_t observedCostCount = 0;
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        if (hasCostObservation[index]) {
            observedCosts[observedCostCount++] =
                observedCostsPerCall[index];
        }
    }
    double fallbackCostPerCall = kMinimumEstimatedCostPerCall;
    if (observedCostCount > 0) {
        std::sort(
            observedCosts.begin(),
            observedCosts.begin() + observedCostCount);
        const std::size_t middle = observedCostCount / 2;
        fallbackCostPerCall = observedCostCount % 2 == 0
            ? 0.5
                * (observedCosts[middle - 1]
                   + observedCosts[middle])
            : observedCosts[middle];
        fallbackCostPerCall = std::max(
            fallbackCostPerCall,
            kMinimumEstimatedCostPerCall);
    }

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        estimatedCostsPerCall[index] = std::max(
            hasCostObservation[index]
                ? observedCostsPerCall[index]
                : fallbackCostPerCall,
            kMinimumEstimatedCostPerCall);
        efficiencies[index] =
            arms[index].distanceCalls > kMinimumScale
            ? arms[index].relativeUpperGain
                / arms[index].distanceCalls
            : 0.0;
    }
    const SelectionWeights standardizedEfficiency =
        robust_standardize(efficiencies);

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double uncertainty = std::sqrt(
            std::log(totalEffectiveObservations + 2.0)
            / (arms[index].effectiveObservations + 1.0));
        scores[index] =
            standardizedEfficiency[index]
            + kExplorationScale * uncertainty;
    }

    const double maximumScore = *std::max_element(
        scores.begin(),
        scores.end());
    SelectionWeights softmaxWeights{};
    double softmaxTotal = 0.0;
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        softmaxWeights[index] = std::exp(
            (scores[index] - maximumScore)
            / kSoftmaxTemperature);
        softmaxTotal += softmaxWeights[index];
    }

    const double uniformProbability =
        1.0 / static_cast<double>(LOCAL_SEARCH_OPERATOR_COUNT);
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double learnedBudgetShare =
            softmaxTotal > 0.0
            ? softmaxWeights[index] / softmaxTotal
            : uniformProbability;
        targetBudgetShares[index] =
            BUDGET_EXPLORATION_RATE * uniformProbability
            + (1.0 - BUDGET_EXPLORATION_RATE)
                * learnedBudgetShare;
        // Convert a desired cost share into a call weight.
        callSelectionWeights[index] =
            targetBudgetShares[index]
            / estimatedCostsPerCall[index];
    }

    const double callWeightTotal = std::accumulate(
        callSelectionWeights.begin(),
        callSelectionWeights.end(),
        0.0);
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        selectionProbabilities[index] =
            callWeightTotal > kMinimumScale
            ? callSelectionWeights[index] / callWeightTotal
            : uniformProbability;
    }
}
