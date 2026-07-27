#include "operator_learning.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include "local_search_allocation.hpp"

namespace {

constexpr double kDiscountFactor = 0.95;
constexpr double kExplorationScale = 0.08;
constexpr double kNormalizedCostWeight = 1.0;
constexpr double kDistanceCostShare = 0.5;
constexpr double kWorkCostShare = 0.5;
constexpr double kSoftmaxTemperature = 1.0;
constexpr double kRobustScaleFactor = 1.4826;
constexpr double kMaximumStandardizedSignal = 3.0;
constexpr double kMinimumScale = 1e-12;

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
    destination.workUnits += source.workUnits;
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
        case OperatorSelectionPolicy::Online:
            return "online";
    }
    return "unknown";
}

void OnlineOperatorLearner::reset() {
    arms = {};
    scores = {};
    selectionWeights.fill(
        1.0 / static_cast<double>(LOCAL_SEARCH_OPERATOR_COUNT));
    selectionProbabilities = selectionWeights;
    recompute_selection_weights();
}

const OnlineOperatorLearner::SelectionWeights&
OnlineOperatorLearner::selection_weights() const {
    return selectionWeights;
}

OnlineOperatorLearner::GenerationStats OnlineOperatorLearner::update(
    const LocalSearchAllocationRun& run) {
    GenerationStats generationStats{};
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        generationStats[index].score = scores[index];
        generationStats[index].selectionProbability =
            selectionProbabilities[index];
        arms[index].effectiveObservations *= kDiscountFactor;
        arms[index].rewardRateSum *= kDiscountFactor;
        arms[index].logCostRateSum *= kDiscountFactor;
    }

    for (const auto& record : run.records) {
        double totalUpperGain = 0.0;
        int totalGammaCrosses = 0;
        for (const auto& stats :
             record.continuationResult.operatorStats) {
            totalUpperGain += stats.upperGain;
            totalGammaCrosses += stats.gammaCrosses;
        }

        const double downstreamReward =
            record.parentReward + record.lowerReward;
        const double weakDistanceCalls = static_cast<double>(
            std::max<std::uint64_t>(
                1,
                record.weakResult.distanceCallsUsed));
        std::uint64_t weakWorkUnits = 0;
        for (const auto& operatorStats :
             record.weakResult.operatorStats) {
            weakWorkUnits += operatorStats.workUnits;
        }
        const double weakWorkCost = static_cast<double>(
            std::max<std::uint64_t>(1, weakWorkUnits));

        for (std::size_t index = 0;
             index < LOCAL_SEARCH_OPERATOR_COUNT;
             ++index) {
            const auto& operatorStats =
                record.continuationResult.operatorStats[index];
            if (operatorStats.calls == 0) {
                continue;
            }

            const double gainCredit = totalUpperGain > 0.0
                ? downstreamReward
                    * operatorStats.upperGain
                    / totalUpperGain
                : 0.0;
            // Gamma credit is kept separate so crossing the follower
            // threshold is attributed to the operator that caused it.
            const double gammaCredit = totalGammaCrosses > 0
                ? record.gammaReward
                    * static_cast<double>(
                        operatorStats.gammaCrosses)
                    / static_cast<double>(totalGammaCrosses)
                : 0.0;
            const double creditedReward =
                gainCredit + gammaCredit;
            const double normalizedDistanceCostUnits =
                static_cast<double>(operatorStats.distanceCalls)
                / weakDistanceCalls;
            const double normalizedWorkCostUnits =
                static_cast<double>(operatorStats.workUnits)
                / weakWorkCost;
            // Separate weak-probe normalization keeps either raw counter from
            // dominating solely because it has a larger numerical scale.
            const double normalizedCostUnits =
                kDistanceCostShare
                    * normalizedDistanceCostUnits
                + kWorkCostShare
                    * normalizedWorkCostUnits;

            add_operator_stats(
                generationStats[index].operatorStats,
                operatorStats);
            generationStats[index].creditedReward +=
                creditedReward;
            generationStats[index].normalizedDistanceCostUnits +=
                normalizedDistanceCostUnits;
            generationStats[index].normalizedWorkCostUnits +=
                normalizedWorkCostUnits;
            generationStats[index].normalizedCostUnits +=
                normalizedCostUnits;
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
        const double rewardPerCall =
            generationStats[index].creditedReward / callCount;
        const double costPerCall =
            generationStats[index].normalizedCostUnits
            / callCount;
        arms[index].effectiveObservations += 1.0;
        arms[index].rewardRateSum += rewardPerCall;
        arms[index].logCostRateSum +=
            std::log1p(costPerCall);
    }

    recompute_selection_weights();
    return generationStats;
}

double OnlineOperatorLearner::score(
    LocalSearchOperator localSearchOperator) const {
    return scores[operator_index(localSearchOperator)];
}

double OnlineOperatorLearner::selection_probability(
    LocalSearchOperator localSearchOperator) const {
    return selectionProbabilities[
        operator_index(localSearchOperator)];
}

double OnlineOperatorLearner::effective_observations(
    LocalSearchOperator localSearchOperator) const {
    return arms[operator_index(localSearchOperator)]
        .effectiveObservations;
}

void OnlineOperatorLearner::recompute_selection_weights() {
    const double totalEffectiveObservations = std::accumulate(
        arms.begin(),
        arms.end(),
        0.0,
        [](double total, const ArmState& arm) {
            return total + arm.effectiveObservations;
        });

    SelectionWeights rewardSignals{};
    SelectionWeights costSignals{};
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double effectiveObservations =
            arms[index].effectiveObservations;
        if (effectiveObservations > kMinimumScale) {
            rewardSignals[index] =
                arms[index].rewardRateSum
                / effectiveObservations;
            costSignals[index] =
                arms[index].logCostRateSum
                / effectiveObservations;
        }
    }
    const SelectionWeights standardizedReward =
        robust_standardize(rewardSignals);
    const SelectionWeights standardizedCost =
        robust_standardize(costSignals);

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double uncertainty = std::sqrt(
            std::log(totalEffectiveObservations + 2.0)
            / (arms[index].effectiveObservations + 1.0));
        scores[index] =
            standardizedReward[index]
            - kNormalizedCostWeight
                * standardizedCost[index]
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
        const double exploitationProbability =
            softmaxTotal > 0.0
            ? softmaxWeights[index] / softmaxTotal
            : uniformProbability;
        selectionWeights[index] = exploitationProbability;
        selectionProbabilities[index] =
            UNIFORM_EXPLORATION_RATE * uniformProbability
            + (1.0 - UNIFORM_EXPLORATION_RATE)
                * exploitationProbability;
    }
}
