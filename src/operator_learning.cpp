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

std::size_t context_index(OperatorLearningContext context) {
    const std::size_t index = static_cast<std::size_t>(context);
    if (index >= OPERATOR_LEARNING_CONTEXT_COUNT) {
        throw std::logic_error("invalid operator-learning context");
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
        case OperatorSelectionPolicy::Online:
            return "online";
    }
    return "unknown";
}

const char* operator_learning_context_name(
    OperatorLearningContext context) {
    switch (context) {
        case OperatorLearningContext::MediumContinuation:
            return "medium_continuation";
        case OperatorLearningContext::DeepContinuation:
            return "deep_continuation";
        case OperatorLearningContext::Count:
            break;
    }
    return "unknown";
}

void OnlineOperatorLearner::reset() {
    contexts = {};
    for (std::size_t contextIndex = 0;
         contextIndex < OPERATOR_LEARNING_CONTEXT_COUNT;
         ++contextIndex) {
        auto& state = contexts[contextIndex];
        state.selectionWeights.fill(
            1.0 / static_cast<double>(
                LOCAL_SEARCH_OPERATOR_COUNT));
        state.selectionProbabilities =
            state.selectionWeights;
        recompute_selection_weights(
            static_cast<OperatorLearningContext>(
                contextIndex));
    }
}

const OnlineOperatorLearner::SelectionWeights&
OnlineOperatorLearner::selection_weights(
    OperatorLearningContext context) const {
    return contexts[context_index(context)].selectionWeights;
}

OnlineOperatorLearner::GenerationStats OnlineOperatorLearner::update(
    const LocalSearchAllocationRun& run) {
    GenerationStats generationStats{};
    for (std::size_t contextIndex = 0;
         contextIndex < OPERATOR_LEARNING_CONTEXT_COUNT;
         ++contextIndex) {
        auto& state = contexts[contextIndex];
        auto& contextStats = generationStats[contextIndex];
        for (std::size_t operatorIndex = 0;
             operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
             ++operatorIndex) {
            contextStats[operatorIndex].effectiveObservations =
                state.arms[operatorIndex].effectiveObservations;
            contextStats[operatorIndex].score =
                state.scores[operatorIndex];
            contextStats[operatorIndex].selectionProbability =
                state.selectionProbabilities[operatorIndex];
            state.arms[operatorIndex].effectiveObservations *=
                kDiscountFactor;
            state.arms[operatorIndex].rewardRateSum *=
                kDiscountFactor;
            state.arms[operatorIndex].logCostRateSum *=
                kDiscountFactor;
        }
    }

    for (const auto& record : run.records) {
        if (record.terminalIntensity
                != LocalSearchIntensity::Medium
            && record.terminalIntensity
                != LocalSearchIntensity::BoundedStrong
            && record.terminalIntensity
                != LocalSearchIntensity::Strong) {
            continue;
        }
        const OperatorLearningContext context =
            context_for_intensity(record.terminalIntensity);
        const std::size_t contextIndex = context_index(context);
        auto& contextStats = generationStats[contextIndex];

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
            const double normalizedCostUnits =
                static_cast<double>(operatorStats.distanceCalls)
                / weakDistanceCalls;

            add_operator_stats(
                contextStats[index].operatorStats,
                operatorStats);
            contextStats[index].creditedReward +=
                creditedReward;
            contextStats[index].normalizedCostUnits +=
                normalizedCostUnits;
        }
    }

    for (std::size_t contextIndex = 0;
         contextIndex < OPERATOR_LEARNING_CONTEXT_COUNT;
         ++contextIndex) {
        auto& state = contexts[contextIndex];
        const auto& contextStats =
            generationStats[contextIndex];
        for (std::size_t operatorIndex = 0;
             operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
             ++operatorIndex) {
            const double callCount = static_cast<double>(
                contextStats[operatorIndex]
                    .operatorStats.calls);
            if (callCount <= 0.0) {
                continue;
            }
            const double rewardPerCall =
                contextStats[operatorIndex].creditedReward
                / callCount;
            const double costPerCall =
                contextStats[operatorIndex]
                    .normalizedCostUnits
                / callCount;
            state.arms[operatorIndex].effectiveObservations +=
                1.0;
            state.arms[operatorIndex].rewardRateSum +=
                rewardPerCall;
            state.arms[operatorIndex].logCostRateSum +=
                std::log1p(costPerCall);
        }
        recompute_selection_weights(
            static_cast<OperatorLearningContext>(
                contextIndex));
    }

    return generationStats;
}

double OnlineOperatorLearner::score(
    OperatorLearningContext context,
    LocalSearchOperator localSearchOperator) const {
    return contexts[context_index(context)]
        .scores[operator_index(localSearchOperator)];
}

double OnlineOperatorLearner::selection_probability(
    OperatorLearningContext context,
    LocalSearchOperator localSearchOperator) const {
    return contexts[context_index(context)]
        .selectionProbabilities[
            operator_index(localSearchOperator)];
}

double OnlineOperatorLearner::effective_observations(
    OperatorLearningContext context,
    LocalSearchOperator localSearchOperator) const {
    return contexts[context_index(context)]
        .arms[operator_index(localSearchOperator)]
        .effectiveObservations;
}

OperatorLearningContext
OnlineOperatorLearner::context_for_intensity(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Medium:
            return OperatorLearningContext::MediumContinuation;
        case LocalSearchIntensity::BoundedStrong:
        case LocalSearchIntensity::Strong:
            return OperatorLearningContext::DeepContinuation;
        case LocalSearchIntensity::Skip:
        case LocalSearchIntensity::Weak:
            break;
    }
    throw std::logic_error(
        "operator learning requires a continuation intensity");
}

void OnlineOperatorLearner::recompute_selection_weights(
    OperatorLearningContext context) {
    auto& state = contexts[context_index(context)];
    const double totalEffectiveObservations = std::accumulate(
        state.arms.begin(),
        state.arms.end(),
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
            state.arms[index].effectiveObservations;
        if (effectiveObservations > kMinimumScale) {
            rewardSignals[index] =
                state.arms[index].rewardRateSum
                / effectiveObservations;
            costSignals[index] =
                state.arms[index].logCostRateSum
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
            / (state.arms[index].effectiveObservations
                + 1.0));
        state.scores[index] =
            standardizedReward[index]
            - kNormalizedCostWeight
                * standardizedCost[index]
            + kExplorationScale * uncertainty;
    }

    const double maximumScore = *std::max_element(
        state.scores.begin(),
        state.scores.end());
    SelectionWeights softmaxWeights{};
    double softmaxTotal = 0.0;
    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        softmaxWeights[index] = std::exp(
            (state.scores[index] - maximumScore)
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
        state.selectionWeights[index] =
            exploitationProbability;
        state.selectionProbabilities[index] =
            UNIFORM_EXPLORATION_RATE * uniformProbability
            + (1.0 - UNIFORM_EXPLORATION_RATE)
                * exploitationProbability;
    }
}
