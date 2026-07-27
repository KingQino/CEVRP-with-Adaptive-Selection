#include "operator_learning.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include "local_search_allocation.hpp"

namespace {

constexpr double kDiscountFactor = 0.95;
constexpr double kExplorationScale = 0.08;
constexpr double kLogCostPenalty = 0.02;
constexpr double kSoftmaxTemperature = 0.05;

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
        arms[index].effectiveCalls *= kDiscountFactor;
        arms[index].creditedReward *= kDiscountFactor;
        arms[index].logCost *= kDiscountFactor;
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
            const double callCount =
                static_cast<double>(operatorStats.calls);
            const double costPerCall =
                normalizedCostUnits / callCount;

            add_operator_stats(
                generationStats[index].operatorStats,
                operatorStats);
            generationStats[index].creditedReward +=
                creditedReward;
            generationStats[index].normalizedCostUnits +=
                normalizedCostUnits;

            arms[index].effectiveCalls += callCount;
            arms[index].creditedReward += creditedReward;
            arms[index].logCost +=
                callCount * std::log1p(costPerCall);
        }
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

void OnlineOperatorLearner::recompute_selection_weights() {
    const double totalEffectiveCalls = std::accumulate(
        arms.begin(),
        arms.end(),
        0.0,
        [](double total, const ArmState& arm) {
            return total + arm.effectiveCalls;
        });

    for (std::size_t index = 0;
         index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++index) {
        const double denominator =
            std::max(1.0, arms[index].effectiveCalls);
        const double meanReward =
            arms[index].creditedReward / denominator;
        const double meanLogCost =
            arms[index].logCost / denominator;
        const double uncertainty = std::sqrt(
            std::log(totalEffectiveCalls + 2.0)
            / (arms[index].effectiveCalls + 1.0));
        scores[index] =
            meanReward
            - kLogCostPenalty * meanLogCost
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
