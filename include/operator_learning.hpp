#ifndef OPERATOR_LEARNING_HPP
#define OPERATOR_LEARNING_HPP

#include <array>
#include <cstddef>

#include "leader.hpp"

struct LocalSearchAllocationRun;

enum class OperatorSelectionPolicy {
    Uniform,
    Online,
};

const char* operator_selection_policy_name(
    OperatorSelectionPolicy policy);

struct OperatorLearningStats {
    LocalSearchOperatorStats operatorStats;
    double creditedReward{};
    double normalizedCostUnits{};
    double score{};
    double selectionProbability{};
};

class OnlineOperatorLearner {
public:
    static constexpr double UNIFORM_EXPLORATION_RATE = 0.05;
    using SelectionWeights =
        std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>;
    using GenerationStats =
        std::array<OperatorLearningStats, LOCAL_SEARCH_OPERATOR_COUNT>;

    void reset();
    [[nodiscard]] const SelectionWeights& selection_weights() const;
    [[nodiscard]] GenerationStats update(
        const LocalSearchAllocationRun& run);
    [[nodiscard]] double score(LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double selection_probability(
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double effective_observations(
        LocalSearchOperator localSearchOperator) const;

private:
    struct ArmState {
        double effectiveObservations{};
        double rewardRateSum{};
        double logCostRateSum{};
    };

    std::array<ArmState, LOCAL_SEARCH_OPERATOR_COUNT> arms{};
    SelectionWeights scores{};
    SelectionWeights selectionWeights{};
    SelectionWeights selectionProbabilities{};

    void recompute_selection_weights();
};

#endif
