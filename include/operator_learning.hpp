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

enum class OperatorLearningContext {
    MediumContinuation,
    DeepContinuation,
    Count,
};

constexpr std::size_t OPERATOR_LEARNING_CONTEXT_COUNT =
    static_cast<std::size_t>(OperatorLearningContext::Count);

const char* operator_selection_policy_name(
    OperatorSelectionPolicy policy);
const char* operator_learning_context_name(
    OperatorLearningContext context);

struct OperatorLearningStats {
    LocalSearchOperatorStats operatorStats;
    double creditedReward{};
    double normalizedCostUnits{};
    double effectiveObservations{};
    double score{};
    double selectionProbability{};
};

class OnlineOperatorLearner {
public:
    static constexpr double UNIFORM_EXPLORATION_RATE = 0.05;
    using SelectionWeights =
        std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>;
    using ContextStats =
        std::array<OperatorLearningStats, LOCAL_SEARCH_OPERATOR_COUNT>;
    using GenerationStats =
        std::array<ContextStats, OPERATOR_LEARNING_CONTEXT_COUNT>;
    using SelectionTables =
        std::array<
            LocalSearchOperatorSelectionTable,
            OPERATOR_LEARNING_CONTEXT_COUNT>;

    void reset();
    [[nodiscard]] const SelectionWeights& selection_weights(
        OperatorLearningContext context) const;
    [[nodiscard]] GenerationStats update(
        const LocalSearchAllocationRun& run);
    [[nodiscard]] double score(
        OperatorLearningContext context,
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double selection_probability(
        OperatorLearningContext context,
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double effective_observations(
        OperatorLearningContext context,
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] static OperatorLearningContext context_for_intensity(
        LocalSearchIntensity intensity);

private:
    struct ArmState {
        double effectiveObservations{};
        double rewardRateSum{};
        double logCostRateSum{};
    };

    struct ContextState {
        std::array<ArmState, LOCAL_SEARCH_OPERATOR_COUNT> arms{};
        SelectionWeights scores{};
        SelectionWeights selectionWeights{};
        SelectionWeights selectionProbabilities{};
    };

    std::array<ContextState, OPERATOR_LEARNING_CONTEXT_COUNT> contexts{};

    void recompute_selection_weights(
        OperatorLearningContext context);
};

#endif
