#ifndef OPERATOR_LEARNING_HPP
#define OPERATOR_LEARNING_HPP

#include <array>
#include <cstddef>

#include "leader.hpp"

struct LocalSearchAllocationRun;

enum class OperatorSelectionPolicy {
    Uniform,
    BudgetAware,
};

const char* operator_selection_policy_name(
    OperatorSelectionPolicy policy);

struct OperatorLearningStats {
    LocalSearchOperatorStats operatorStats;
    double relativeUpperGain{};
    double futureRelativeGainCredit{};
    double sequenceAwareRelativeValue{};
    double estimatedCostPerCall{};
    double efficiency{};
    double targetBudgetShare{};
    double selectionProbability{};
};

class BudgetAwareOperatorScheduler {
public:
    static constexpr double BUDGET_EXPLORATION_RATE = 0.20;
    using SelectionWeights =
        std::array<double, LOCAL_SEARCH_OPERATOR_COUNT>;
    using GenerationStats =
        std::array<OperatorLearningStats, LOCAL_SEARCH_OPERATOR_COUNT>;

    void reset();
    [[nodiscard]] const SelectionWeights&
    call_selection_weights() const;
    [[nodiscard]] GenerationStats update(
        const LocalSearchAllocationRun& run,
        bool budgetAwareSelection = true);
    [[nodiscard]] double estimated_cost_per_call(
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double efficiency(
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double target_budget_share(
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double selection_probability(
        LocalSearchOperator localSearchOperator) const;
    [[nodiscard]] double effective_observations(
        LocalSearchOperator localSearchOperator) const;

private:
    struct ArmState {
        double effectiveObservations{};
        double calls{};
        double distanceCalls{};
        double sequenceAwareRelativeValue{};
    };

    std::array<ArmState, LOCAL_SEARCH_OPERATOR_COUNT> arms{};
    SelectionWeights scores{};
    SelectionWeights estimatedCostsPerCall{};
    SelectionWeights efficiencies{};
    SelectionWeights targetBudgetShares{};
    SelectionWeights callSelectionWeights{};
    SelectionWeights selectionProbabilities{};

    void recompute_scheduler();
};

#endif
