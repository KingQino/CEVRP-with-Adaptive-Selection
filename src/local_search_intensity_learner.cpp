#include "local_search_intensity_learner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kRidge = 1.0;
constexpr double kExplorationScale = 0.05;
constexpr double kGammaCrossBenefit = 0.05;
constexpr double kMaximumLowerBenefit = 0.05;
constexpr double kBudgetPriceUpdateRate = 0.20;
constexpr double kTieTolerance = 1e-12;

double bounded_quality_gap(double qualityGap) {
    const double nonnegativeGap = std::max(0.0, qualityGap);
    return nonnegativeGap / (1.0 + nonnegativeGap);
}

}  // namespace

LocalSearchIntensityLearner::LocalSearchIntensityLearner() {
    reset();
}

void LocalSearchIntensityLearner::reset() {
    models = {};
    for (auto& model : models) {
        for (std::size_t feature = 0;
             feature < FEATURE_COUNT;
             ++feature) {
            model.gram[feature][feature] = kRidge;
        }
    }
    budgetPrice = 0.0;
    updatedBatches = 0;
}

LocalSearchIntensityDecision LocalSearchIntensityLearner::select(
    const LocalSearchLearningContext& context,
    std::mt19937& randomEngine) {
    const FeatureVector contextFeatures = features(context);
    std::vector<std::size_t> untriedActions;
    for (std::size_t actionIndex = 0;
         actionIndex < ACTION_COUNT;
         ++actionIndex) {
        if (models[actionIndex].selections == 0) {
            untriedActions.push_back(actionIndex);
        }
    }

    if (!untriedActions.empty()) {
        std::uniform_int_distribution<std::size_t> selectUntried(
            0,
            untriedActions.size() - 1);
        const std::size_t selectedIndex =
            untriedActions[selectUntried(randomEngine)];
        ++models[selectedIndex].selections;

        LocalSearchIntensityDecision decision;
        decision.intensity = action_at(selectedIndex);
        decision.context = context;
        decision.budgetPrice = budgetPrice;
        decision.exploratory = true;
        return decision;
    }

    std::array<LocalSearchIntensityDecision, ACTION_COUNT> candidates;
    double bestScore = -std::numeric_limits<double>::infinity();
    std::vector<std::size_t> bestActions;
    for (std::size_t actionIndex = 0;
         actionIndex < ACTION_COUNT;
         ++actionIndex) {
        LocalSearchIntensityDecision& decision = candidates[actionIndex];
        decision.intensity = action_at(actionIndex);
        decision.context = context;
        decision.budgetPrice = budgetPrice;

        if (decision.intensity != LocalSearchIntensity::Skip) {
            const ActionModel& model = models[actionIndex];
            decision.predictedBenefit = std::max(
                0.0,
                predict(model, contextFeatures, true));
            decision.predictedCost = std::max(
                0.0,
                predict(model, contextFeatures, false));
            decision.score =
                decision.predictedBenefit
                + kExplorationScale
                    * uncertainty(model, contextFeatures)
                - budgetPrice * decision.predictedCost;
        }

        if (decision.score > bestScore + kTieTolerance) {
            bestScore = decision.score;
            bestActions.assign(1, actionIndex);
        } else if (std::fabs(decision.score - bestScore)
                   <= kTieTolerance) {
            bestActions.push_back(actionIndex);
        }
    }

    std::uniform_int_distribution<std::size_t> selectBest(
        0,
        bestActions.size() - 1);
    const std::size_t selectedIndex =
        bestActions[selectBest(randomEngine)];
    ++models[selectedIndex].selections;
    return candidates[selectedIndex];
}

void LocalSearchIntensityLearner::update_batch(
    const std::vector<LocalSearchLearningOutcome>& outcomes) {
    double batchBenefit = 0.0;
    double batchCost = 0.0;
    for (const auto& outcome : outcomes) {
        const double benefit = observed_benefit(outcome);
        const double cost = observed_cost(outcome);
        batchBenefit += benefit;
        batchCost += cost;

        if (outcome.decision.intensity == LocalSearchIntensity::Skip) {
            continue;
        }
        update_model(
            models[action_index(outcome.decision.intensity)],
            features(outcome.decision.context),
            benefit,
            cost);
    }

    if (batchCost <= 0.0) {
        return;
    }
    const double observedBudgetPrice = batchBenefit / batchCost;
    if (updatedBatches == 0) {
        budgetPrice = observedBudgetPrice;
    } else {
        budgetPrice =
            (1.0 - kBudgetPriceUpdateRate) * budgetPrice
            + kBudgetPriceUpdateRate * observedBudgetPrice;
    }
    ++updatedBatches;
}

double LocalSearchIntensityLearner::budget_price() const {
    return budgetPrice;
}

double LocalSearchIntensityLearner::observed_benefit(
    const LocalSearchLearningOutcome& outcome) {
    return std::clamp(
               outcome.relativeUpperImprovement,
               0.0,
               1.0)
        + (outcome.crossedGamma ? kGammaCrossBenefit : 0.0)
        + std::clamp(
            outcome.relativeVerifiedLowerImprovement,
            0.0,
            kMaximumLowerBenefit);
}

double LocalSearchIntensityLearner::observed_cost(
    const LocalSearchLearningOutcome& outcome) {
    if (outcome.evaluationLimitDistanceCalls == 0) {
        return 0.0;
    }
    return static_cast<double>(outcome.distanceCalls)
        / static_cast<double>(
            outcome.evaluationLimitDistanceCalls);
}

const char* LocalSearchIntensityLearner::intensity_name(
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

std::size_t LocalSearchIntensityLearner::action_index(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Skip:
            return 0;
        case LocalSearchIntensity::Weak:
            return 1;
        case LocalSearchIntensity::Medium:
            return 2;
        case LocalSearchIntensity::Strong:
            return 3;
    }
    throw std::logic_error("unknown local-search intensity");
}

LocalSearchIntensity LocalSearchIntensityLearner::action_at(
    std::size_t actionIndex) {
    static constexpr std::array<
        LocalSearchIntensity,
        ACTION_COUNT> actions = {
            LocalSearchIntensity::Skip,
            LocalSearchIntensity::Weak,
            LocalSearchIntensity::Medium,
            LocalSearchIntensity::Strong,
        };
    if (actionIndex >= actions.size()) {
        throw std::out_of_range("local-search action index");
    }
    return actions[actionIndex];
}

LocalSearchIntensityLearner::FeatureVector
LocalSearchIntensityLearner::features(
    const LocalSearchLearningContext& context) {
    return {
        1.0,
        bounded_quality_gap(context.qualityGap),
        std::clamp(context.adjacencyDistance, 0.0, 1.0),
        std::clamp(context.budgetProgress, 0.0, 1.0),
        std::tanh(context.gammaMargin),
    };
}

LocalSearchIntensityLearner::FeatureVector
LocalSearchIntensityLearner::solve(
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

double LocalSearchIntensityLearner::dot(
    const FeatureVector& first,
    const FeatureVector& second) {
    double result = 0.0;
    for (std::size_t feature = 0;
         feature < FEATURE_COUNT;
         ++feature) {
        result += first[feature] * second[feature];
    }
    return result;
}

double LocalSearchIntensityLearner::predict(
    const ActionModel& model,
    const FeatureVector& contextFeatures,
    bool predictBenefit) {
    const FeatureVector coefficients = solve(
        model.gram,
        predictBenefit
            ? model.benefitTargets
            : model.costTargets);
    return dot(coefficients, contextFeatures);
}

double LocalSearchIntensityLearner::uncertainty(
    const ActionModel& model,
    const FeatureVector& contextFeatures) {
    const FeatureVector projectedContext = solve(
        model.gram,
        contextFeatures);
    return std::sqrt(std::max(
        0.0,
        dot(contextFeatures, projectedContext)));
}

void LocalSearchIntensityLearner::update_model(
    ActionModel& model,
    const FeatureVector& contextFeatures,
    double benefit,
    double cost) {
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            model.gram[row][column] +=
                contextFeatures[row] * contextFeatures[column];
        }
        model.benefitTargets[row] +=
            contextFeatures[row] * benefit;
        model.costTargets[row] +=
            contextFeatures[row] * cost;
    }
}
