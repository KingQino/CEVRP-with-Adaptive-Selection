#ifndef LOCAL_SEARCH_INTENSITY_LEARNER_HPP
#define LOCAL_SEARCH_INTENSITY_LEARNER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "leader.hpp"

struct LocalSearchLearningContext {
    double qualityGap{};
    double adjacencyDistance{};
    double budgetProgress{};
    double gammaMargin{};
};

struct LocalSearchIntensityDecision {
    LocalSearchIntensity intensity{LocalSearchIntensity::Skip};
    LocalSearchLearningContext context;
    double predictedBenefit{};
    double predictedCost{};
    double budgetPrice{};
    double score{};
    bool exploratory{};
};

struct LocalSearchLearningOutcome {
    LocalSearchIntensityDecision decision;
    double relativeUpperImprovement{};
    std::uint64_t distanceCalls{};
    std::uint64_t evaluationLimitDistanceCalls{};
    bool crossedGamma{};
    double relativeVerifiedLowerImprovement{};
};

class LocalSearchIntensityLearner {
public:
    LocalSearchIntensityLearner();

    void reset();
    LocalSearchIntensityDecision select(
        const LocalSearchLearningContext& context,
        std::mt19937& randomEngine);
    void update_batch(
        const std::vector<LocalSearchLearningOutcome>& outcomes);

    [[nodiscard]] double budget_price() const;
    [[nodiscard]] static double observed_benefit(
        const LocalSearchLearningOutcome& outcome);
    [[nodiscard]] static double observed_cost(
        const LocalSearchLearningOutcome& outcome);
    [[nodiscard]] static const char* intensity_name(
        LocalSearchIntensity intensity);

private:
    static constexpr std::size_t ACTION_COUNT = 4;
    static constexpr std::size_t FEATURE_COUNT = 5;

    using FeatureVector = std::array<double, FEATURE_COUNT>;
    using Matrix =
        std::array<std::array<double, FEATURE_COUNT>, FEATURE_COUNT>;

    struct ActionModel {
        Matrix gram{};
        FeatureVector benefitTargets{};
        FeatureVector costTargets{};
        std::uint64_t selections{};
    };

    std::array<ActionModel, ACTION_COUNT> models;
    double budgetPrice{};
    std::uint64_t updatedBatches{};

    static std::size_t action_index(LocalSearchIntensity intensity);
    static LocalSearchIntensity action_at(std::size_t actionIndex);
    static FeatureVector features(
        const LocalSearchLearningContext& context);
    static FeatureVector solve(
        const Matrix& matrix,
        const FeatureVector& target);
    static double dot(
        const FeatureVector& first,
        const FeatureVector& second);
    static double predict(
        const ActionModel& model,
        const FeatureVector& contextFeatures,
        bool predictBenefit);
    static double uncertainty(
        const ActionModel& model,
        const FeatureVector& contextFeatures);
    static void update_model(
        ActionModel& model,
        const FeatureVector& contextFeatures,
        double benefit,
        double cost);
};

#endif
