#include <array>
#include <cassert>
#include <cmath>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "local_search_intensity_learner.hpp"

namespace {

LocalSearchIntensityDecision decision_for(
    LocalSearchIntensity intensity,
    const LocalSearchLearningContext& context) {
    LocalSearchIntensityDecision decision;
    decision.intensity = intensity;
    decision.context = context;
    return decision;
}

LocalSearchLearningOutcome outcome_for(
    LocalSearchIntensity intensity,
    const LocalSearchLearningContext& context,
    double relativeImprovement,
    std::uint64_t distanceCalls) {
    LocalSearchLearningOutcome outcome;
    outcome.decision = decision_for(intensity, context);
    outcome.relativeUpperImprovement = relativeImprovement;
    outcome.distanceCalls = distanceCalls;
    outcome.evaluationLimitDistanceCalls = 1'000'000;
    return outcome;
}

}  // namespace

int main() {
    const LocalSearchLearningContext context{
        0.25,
        0.70,
        0.40,
        0.10,
    };

    LocalSearchIntensityLearner firstLearner;
    LocalSearchIntensityLearner secondLearner;
    std::mt19937 firstEngine(17);
    std::mt19937 secondEngine(17);
    std::set<std::string> exploredActions;
    for (int selection = 0; selection < 4; ++selection) {
        const LocalSearchIntensityDecision firstDecision =
            firstLearner.select(context, firstEngine);
        const LocalSearchIntensityDecision secondDecision =
            secondLearner.select(context, secondEngine);
        assert(firstDecision.intensity == secondDecision.intensity);
        assert(firstDecision.exploratory);
        exploredActions.insert(
            LocalSearchIntensityLearner::intensity_name(
                firstDecision.intensity));
    }
    assert(exploredActions.size() == 4);

    LocalSearchLearningOutcome gammaOutcome =
        outcome_for(
            LocalSearchIntensity::Weak,
            context,
            0.10,
            1'000);
    gammaOutcome.crossedGamma = true;
    gammaOutcome.relativeVerifiedLowerImprovement = 0.02;
    assert(std::fabs(
        LocalSearchIntensityLearner::observed_benefit(gammaOutcome)
        - 0.17) <= 1e-12);
    assert(std::fabs(
        LocalSearchIntensityLearner::observed_cost(gammaOutcome)
        - 0.001) <= 1e-12);

    LocalSearchIntensityLearner efficiencyLearner;
    std::vector<LocalSearchLearningOutcome> outcomes;
    outcomes.reserve(300);
    for (int repetition = 0; repetition < 100; ++repetition) {
        outcomes.push_back(outcome_for(
            LocalSearchIntensity::Weak,
            context,
            0.20,
            100));
        outcomes.push_back(outcome_for(
            LocalSearchIntensity::Medium,
            context,
            0.20,
            1'000));
        outcomes.push_back(outcome_for(
            LocalSearchIntensity::Strong,
            context,
            0.20,
            10'000));
    }
    efficiencyLearner.update_batch(outcomes);

    std::mt19937 efficiencyEngine(31);
    for (int selection = 0; selection < 4; ++selection) {
        efficiencyLearner.select(context, efficiencyEngine);
    }
    const LocalSearchIntensityDecision efficientDecision =
        efficiencyLearner.select(context, efficiencyEngine);
    assert(efficientDecision.intensity == LocalSearchIntensity::Weak);
    assert(efficientDecision.predictedBenefit > 0.0);
    assert(efficientDecision.predictedCost > 0.0);
    assert(efficientDecision.budgetPrice > 0.0);

    efficiencyLearner.reset();
    assert(std::fabs(efficiencyLearner.budget_price()) <= 1e-12);

    return 0;
}
