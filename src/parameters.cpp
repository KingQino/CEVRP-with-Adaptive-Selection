#include "parameters.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

void require_probability(double value, const std::string& name) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
        throw std::invalid_argument(name + " must be in [0, 1]");
    }
}

}  // namespace

void Parameters::validate() const {
    if (instance.empty()) {
        throw std::invalid_argument("instance must not be empty");
    }
    if (stopCriteria != 1 && stopCriteria != 2) {
        throw std::invalid_argument("stp must be 1 (max evals) or 2 (max time)");
    }
    if (seed < 0) {
        throw std::invalid_argument("seed must be non-negative");
    }
    if (seed > std::numeric_limits<int>::max() - 10) {
        throw std::invalid_argument("seed is too large for ten-trial mode");
    }
    if (popSize < 2) {
        throw std::invalid_argument("pop_size must be at least 2");
    }
    if (localSearchPolicy != LocalSearchPolicy::Static
        && localSearchIntensity != LocalSearchIntensity::Strong
        && localSearchIntensity
            != LocalSearchIntensity::BoundedStrong) {
        throw std::invalid_argument(
            "allocated ls_policy requires -ls strong or bounded_strong");
    }
    if (operatorSelectionPolicy
            == OperatorSelectionPolicy::BudgetAware
        && ((localSearchPolicy
                != LocalSearchPolicy::MatchedRandom
             && localSearchPolicy
                != LocalSearchPolicy::OnlineIndividual)
            || localSearchIntensity
                != LocalSearchIntensity::BoundedStrong)) {
        throw std::invalid_argument(
            "budget_aware op_policy requires -ls bounded_strong "
            "and -ls_policy matched_random or online");
    }

    require_probability(mutationProb, "mutation_prob");
    require_probability(mutationIndProb, "mutation_ind_prob");
    require_probability(parentPoolRatio, "parent_pool_ratio");
    require_probability(qualityRatio, "quality_ratio");
    require_probability(verifiedUpperRatio, "verified_upper_ratio");
    require_probability(pureImmigrantRatio, "pure_immigrant_ratio");

    if (parentPoolRatio <= 0.0) {
        throw std::invalid_argument("parent_pool_ratio must be greater than 0");
    }
    if (qualityRatio <= 0.0 || qualityRatio >= 1.0) {
        throw std::invalid_argument(
            "quality_ratio must be strictly between 0 and 1");
    }
    if (parentPoolRatio * static_cast<double>(popSize) < 5.0 - 1e-12) {
        throw std::invalid_argument(
            "parent_pool_ratio * pop_size must be at least 5");
    }
    if (verifiedUpperRatio + pureImmigrantRatio > 0.25 + 1e-12) {
        throw std::invalid_argument(
            "verified_upper_ratio + pure_immigrant_ratio must not exceed 0.25");
    }
    if (tournamentSize < 1) {
        throw std::invalid_argument("tournament_size must be at least 1");
    }
    const int parentPoolSize = static_cast<int>(std::ceil(
        parentPoolRatio * static_cast<double>(popSize)));
    if (tournamentSize > parentPoolSize) {
        throw std::invalid_argument(
            "tournament_size must not exceed the configured parent-pool size");
    }
    if (!std::isfinite(gamma) || gamma < 1.0) {
        throw std::invalid_argument("gamma must be finite and at least 1.0");
    }
}
