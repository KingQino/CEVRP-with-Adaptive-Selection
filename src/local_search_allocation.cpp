#include "local_search_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include "algorithm_constants.hpp"
#include "case.hpp"
#include "individual.hpp"

namespace {

constexpr double kRidge = 1.0;
constexpr double kExplorationScale = 0.08;
constexpr double kRandomExplorationRate = 0.05;
constexpr double kLowerRewardWeight = 0.45;
constexpr double kGammaRewardWeight = 0.25;
constexpr double kParentRewardWeight = 0.20;
constexpr double kContinuationGainSignalWeight = 0.10;
constexpr double kContinuationGainScale = 100.0;
constexpr std::size_t kWorkspaceBatchSize = 10;
constexpr double kMediumSelectionRatio = 0.80;
constexpr double kStrongSelectionRatio = 0.50;
// Quality-only shared configuration's confirmed aggregate online allocation.
constexpr double kMatchedMediumRatio = 0.0348226126187;
constexpr double kMatchedDeepestRatio = 0.2088046423357;

LocalSearchActionRatios normalized_ratios(
    const LocalSearchActionRatios& ratios,
    const std::string& source) {
    if (!std::isfinite(ratios.weak)
        || !std::isfinite(ratios.medium)
        || !std::isfinite(ratios.deepest)
        || ratios.weak < 0.0
        || ratios.medium < 0.0
        || ratios.deepest < 0.0) {
        throw std::runtime_error(
            "invalid local-search action ratios in " + source);
    }
    const double total = ratios.weak + ratios.medium + ratios.deepest;
    if (std::fabs(total - 1.0) > 1e-6) {
        throw std::runtime_error(
            "local-search action ratios must sum to one in " + source);
    }
    return {
        ratios.weak / total,
        ratios.medium / total,
        ratios.deepest / total,
    };
}

std::vector<LocalSearchIntensity> make_balanced_random_intensities(
    std::size_t count,
    const LocalSearchActionRatios& ratios,
    LocalSearchIntensity deepestIntensity,
    std::array<std::size_t, 3>& assignedCounts,
    std::size_t& eligibleCount,
    std::mt19937& randomEngine) {
    const std::array<double, 3> probabilities = {
        ratios.weak,
        ratios.medium,
        ratios.deepest,
    };
    const std::array<LocalSearchIntensity, 3> intensities = {
        LocalSearchIntensity::Weak,
        LocalSearchIntensity::Medium,
        deepestIntensity,
    };
    std::vector<LocalSearchIntensity> assignments;
    assignments.reserve(count);
    for (std::size_t item = 0; item < count; ++item) {
        ++eligibleCount;
        std::size_t selectedIndex = 0;
        double largestDeficit = -std::numeric_limits<double>::infinity();
        for (std::size_t actionIndex = 0;
             actionIndex < probabilities.size();
             ++actionIndex) {
            const double deficit =
                probabilities[actionIndex]
                    * static_cast<double>(eligibleCount)
                - static_cast<double>(assignedCounts[actionIndex]);
            if (deficit > largestDeficit) {
                largestDeficit = deficit;
                selectedIndex = actionIndex;
            }
        }
        ++assignedCounts[selectedIndex];
        assignments.push_back(intensities[selectedIndex]);
    }
    std::shuffle(
        assignments.begin(),
        assignments.end(),
        randomEngine);
    return assignments;
}

double bounded_nonnegative(double value) {
    const double nonnegative = std::max(0.0, value);
    return nonnegative / (1.0 + nonnegative);
}

double result_upper_gain(const LocalSearchResult& result) {
    double gain = 0.0;
    for (const auto& stats : result.operatorStats) {
        gain += stats.upperGain;
    }
    return gain;
}

void add_operator_stats(
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT>& destination,
    const LocalSearchResult& source) {
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        auto& destinationStats = destination[operatorIndex];
        const auto& sourceStats = source.operatorStats[operatorIndex];
        destinationStats.calls += sourceStats.calls;
        destinationStats.accepts += sourceStats.accepts;
        destinationStats.distanceCalls += sourceStats.distanceCalls;
        destinationStats.upperGain += sourceStats.upperGain;
        destinationStats.gammaCrosses += sourceStats.gammaCrosses;
    }
}

LocalSearchResult combine_results(
    const LocalSearchResult& first,
    const LocalSearchResult& second,
    double costBefore,
    double costAfter) {
    LocalSearchResult combined;
    combined.moveLimit = second.moveLimit;
    combined.acceptedMoves =
        first.acceptedMoves + second.acceptedMoves;
    combined.neighborhoodCalls =
        first.neighborhoodCalls + second.neighborhoodCalls;
    combined.distanceCallsUsed =
        first.distanceCallsUsed + second.distanceCallsUsed;
    combined.relativeUpperImprovement = costBefore > 0.0
        ? (costBefore - costAfter) / costBefore
        : 0.0;
    combined.reachedLocalOptimum = second.reachedLocalOptimum;
    combined.hitMoveLimit = second.hitMoveLimit;
    combined.hitDistanceCallLimit = second.hitDistanceCallLimit;
    combined.operatorStats = first.operatorStats;
    add_operator_stats(combined.operatorStats, second);
    return combined;
}

LocalSearchAllocationContext make_context(
    const Individual& individual,
    const ParentCandidate& reference,
    double triggerUpperBound,
    double budgetProgress,
    double upperStagnation,
    double lowerStagnation,
    double populationDispersion,
    double recentGammaRate,
    const LocalSearchResult& weakResult,
    std::uint64_t evaluationLimitDistanceCalls,
    std::size_t populationSize) {
    const ParentCandidate candidate =
        Reproduction::make_parent_candidate(individual);
    LocalSearchAllocationContext context;
    context.qualityGap = reference.upperCost > 0.0
        ? (candidate.upperCost - reference.upperCost)
            / reference.upperCost
        : 0.0;
    context.adjacencyDistance = Reproduction::adjacency_distance(
        candidate,
        reference);
    context.budgetProgress = budgetProgress;
    context.gammaMargin = triggerUpperBound > 0.0
        ? (candidate.upperCost - triggerUpperBound)
            / triggerUpperBound
        : 0.0;
    context.upperStagnation = upperStagnation;
    context.lowerStagnation = lowerStagnation;
    context.populationDispersion = populationDispersion;
    context.recentGammaRate = recentGammaRate;
    context.probeSuccessRate = weakResult.neighborhoodCalls > 0
        ? static_cast<double>(weakResult.acceptedMoves)
            / static_cast<double>(weakResult.neighborhoodCalls)
        : 0.0;
    context.probeRelativeGain = weakResult.relativeUpperImprovement;
    if (evaluationLimitDistanceCalls > 0) {
        const double budgetFraction =
            static_cast<double>(weakResult.distanceCallsUsed)
            / static_cast<double>(evaluationLimitDistanceCalls);
        context.probeEfficiency = budgetFraction > 0.0
            ? bounded_nonnegative(
                weakResult.relativeUpperImprovement / budgetFraction)
            : 0.0;
        context.probeCost = std::clamp(
            budgetFraction
                * static_cast<double>(populationSize),
            0.0,
            1.0);
    }
    return context;
}

void finish_record(
    AllocatedLocalSearchRecord& record,
    double triggerUpperBound) {
    record.costAfterTerminal = record.individual->get_upper_cost();
    record.crossedGamma =
        record.costBeforeWeak > triggerUpperBound
        && record.costAfterTerminal <= triggerUpperBound;
    record.postProbeGammaCross =
        record.costAfterWeak > triggerUpperBound
        && record.costAfterTerminal <= triggerUpperBound;
}

void aggregate_run_operator_stats(LocalSearchAllocationRun& run) {
    run.operatorStats = {};
    for (const auto& record : run.records) {
        add_operator_stats(run.operatorStats, record.totalResult);
    }
}

std::array<double, 3> default_incremental_cost_units() {
    return {0.0, 3.0, 15.0};
}

}  // namespace

LocalSearchActionRatios load_instance_matched_ratios(
    const std::string& ratioFile,
    const std::string& instanceName) {
    std::ifstream input(ratioFile);
    if (!input.is_open()) {
        throw std::runtime_error(
            "failed to open instance-matched ratio file: " + ratioFile);
    }

    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream row(line);
        std::string name;
        std::string weak;
        std::string medium;
        std::string deepest;
        if (!std::getline(row, name, '\t')
            || !std::getline(row, weak, '\t')
            || !std::getline(row, medium, '\t')
            || !std::getline(row, deepest, '\t')) {
            throw std::runtime_error(
                "malformed instance-matched ratio row "
                + std::to_string(lineNumber)
                + " in " + ratioFile);
        }
        if (name == "instance") {
            continue;
        }
        if (name != instanceName) {
            continue;
        }
        try {
            return normalized_ratios(
                {
                    std::stod(weak),
                    std::stod(medium),
                    std::stod(deepest),
                },
                ratioFile + ":" + std::to_string(lineNumber));
        } catch (const std::invalid_argument&) {
            throw std::runtime_error(
                "non-numeric instance-matched ratios for "
                + instanceName + " in " + ratioFile);
        } catch (const std::out_of_range&) {
            throw std::runtime_error(
                "out-of-range instance-matched ratios for "
                + instanceName + " in " + ratioFile);
        }
    }
    throw std::runtime_error(
        "missing instance-matched ratios for "
        + instanceName + " in " + ratioFile);
}

LinearUcbModel::LinearUcbModel() {
    reset();
}

void LinearUcbModel::reset() {
    inverseGram = {};
    targets = {};
    coefficients = {};
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        inverseGram[feature][feature] = 1.0 / kRidge;
    }
}

LinearUcbEstimate LinearUcbModel::estimate(
    const LocalSearchAllocationContext& context) const {
    const FeatureVector contextFeatures = features(context);
    const FeatureVector projectedContext = multiply(
        inverseGram,
        contextFeatures);
    LinearUcbEstimate estimate;
    estimate.prediction = dot(coefficients, contextFeatures);
    estimate.uncertainty = std::sqrt(std::max(
        0.0,
        dot(contextFeatures, projectedContext)));
    return estimate;
}

void LinearUcbModel::update(
    const LocalSearchAllocationContext& context,
    double target) {
    const FeatureVector contextFeatures = features(context);
    const FeatureVector projectedContext = multiply(
        inverseGram,
        contextFeatures);
    const double denominator =
        1.0 + dot(contextFeatures, projectedContext);

    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            inverseGram[row][column] -=
                projectedContext[row]
                * projectedContext[column]
                / denominator;
        }
        targets[row] += contextFeatures[row] * target;
    }
    coefficients = multiply(inverseGram, targets);
}

const LinearUcbModel::FeatureVector&
LinearUcbModel::coefficient_values() const {
    return coefficients;
}

LinearUcbModel::FeatureVector LinearUcbModel::feature_values(
    const LocalSearchAllocationContext& context) {
    return features(context);
}

const std::array<const char*, LinearUcbModel::FEATURE_COUNT>&
LinearUcbModel::feature_names() {
    static const std::array<const char*, FEATURE_COUNT> names = {
        "intercept",
        "quality_gap",
        "adjacency_distance",
        "budget_progress",
        "gamma_margin",
        "upper_stagnation",
        "lower_stagnation",
        "population_dispersion",
        "recent_gamma_rate",
        "probe_success_rate",
        "probe_relative_gain",
        "probe_efficiency",
        "probe_cost",
        "quality_x_distance",
    };
    return names;
}

LinearUcbModel::FeatureVector LinearUcbModel::features(
    const LocalSearchAllocationContext& context) {
    const double qualityGap = bounded_nonnegative(
        context.qualityGap);
    const double adjacencyDistance = std::clamp(
        context.adjacencyDistance,
        0.0,
        1.0);
    return {
        1.0,
        qualityGap,
        adjacencyDistance,
        std::clamp(context.budgetProgress, 0.0, 1.0),
        std::tanh(context.gammaMargin),
        std::clamp(context.upperStagnation, 0.0, 1.0),
        std::clamp(context.lowerStagnation, 0.0, 1.0),
        std::clamp(context.populationDispersion, 0.0, 1.0),
        std::clamp(context.recentGammaRate, 0.0, 1.0),
        std::clamp(context.probeSuccessRate, 0.0, 1.0),
        bounded_nonnegative(100.0 * context.probeRelativeGain),
        std::clamp(context.probeEfficiency, 0.0, 1.0),
        std::clamp(context.probeCost, 0.0, 1.0),
        qualityGap * adjacencyDistance,
    };
}

LinearUcbModel::FeatureVector LinearUcbModel::multiply(
    const Matrix& matrix,
    const FeatureVector& vector) {
    FeatureVector result{};
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    return result;
}

double LinearUcbModel::dot(
    const FeatureVector& first,
    const FeatureVector& second) {
    double result = 0.0;
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        result += first[feature] * second[feature];
    }
    return result;
}

void OnlineIntensityLearner::reset() {
    for (auto& model : rewardModels) {
        model.reset();
    }
    for (auto& model : logCostModels) {
        model.reset();
    }
    observationCounts = {};
    featureSums = {};
    featureSquaredSums = {};
    lowerArchive.clear();
}

void OnlineIntensityLearner::set_cost_penalty(double newCostPenalty) {
    if (!std::isfinite(newCostPenalty) || newCostPenalty < 0.0) {
        throw std::invalid_argument(
            "local-search cost penalty must be finite and non-negative");
    }
    costPenalty = newCostPenalty;
}

double OnlineIntensityLearner::cost_penalty() const {
    return costPenalty;
}

LocalSearchIntensityDecision OnlineIntensityLearner::select(
    const LocalSearchAllocationContext& context,
    int generation,
    std::mt19937& randomEngine,
    LocalSearchIntensity deepestIntensity) const {
    const std::array<LocalSearchIntensity, 3> intensities = {
        LocalSearchIntensity::Weak,
        LocalSearchIntensity::Medium,
        deepestIntensity,
    };
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    std::uniform_int_distribution<std::size_t> randomAction(
        0,
        intensities.size() - 1);

    LocalSearchIntensityDecision decision;
    if (generation <= 1
        || probability(randomEngine) < kRandomExplorationRate) {
        decision.intensity = intensities[randomAction(randomEngine)];
        decision.score = score(context, decision.intensity);
        decision.exploratory = true;
        return decision;
    }

    std::array<double, 3> scores{};
    for (std::size_t actionIndex = 0;
         actionIndex < intensities.size();
         ++actionIndex) {
        scores[actionIndex] = score(
            context,
            intensities[actionIndex]);
    }
    const double bestScore = *std::max_element(
        scores.begin(),
        scores.end());
    std::vector<std::size_t> bestActions;
    for (std::size_t actionIndex = 0;
         actionIndex < scores.size();
         ++actionIndex) {
        if (std::fabs(scores[actionIndex] - bestScore) <= 1e-12) {
            bestActions.push_back(actionIndex);
        }
    }
    std::uniform_int_distribution<std::size_t> tieBreaker(
        0,
        bestActions.size() - 1);
    const std::size_t selectedIndex =
        bestActions[tieBreaker(randomEngine)];
    decision.intensity = intensities[selectedIndex];
    decision.score = scores[selectedIndex];
    return decision;
}

void OnlineIntensityLearner::update(
    const LocalSearchAllocationContext& context,
    LocalSearchIntensity intensity,
    double reward,
    double incrementalCostUnits) {
    const std::size_t actionIndex =
        LocalSearchAllocationRunner::intensity_index(intensity);
    rewardModels[actionIndex].update(
        context,
        std::clamp(reward, 0.0, 1.0));
    logCostModels[actionIndex].update(
        context,
        std::log1p(std::max(0.0, incrementalCostUnits)));
    const LinearUcbModel::FeatureVector features =
        LinearUcbModel::feature_values(context);
    for (std::size_t featureIndex = 0;
         featureIndex < features.size();
         ++featureIndex) {
        featureSums[actionIndex][featureIndex] +=
            features[featureIndex];
        featureSquaredSums[actionIndex][featureIndex] +=
            features[featureIndex] * features[featureIndex];
    }
    ++observationCounts[actionIndex];
}

std::array<IntensityModelSnapshot, 3>
OnlineIntensityLearner::model_snapshots(
    LocalSearchIntensity deepestIntensity) const {
    const std::array<LocalSearchIntensity, 3> intensities = {
        LocalSearchIntensity::Weak,
        LocalSearchIntensity::Medium,
        deepestIntensity,
    };
    std::array<IntensityModelSnapshot, 3> snapshots{};
    for (std::size_t actionIndex = 0;
         actionIndex < snapshots.size();
         ++actionIndex) {
        auto& snapshot = snapshots[actionIndex];
        snapshot.action =
            LocalSearchAllocationRunner::intensity_name(
                intensities[actionIndex]);
        snapshot.observations = observationCounts[actionIndex];
        snapshot.rewardCoefficients =
            rewardModels[actionIndex].coefficient_values();
        snapshot.logCostCoefficients =
            logCostModels[actionIndex].coefficient_values();
        if (snapshot.observations <= 0) {
            continue;
        }
        const double observations = static_cast<double>(
            snapshot.observations);
        for (std::size_t featureIndex = 0;
             featureIndex < LinearUcbModel::FEATURE_COUNT;
             ++featureIndex) {
            const double mean =
                featureSums[actionIndex][featureIndex]
                / observations;
            const double secondMoment =
                featureSquaredSums[actionIndex][featureIndex]
                / observations;
            snapshot.featureMeans[featureIndex] = mean;
            snapshot.featureStandardDeviations[featureIndex] =
                std::sqrt(std::max(
                    0.0,
                    secondMoment - mean * mean));
        }
    }
    return snapshots;
}

std::vector<std::pair<const Individual*, double>>
OnlineIntensityLearner::update_lower_archive(
    const std::vector<std::shared_ptr<Individual>>& completeSolutions) {
    struct Candidate {
        const Individual* individual{};
        std::vector<int> chromosome;
        double lowerCost{};
        bool improvesPreviousArchive{};
    };

    std::vector<Candidate> candidates;
    for (const auto& individual : completeSolutions) {
        if (individual == nullptr
            || individual->get_lower_cost() >= INFEASIBLE_COST) {
            continue;
        }
        const std::vector<int> chromosome =
            individual->get_chromosome();
        const auto previous = std::find_if(
            lowerArchive.begin(),
            lowerArchive.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == chromosome;
            });
        const bool improvesPrevious =
            previous == lowerArchive.end()
            || individual->get_lower_cost()
                < previous->lowerCost - 1e-12;

        const auto duplicate = std::find_if(
            candidates.begin(),
            candidates.end(),
            [&](const Candidate& candidate) {
                return candidate.chromosome == chromosome;
            });
        if (duplicate == candidates.end()) {
            candidates.push_back({
                individual.get(),
                chromosome,
                individual->get_lower_cost(),
                improvesPrevious,
            });
        } else if (individual->get_lower_cost()
                   < duplicate->lowerCost) {
            duplicate->individual = individual.get();
            duplicate->lowerCost = individual->get_lower_cost();
            duplicate->improvesPreviousArchive = improvesPrevious;
        }
    }

    std::vector<LowerArchiveEntry> merged = lowerArchive;
    for (const auto& candidate : candidates) {
        const auto existing = std::find_if(
            merged.begin(),
            merged.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == candidate.chromosome;
            });
        if (existing == merged.end()) {
            merged.push_back({
                candidate.chromosome,
                candidate.lowerCost,
            });
        } else if (candidate.lowerCost < existing->lowerCost) {
            existing->lowerCost = candidate.lowerCost;
        }
    }
    std::sort(
        merged.begin(),
        merged.end(),
        [](const LowerArchiveEntry& first,
           const LowerArchiveEntry& second) {
            if (std::fabs(first.lowerCost - second.lowerCost) > 1e-12) {
                return first.lowerCost < second.lowerCost;
            }
            return first.chromosome < second.chromosome;
        });
    if (merged.size() > LOWER_ARCHIVE_CAPACITY) {
        merged.resize(LOWER_ARCHIVE_CAPACITY);
    }
    lowerArchive.swap(merged);

    std::vector<std::pair<const Individual*, double>> credits;
    if (lowerArchive.empty()) {
        return credits;
    }
    for (const auto& candidate : candidates) {
        if (!candidate.improvesPreviousArchive) {
            continue;
        }
        const auto archived = std::find_if(
            lowerArchive.begin(),
            lowerArchive.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == candidate.chromosome
                    && std::fabs(
                        entry.lowerCost - candidate.lowerCost)
                        <= 1e-12;
            });
        if (archived == lowerArchive.end()) {
            continue;
        }
        const std::size_t rank = static_cast<std::size_t>(
            std::distance(lowerArchive.begin(), archived));
        const double credit =
            static_cast<double>(lowerArchive.size() - rank)
            / static_cast<double>(lowerArchive.size());
        credits.emplace_back(candidate.individual, credit);
    }
    return credits;
}

double OnlineIntensityLearner::score(
    const LocalSearchAllocationContext& context,
    LocalSearchIntensity intensity) const {
    const std::size_t actionIndex =
        LocalSearchAllocationRunner::intensity_index(intensity);
    const LinearUcbEstimate rewardEstimate =
        rewardModels[actionIndex].estimate(context);
    const double optimisticReward = std::max(
        0.0,
        rewardEstimate.prediction
            + kExplorationScale * rewardEstimate.uncertainty);

    double estimatedCost =
        default_incremental_cost_units()[actionIndex];
    if (observationCounts[actionIndex] > 0) {
        const LinearUcbEstimate costEstimate =
            logCostModels[actionIndex].estimate(context);
        estimatedCost = std::max(
            0.0,
            std::expm1(std::clamp(
                costEstimate.prediction,
                0.0,
                std::log1p(1e6))));
    }
    return optimisticReward
        - costPenalty * std::log1p(estimatedCost);
}

int OnlineIntensityLearner::observation_count(
    LocalSearchIntensity intensity) const {
    return observationCounts[
        LocalSearchAllocationRunner::intensity_index(intensity)];
}

LocalSearchAllocationRun LocalSearchAllocationRunner::run(
    const std::vector<std::shared_ptr<Individual>>& population,
    const std::shared_ptr<Individual>& unchangedLowerElite,
    Case& instance,
    int generation,
    LocalSearchPolicy policy,
    LocalSearchIntensity deepestIntensity,
    const ParentCandidate& upperReference,
    double triggerUpperBound,
    double budgetProgress,
    double upperStagnation,
    double lowerStagnation,
    double populationDispersion,
    double recentGammaRate,
    std::mt19937& localSearchEngine,
    std::mt19937& allocationEngine,
    std::vector<LocalSearchWorkspace>& workspaces,
    const OnlineIntensityLearner& learner,
    const LocalSearchDepthConfig& depthConfig,
    const LocalSearchActionRatios& instanceMatchedRatios) {
    if (policy == LocalSearchPolicy::Static) {
        throw std::logic_error(
            "static local search does not use the allocation runner");
    }

    LocalSearchAllocationRun run;
    run.records.reserve(population.size());
    const std::size_t requiredWorkspaces = std::min(
        kWorkspaceBatchSize,
        population.size());
    if (workspaces.size() < requiredWorkspaces) {
        workspaces.resize(requiredWorkspaces);
    }

    std::vector<std::shared_ptr<Individual>> searchCandidates;
    searchCandidates.reserve(population.size());
    for (const auto& individual : population) {
        const bool canReuseLowerElite =
            individual == unchangedLowerElite
            && individual->is_upper_locally_optimal()
            && individual->get_lower_cost() < INFEASIBLE_COST;
        if (!canReuseLowerElite) {
            searchCandidates.push_back(individual);
        }
    }
    if (policy == LocalSearchPolicy::InstanceMatchedRandom) {
        std::shuffle(
            searchCandidates.begin(),
            searchCandidates.end(),
            allocationEngine);
    }

    std::vector<LocalSearchIntensity> matchedIntensities;
    if (policy == LocalSearchPolicy::MatchedRandom) {
        const std::size_t candidateCount = searchCandidates.size();
        const std::size_t deepestCount = std::min(
            candidateCount,
            static_cast<std::size_t>(std::lround(
                static_cast<double>(candidateCount)
                * kMatchedDeepestRatio)));
        const std::size_t mediumCount = std::min(
            candidateCount - deepestCount,
            static_cast<std::size_t>(std::lround(
                static_cast<double>(candidateCount)
                * kMatchedMediumRatio)));
        matchedIntensities.assign(
            candidateCount - mediumCount - deepestCount,
            LocalSearchIntensity::Weak);
        matchedIntensities.insert(
            matchedIntensities.end(),
            mediumCount,
            LocalSearchIntensity::Medium);
        matchedIntensities.insert(
            matchedIntensities.end(),
            deepestCount,
            deepestIntensity);
        std::shuffle(
            matchedIntensities.begin(),
            matchedIntensities.end(),
            allocationEngine);
    }
    std::array<std::size_t, 3> instanceMatchedAssignedCounts{};
    std::size_t instanceMatchedEligibleCount = 0;
    const LocalSearchActionRatios normalizedInstanceRatios =
        policy == LocalSearchPolicy::InstanceMatchedRandom
        ? normalized_ratios(
            instanceMatchedRatios,
            "instance-matched runtime configuration")
        : LocalSearchActionRatios{};

    for (std::size_t batchStart = 0;
         batchStart < searchCandidates.size();
         batchStart += kWorkspaceBatchSize) {
        const std::size_t batchSize = std::min(
            kWorkspaceBatchSize,
            searchCandidates.size() - batchStart);
        const std::size_t recordStart = run.records.size();

        for (std::size_t localIndex = 0;
             localIndex < batchSize;
             ++localIndex) {
            AllocatedLocalSearchRecord record;
            record.individual =
                searchCandidates[batchStart + localIndex];
            record.costBeforeWeak =
                record.individual->get_upper_cost();
            record.boundedStrongMoveLimit =
                Leader::move_limit_for_intensity(
                    *record.individual,
                    instance,
                    LocalSearchIntensity::BoundedStrong,
                    depthConfig);
            Leader::begin_eight_neighborhood_rvnd_one_move_session(
                *record.individual,
                record.session,
                workspaces[localIndex]);
            const int weakLimit =
                Leader::move_limit_for_intensity(
                    *record.individual,
                    instance,
                    LocalSearchIntensity::Weak,
                    depthConfig);
            record.weakResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    weakLimit,
                    workspaces[localIndex],
                    triggerUpperBound);
            record.costAfterWeak =
                record.individual->get_upper_cost();
            record.boundedStrongDistanceCallLimit =
                Leader::bounded_strong_distance_call_limit(
                    record.weakResult.distanceCallsUsed,
                    depthConfig);
            record.costAfterTerminal = record.costAfterWeak;
            record.totalResult = record.weakResult;
            if (policy == LocalSearchPolicy::OnlineIndividual) {
                record.context = make_context(
                    *record.individual,
                    upperReference,
                    triggerUpperBound,
                    budgetProgress,
                    upperStagnation,
                    lowerStagnation,
                    populationDispersion,
                    recentGammaRate,
                    record.weakResult,
                    instance.get_evaluation_limit_distance_calls(),
                    population.size());
            }
            run.records.push_back(std::move(record));
        }

        std::vector<LocalSearchIntensity> instanceBatchIntensities(
            batchSize,
            LocalSearchIntensity::Weak);
        if (policy == LocalSearchPolicy::InstanceMatchedRandom) {
            std::vector<std::size_t> eligibleLocalIndexes;
            eligibleLocalIndexes.reserve(batchSize);
            for (std::size_t localIndex = 0;
                 localIndex < batchSize;
                 ++localIndex) {
                if (!run.records[recordStart + localIndex]
                         .weakResult.reachedLocalOptimum) {
                    eligibleLocalIndexes.push_back(localIndex);
                }
            }
            auto assignments = make_balanced_random_intensities(
                eligibleLocalIndexes.size(),
                normalizedInstanceRatios,
                deepestIntensity,
                instanceMatchedAssignedCounts,
                instanceMatchedEligibleCount,
                allocationEngine);
            for (std::size_t assignmentIndex = 0;
                 assignmentIndex < eligibleLocalIndexes.size();
                 ++assignmentIndex) {
                instanceBatchIntensities[
                    eligibleLocalIndexes[assignmentIndex]] =
                        assignments[assignmentIndex];
            }
        }

        if (policy == LocalSearchPolicy::MatchedRandom
            || policy == LocalSearchPolicy::InstanceMatchedRandom
            || policy == LocalSearchPolicy::OnlineNonContextual
            || policy == LocalSearchPolicy::OnlineIndividual) {
            for (std::size_t localIndex = 0;
                 localIndex < batchSize;
                 ++localIndex) {
                auto& record =
                    run.records[recordStart + localIndex];
                if (record.weakResult.reachedLocalOptimum) {
                    record.terminalIntensity =
                        LocalSearchIntensity::Weak;
                    record.forcedLocalOptimum = true;
                    finish_record(record, triggerUpperBound);
                    continue;
                }

                LocalSearchIntensityDecision decision;
                if (policy == LocalSearchPolicy::MatchedRandom) {
                    decision.intensity =
                        matchedIntensities[batchStart + localIndex];
                } else if (policy
                    == LocalSearchPolicy::InstanceMatchedRandom) {
                    decision.intensity =
                        instanceBatchIntensities[localIndex];
                } else {
                    decision = learner.select(
                        record.context,
                        generation,
                        allocationEngine,
                        deepestIntensity);
                }
                record.terminalIntensity = decision.intensity;
                record.selectionScore = decision.score;
                record.exploratorySelection =
                    decision.exploratory;

                if (decision.intensity
                    != LocalSearchIntensity::Weak) {
                    const bool boundedStrong =
                        decision.intensity
                        == LocalSearchIntensity::BoundedStrong;
                    const int moveLimit =
                        decision.intensity
                            == LocalSearchIntensity::Strong
                        ? -1
                        : boundedStrong
                            ? record.boundedStrongMoveLimit
                            : Leader::move_limit_for_intensity(
                                *record.individual,
                                instance,
                                LocalSearchIntensity::Medium,
                                depthConfig);
                    record.continuationResult =
                        Leader::continue_eight_neighborhood_rvnd_one_move_session(
                            *record.individual,
                            instance,
                            localSearchEngine,
                            record.session,
                            moveLimit,
                            workspaces[localIndex],
                            triggerUpperBound,
                            boundedStrong
                                ? record.boundedStrongDistanceCallLimit
                                : std::numeric_limits<
                                    std::uint64_t>::max());
                    record.totalResult = combine_results(
                        record.weakResult,
                        record.continuationResult,
                        record.costBeforeWeak,
                        record.individual->get_upper_cost());
                }
                finish_record(record, triggerUpperBound);
            }
            continue;
        }

        std::vector<std::size_t> mediumEligible;
        for (std::size_t localIndex = 0;
             localIndex < batchSize;
             ++localIndex) {
            const std::size_t recordIndex =
                recordStart + localIndex;
            if (!run.records[recordIndex]
                     .weakResult.reachedLocalOptimum) {
                mediumEligible.push_back(recordIndex);
            }
        }
        std::shuffle(
            mediumEligible.begin(),
            mediumEligible.end(),
            allocationEngine);
        const std::size_t mediumTarget = std::min(
            mediumEligible.size(),
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batchSize)
                * kMediumSelectionRatio)));
        mediumEligible.resize(mediumTarget);

        if (deepestIntensity
            == LocalSearchIntensity::BoundedStrong) {
            std::vector<std::size_t> boundedStrongSelected =
                mediumEligible;
            std::shuffle(
                boundedStrongSelected.begin(),
                boundedStrongSelected.end(),
                allocationEngine);
            const std::size_t boundedStrongTarget = std::min(
                boundedStrongSelected.size(),
                static_cast<std::size_t>(std::lround(
                    static_cast<double>(batchSize)
                    * kStrongSelectionRatio)));
            boundedStrongSelected.resize(boundedStrongTarget);

            for (const std::size_t recordIndex : mediumEligible) {
                const std::size_t localIndex =
                    recordIndex - recordStart;
                auto& record = run.records[recordIndex];
                const bool useBoundedStrong =
                    std::find(
                        boundedStrongSelected.begin(),
                        boundedStrongSelected.end(),
                        recordIndex)
                    != boundedStrongSelected.end();
                record.terminalIntensity = useBoundedStrong
                    ? LocalSearchIntensity::BoundedStrong
                    : LocalSearchIntensity::Medium;
                const int moveLimit = useBoundedStrong
                    ? record.boundedStrongMoveLimit
                    : Leader::move_limit_for_intensity(
                        *record.individual,
                        instance,
                        LocalSearchIntensity::Medium,
                        depthConfig);
                record.continuationResult =
                    Leader::continue_eight_neighborhood_rvnd_one_move_session(
                        *record.individual,
                        instance,
                        localSearchEngine,
                        record.session,
                        moveLimit,
                        workspaces[localIndex],
                        triggerUpperBound,
                        useBoundedStrong
                            ? record.boundedStrongDistanceCallLimit
                            : std::numeric_limits<
                                std::uint64_t>::max());
                record.totalResult = combine_results(
                    record.weakResult,
                    record.continuationResult,
                    record.costBeforeWeak,
                    record.individual->get_upper_cost());
            }
            for (std::size_t localIndex = 0;
                 localIndex < batchSize;
                 ++localIndex) {
                finish_record(
                    run.records[recordStart + localIndex],
                    triggerUpperBound);
            }
            continue;
        }

        for (const std::size_t recordIndex : mediumEligible) {
            const std::size_t localIndex =
                recordIndex - recordStart;
            auto& record = run.records[recordIndex];
            record.terminalIntensity =
                LocalSearchIntensity::Medium;
            const int mediumLimit =
                Leader::move_limit_for_intensity(
                    *record.individual,
                    instance,
                    LocalSearchIntensity::Medium,
                    depthConfig);
            record.continuationResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    mediumLimit,
                    workspaces[localIndex],
                    triggerUpperBound);
            record.totalResult = combine_results(
                record.weakResult,
                record.continuationResult,
                record.costBeforeWeak,
                record.individual->get_upper_cost());
        }

        std::vector<std::size_t> strongEligible;
        for (const std::size_t recordIndex : mediumEligible) {
            if (!run.records[recordIndex]
                     .continuationResult.reachedLocalOptimum) {
                strongEligible.push_back(recordIndex);
            }
        }
        std::shuffle(
            strongEligible.begin(),
            strongEligible.end(),
            allocationEngine);
        const std::size_t strongTarget = std::min(
            strongEligible.size(),
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batchSize)
                * kStrongSelectionRatio)));
        strongEligible.resize(strongTarget);

        for (const std::size_t recordIndex : strongEligible) {
            const std::size_t localIndex =
                recordIndex - recordStart;
            auto& record = run.records[recordIndex];
            const LocalSearchResult mediumResult =
                record.continuationResult;
            record.terminalIntensity =
                deepestIntensity;
            const bool boundedStrong =
                deepestIntensity
                == LocalSearchIntensity::BoundedStrong;
            const LocalSearchResult strongResult =
                Leader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    localSearchEngine,
                    record.session,
                    boundedStrong
                        ? record.boundedStrongMoveLimit
                        : -1,
                    workspaces[localIndex],
                    triggerUpperBound,
                    boundedStrong
                        ? record.boundedStrongDistanceCallLimit
                        : std::numeric_limits<std::uint64_t>::max());
            record.continuationResult = combine_results(
                mediumResult,
                strongResult,
                record.costAfterWeak,
                record.individual->get_upper_cost());
            record.totalResult = combine_results(
                record.weakResult,
                record.continuationResult,
                record.costBeforeWeak,
                record.individual->get_upper_cost());
        }

        for (std::size_t localIndex = 0;
             localIndex < batchSize;
             ++localIndex) {
            auto& record =
                run.records[recordStart + localIndex];
            finish_record(record, triggerUpperBound);
        }
    }

    aggregate_run_operator_stats(run);
    return run;
}

void LocalSearchAllocationRunner::assign_parent_use_feedback(
    LocalSearchAllocationRun& run,
    const std::vector<ParentCandidate>& parentPool,
    const std::vector<int>& parentUseCounts) {
    if (parentPool.size() != parentUseCounts.size()) {
        throw std::invalid_argument(
            "parent use counts must match parent pool size");
    }
    int totalParentUses = 0;
    std::size_t scoredParentCount = 0;
    for (std::size_t parentIndex = 0;
         parentIndex < parentPool.size();
         ++parentIndex) {
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get()
                    == parentPool[parentIndex].source;
            });
        if (record != run.records.end()
            && record->excludeFromLearnerFeedback) {
            continue;
        }
        totalParentUses += parentUseCounts[parentIndex];
        ++scoredParentCount;
    }
    if (totalParentUses <= 0 || scoredParentCount == 0) {
        return;
    }
    const double averageParentUses =
        static_cast<double>(totalParentUses)
        / static_cast<double>(scoredParentCount);
    for (std::size_t parentIndex = 0;
         parentIndex < parentPool.size();
         ++parentIndex) {
        if (parentUseCounts[parentIndex] <= 0
            || parentPool[parentIndex].source == nullptr) {
            continue;
        }
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get()
                    == parentPool[parentIndex].source;
            });
        if (record == run.records.end()
            || record->excludeFromLearnerFeedback) {
            continue;
        }
        record->parentUseCount += parentUseCounts[parentIndex];
        record->parentCredit = std::min(
            1.0,
            record->parentCredit
                + static_cast<double>(parentUseCounts[parentIndex])
                    / averageParentUses);
    }
}

std::vector<std::pair<const Individual*, double>>
LocalSearchAllocationRunner::assign_lower_archive_feedback(
    LocalSearchAllocationRun& run,
    const std::vector<std::shared_ptr<Individual>>& completeSolutions,
    OnlineIntensityLearner& learner) {
    const auto credits = learner.update_lower_archive(
        completeSolutions);
    for (const auto& creditEntry : credits) {
        const Individual* individual = creditEntry.first;
        const double credit = creditEntry.second;
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get() == individual;
            });
        if (record != run.records.end()
            && !record->excludeFromLearnerFeedback) {
            record->lowerCredit += credit;
        }
    }
    return credits;
}

void LocalSearchAllocationRunner::finalize_feedback(
    LocalSearchAllocationRun& run,
    LocalSearchPolicy policy,
    OnlineIntensityLearner& learner) {
    for (auto& record : run.records) {
        // The allocation action starts after the common weak probe.
        const double continuationRelativeGain =
            record.costAfterWeak > 0.0
            ? std::max(
                0.0,
                (record.costAfterWeak
                    - record.costAfterTerminal)
                    / record.costAfterWeak)
            : 0.0;
        record.normalizedContinuationGain =
            bounded_nonnegative(
                kContinuationGainScale
                    * continuationRelativeGain);
        const double parentReward =
            kParentRewardWeight
            * std::clamp(record.parentCredit, 0.0, 1.0);
        const double lowerReward =
            kLowerRewardWeight
            * std::clamp(record.lowerCredit, 0.0, 1.0);
        const double gammaReward =
            record.postProbeGammaCross
            ? kGammaRewardWeight
            : 0.0;
        const double continuationGainSignal =
            kContinuationGainSignalWeight
            * record.normalizedContinuationGain;
        record.reward =
            parentReward
            + lowerReward
            + gammaReward;

        const std::uint64_t weakDistanceCalls =
            std::max<std::uint64_t>(
                1,
                record.weakResult.distanceCallsUsed);
        record.incrementalCostUnits =
            static_cast<double>(
                record.continuationResult.distanceCallsUsed)
            / static_cast<double>(weakDistanceCalls);
        if ((policy == LocalSearchPolicy::OnlineNonContextual
             || policy == LocalSearchPolicy::OnlineIndividual)
            && !record.excludeFromLearnerFeedback) {
            learner.update(
                record.context,
                record.terminalIntensity,
                record.reward,
                record.incrementalCostUnits);
        }

        auto& stats = run.stats[
            intensity_index(record.terminalIntensity)];
        ++stats.selections;
        stats.forcedLocalOptima += record.forcedLocalOptimum;
        stats.exploratorySelections +=
            record.exploratorySelection;
        stats.localOptimumTerminations +=
            record.totalResult.reachedLocalOptimum;
        stats.moveLimitTerminations +=
            record.totalResult.hitMoveLimit;
        stats.distanceLimitTerminations +=
            record.totalResult.hitDistanceCallLimit;
        stats.acceptedMoves +=
            record.totalResult.acceptedMoves;
        stats.neighborhoodCalls +=
            record.totalResult.neighborhoodCalls;
        stats.distanceCalls +=
            record.totalResult.distanceCallsUsed;
        stats.continuationDistanceCalls +=
            record.continuationResult.distanceCallsUsed;
        stats.upperGain +=
            result_upper_gain(record.totalResult);
        stats.gammaCrosses += record.crossedGamma;
        stats.parentUses += record.parentUseCount;
        stats.lowerArchiveEntries +=
            record.lowerCredit > 0.0;
        stats.parentReward += parentReward;
        stats.lowerReward += lowerReward;
        stats.gammaReward += gammaReward;
        stats.continuationGainSignal +=
            continuationGainSignal;
        stats.reward += record.reward;
        stats.incrementalCostUnits +=
            record.incrementalCostUnits;
        stats.selectionScore += record.selectionScore;
    }
}

std::size_t LocalSearchAllocationRunner::intensity_index(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Weak:
            return 0;
        case LocalSearchIntensity::Medium:
            return 1;
        case LocalSearchIntensity::BoundedStrong:
        case LocalSearchIntensity::Strong:
            return 2;
        case LocalSearchIntensity::Skip:
            break;
    }
    throw std::logic_error(
        "skip is not an allocated local-search intensity");
}

const char* LocalSearchAllocationRunner::intensity_name(
    LocalSearchIntensity intensity) {
    switch (intensity) {
        case LocalSearchIntensity::Skip:
            return "skip";
        case LocalSearchIntensity::Weak:
            return "weak";
        case LocalSearchIntensity::Medium:
            return "medium";
        case LocalSearchIntensity::BoundedStrong:
            return "bounded_strong";
        case LocalSearchIntensity::Strong:
            return "strong";
    }
    return "unknown";
}

const char* local_search_policy_name(LocalSearchPolicy policy) {
    switch (policy) {
        case LocalSearchPolicy::Static:
            return "static";
        case LocalSearchPolicy::RandomMixed:
            return "random";
        case LocalSearchPolicy::MatchedRandom:
            return "matched_random";
        case LocalSearchPolicy::InstanceMatchedRandom:
            return "instance_matched_random";
        case LocalSearchPolicy::OnlineNonContextual:
            return "non_contextual";
        case LocalSearchPolicy::OnlineIndividual:
            return "online";
    }
    return "unknown";
}
