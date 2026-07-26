#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "MA.hpp"
#include "elite_unlimited.hpp"
#include "initializer.hpp"
#include "local_search_allocation.hpp"
#include "reproduction.hpp"

namespace {

AllocatedLocalSearchRecord make_bounded_record(
    const std::shared_ptr<Individual>& individual) {
    AllocatedLocalSearchRecord record;
    record.individual = individual;
    record.terminalIntensity =
        LocalSearchIntensity::BoundedStrong;
    record.costAfterWeak = individual->get_upper_cost();
    record.costAfterTerminal = individual->get_upper_cost();
    record.weakResult.distanceCallsUsed = 1;
    return record;
}

}  // namespace

int main() {
    const std::string instancePath =
        std::string(TEST_DATA_DIRECTORY) + "/E-n22-k4.evrp";
    Case instance(instancePath, 51);
    std::mt19937 initializationEngine(51);
    const auto routes = Initializer::build_with_clustering(
        instance,
        initializationEngine);
    auto preferred = std::make_shared<Individual>(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));
    auto expensive = std::make_shared<Individual>(*preferred);
    expensive->set_upper_cost(
        preferred->get_upper_cost() + 1000.0);
    auto ineligible = std::make_shared<Individual>(*preferred);
    ineligible->set_upper_cost(
        preferred->get_upper_cost() - 1000.0);

    LocalSearchAllocationRun allocationRun;
    allocationRun.records.push_back(
        make_bounded_record(preferred));
    allocationRun.records.push_back(
        make_bounded_record(expensive));
    auto mediumRecord = make_bounded_record(ineligible);
    mediumRecord.terminalIntensity =
        LocalSearchIntensity::Medium;
    allocationRun.records.push_back(std::move(mediumRecord));

    EliteUnlimitedController controller;
    LocalSearchWorkspace workspace;
    std::mt19937 localSearchEngine(52);
    for (int generation = 1;
         generation <= EliteUnlimitedController::WARMUP_GENERATIONS;
         ++generation) {
        const EliteUnlimitedRun warmupRun = controller.run(
            allocationRun,
            instance,
            generation,
            1,
            preferred->get_upper_cost() * 0.99,
            localSearchEngine,
            workspace);
        assert(!warmupRun.triggered);
        assert(warmupRun.eligibleCandidates == 2);
    }
    const long double expectedWarmupCredit =
        static_cast<long double>(
            EliteUnlimitedController::WARMUP_GENERATIONS)
        * EliteUnlimitedController::CREDIT_RATIO;
    assert(std::fabs(static_cast<double>(
        controller.credit_distance_calls()
        - expectedWarmupCredit)) <= 1e-12);

    EliteUnlimitedRun eliteRun = controller.run(
        allocationRun,
        instance,
        EliteUnlimitedController::WARMUP_GENERATIONS + 1,
        1,
        preferred->get_upper_cost() * 0.99,
        localSearchEngine,
        workspace);
    assert(eliteRun.triggered);
    assert(eliteRun.eligibleCandidates == 2);
    assert(eliteRun.individual == preferred);
    assert(eliteRun.result.reachedLocalOptimum);
    assert(eliteRun.result.distanceCallsUsed > 0);
    assert(allocationRun.records[0].excludeFromLearnerFeedback);
    assert(!allocationRun.records[1].excludeFromLearnerFeedback);
    const long double expectedCredit =
        expectedWarmupCredit
        + EliteUnlimitedController::CREDIT_RATIO
        - static_cast<long double>(
            eliteRun.result.distanceCallsUsed);
    assert(std::fabs(
        static_cast<double>(
            controller.credit_distance_calls()
            - expectedCredit)) <= 1e-9);
    assert(controller.credit_distance_calls() < 0.0L);

    const EliteUnlimitedRun debtRun = controller.run(
        allocationRun,
        instance,
        EliteUnlimitedController::WARMUP_GENERATIONS + 2,
        1,
        preferred->get_upper_cost() * 0.99,
        localSearchEngine,
        workspace);
    assert(!debtRun.triggered);

    std::vector<ParentCandidate> parentPool = {
        Reproduction::make_parent_candidate(*preferred),
        Reproduction::make_parent_candidate(*expensive),
    };
    const std::vector<int> parentUseCounts = {9, 1};
    EliteUnlimitedController::assign_parent_use_feedback(
        eliteRun,
        parentPool,
        parentUseCounts);
    EliteUnlimitedController::assign_lower_archive_feedback(
        eliteRun,
        {{preferred.get(), 1.0}});
    eliteRun.verifiedImprovements = 1;
    const EliteUnlimitedStats eliteStats =
        EliteUnlimitedController::make_stats(
            eliteRun,
            11,
            controller.credit_distance_calls());
    assert(eliteStats.triggers == 1);
    assert(eliteStats.parentUses == 9);
    assert(eliteStats.lowerArchiveEntries == 1);
    assert(eliteStats.verifiedImprovements == 1);

    LocalSearchAllocationRun feedbackRun;
    auto excludedRecord = make_bounded_record(preferred);
    excludedRecord.excludeFromLearnerFeedback = true;
    auto ordinaryRecord = make_bounded_record(expensive);
    feedbackRun.records.push_back(std::move(excludedRecord));
    feedbackRun.records.push_back(std::move(ordinaryRecord));
    LocalSearchAllocationRunner::assign_parent_use_feedback(
        feedbackRun,
        parentPool,
        parentUseCounts);
    assert(feedbackRun.records[0].parentUseCount == 0);
    assert(feedbackRun.records[1].parentUseCount == 1);

    OnlineIntensityLearner learner;
    learner.reset();
    LocalSearchAllocationRunner::finalize_feedback(
        feedbackRun,
        LocalSearchPolicy::OnlineIndividual,
        learner);
    assert(
        learner.observation_count(
            LocalSearchIntensity::BoundedStrong) == 1);
    const auto& boundedStats = feedbackRun.stats[
        LocalSearchAllocationRunner::intensity_index(
            LocalSearchIntensity::BoundedStrong)];
    assert(boundedStats.selections == 2);
    assert(boundedStats.parentUses == 1);

    Parameters parameters;
    parameters.localSearchPolicy =
        LocalSearchPolicy::OnlineIndividual;
    parameters.localSearchIntensity =
        LocalSearchIntensity::BoundedStrong;
    MA algorithm(&instance, parameters);
    algorithm.accumulate_elite_unlimited_stats(eliteStats);
    algorithm.write_elite_unlimited_snapshot();
    std::istringstream rows(algorithm.eliteUnlimitedRows.str());
    std::string row;
    assert(std::getline(rows, row));
    assert(!std::getline(rows, row));
    assert(std::count(row.begin(), row.end(), '\t') == 12);
    return 0;
}
