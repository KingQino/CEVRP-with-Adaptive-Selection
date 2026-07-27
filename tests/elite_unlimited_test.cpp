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
    const std::shared_ptr<Individual>& individual,
    double continuationGain = 0.0,
    std::uint64_t continuationDistanceCalls = 1,
    double adjacencyDistance = 0.0) {
    AllocatedLocalSearchRecord record;
    record.individual = individual;
    record.terminalIntensity =
        LocalSearchIntensity::BoundedStrong;
    record.costAfterWeak =
        individual->get_upper_cost() + continuationGain;
    record.costAfterTerminal = individual->get_upper_cost();
    record.weakResult.distanceCallsUsed = 1;
    record.continuationResult.distanceCallsUsed =
        continuationDistanceCalls;
    record.continuationResult.operatorStats[0].upperGain =
        continuationGain;
    record.context.adjacencyDistance = adjacencyDistance;
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
    auto lowestCost = std::make_shared<Individual>(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));
    auto efficientNear =
        std::make_shared<Individual>(*lowestCost);
    efficientNear->set_upper_cost(
        lowestCost->get_upper_cost() + 100.0);
    auto efficientFar =
        std::make_shared<Individual>(*lowestCost);
    efficientFar->set_upper_cost(
        lowestCost->get_upper_cost() + 200.0);
    auto lowerQuality =
        std::make_shared<Individual>(*lowestCost);
    lowerQuality->set_upper_cost(
        lowestCost->get_upper_cost() + 300.0);
    auto lowestQuality =
        std::make_shared<Individual>(*lowestCost);
    lowestQuality->set_upper_cost(
        lowestCost->get_upper_cost() + 400.0);
    auto ineligible = std::make_shared<Individual>(*lowestCost);
    ineligible->set_upper_cost(
        lowestCost->get_upper_cost() - 1000.0);

    LocalSearchAllocationRun allocationRun;
    allocationRun.records.push_back(
        make_bounded_record(lowestCost, 1.0, 100, 0.9));
    allocationRun.records.push_back(
        make_bounded_record(efficientNear, 10.0, 100, 0.2));
    allocationRun.records.push_back(
        make_bounded_record(efficientFar, 20.0, 200, 0.8));
    allocationRun.records.push_back(
        make_bounded_record(lowerQuality, 100.0, 1, 1.0));
    allocationRun.records.push_back(
        make_bounded_record(lowestQuality, 100.0, 1, 1.0));
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
            lowestCost->get_upper_cost() * 0.99,
            localSearchEngine,
            workspace);
        assert(!warmupRun.triggered);
        assert(warmupRun.eligibleCandidates == 5);
        assert(warmupRun.qualityCandidates == 3);
    }
    assert(std::fabs(
        static_cast<double>(controller.credit_distance_calls())
        - 1.0) <= 1e-12);

    EliteUnlimitedRun eliteRun = controller.run(
        allocationRun,
        instance,
        EliteUnlimitedController::WARMUP_GENERATIONS + 1,
        1,
        lowestCost->get_upper_cost() * 0.99,
        localSearchEngine,
        workspace);
    assert(eliteRun.triggered);
    assert(eliteRun.eligibleCandidates == 5);
    assert(eliteRun.qualityCandidates == 3);
    assert(eliteRun.selectedQualityRank == 3);
    assert(eliteRun.individual == efficientFar);
    assert(eliteRun.selectedProbeDistanceCalls == 200);
    assert(eliteRun.selectedProbeUpperGain == 20.0);
    assert(eliteRun.selectedAdjacencyDistance == 0.8);
    assert(eliteRun.result.reachedLocalOptimum);
    assert(eliteRun.result.distanceCallsUsed > 0);
    assert(!allocationRun.records[0].excludeFromLearnerFeedback);
    assert(!allocationRun.records[1].excludeFromLearnerFeedback);
    assert(allocationRun.records[2].excludeFromLearnerFeedback);
    assert(!allocationRun.records[3].excludeFromLearnerFeedback);
    const long double expectedCredit =
        1.1L
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
        lowestCost->get_upper_cost() * 0.99,
        localSearchEngine,
        workspace);
    assert(!debtRun.triggered);

    std::vector<ParentCandidate> parentPool = {
        Reproduction::make_parent_candidate(*efficientFar),
        Reproduction::make_parent_candidate(*efficientNear),
    };
    const std::vector<int> parentUseCounts = {9, 1};
    EliteUnlimitedController::assign_parent_use_feedback(
        eliteRun,
        parentPool,
        parentUseCounts);
    EliteUnlimitedController::assign_lower_archive_feedback(
        eliteRun,
        {{efficientFar.get(), 1.0}});
    eliteRun.verifiedImprovements = 1;
    const EliteUnlimitedStats eliteStats =
        EliteUnlimitedController::make_stats(
            eliteRun,
            11,
            controller.credit_distance_calls());
    assert(eliteStats.triggers == 1);
    assert(eliteStats.qualityCandidates == 3);
    assert(eliteStats.selectedQualityRank == 3);
    assert(eliteStats.selectedProbeDistanceCalls == 200);
    assert(eliteStats.selectedProbeUpperGain == 20.0);
    assert(eliteStats.selectedAdjacencyDistance == 0.8);
    assert(eliteStats.parentUses == 9);
    assert(eliteStats.lowerArchiveEntries == 1);
    assert(eliteStats.verifiedImprovements == 1);

    LocalSearchAllocationRun feedbackRun;
    auto excludedRecord = make_bounded_record(efficientFar);
    excludedRecord.excludeFromLearnerFeedback = true;
    auto ordinaryRecord = make_bounded_record(efficientNear);
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
    assert(std::count(row.begin(), row.end(), '\t') == 17);
    return 0;
}
