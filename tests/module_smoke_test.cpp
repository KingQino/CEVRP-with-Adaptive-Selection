#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "MA.hpp"
#include "algorithm_constants.hpp"
#include "follower.hpp"
#include "initializer.hpp"
#include "leader.hpp"
#include "reproduction.hpp"

namespace {

void assert_routes_cover_customers(
    const std::vector<std::vector<int>>& routes,
    const Case& instance) {
    std::vector<int> routedCustomers;
    for (const auto& route : routes) {
        assert(route.size() >= 2);
        assert(route.front() == instance.depot);
        assert(route.back() == instance.depot);
        routedCustomers.insert(routedCustomers.end(), route.begin() + 1, route.end() - 1);
    }

    std::vector<int> expectedCustomers = instance.customers;
    std::sort(routedCustomers.begin(), routedCustomers.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(routedCustomers == expectedCustomers);
}

void assert_is_customer_permutation(
    const std::vector<int>& chromosome,
    const Case& instance) {
    std::vector<int> sortedChromosome = chromosome;
    std::vector<int> expectedCustomers = instance.customers;
    std::sort(sortedChromosome.begin(), sortedChromosome.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(sortedChromosome == expectedCustomers);
}

void assert_individual_is_consistent(
    const Individual& individual,
    Case& instance) {
    const auto routes = individual.get_routes();
    assert(static_cast<int>(routes.size()) == individual.route_num);
    assert(individual.route_num <= individual.route_cap);
    assert_routes_cover_customers(routes, instance);

    double recomputedCost = 0.0;
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        const auto& route = routes[routeIndex];
        assert(static_cast<int>(route.size()) == individual.node_num[routeIndex]);

        int recomputedDemand = 0;
        for (std::size_t nodeIndex = 1; nodeIndex + 1 < route.size(); ++nodeIndex) {
            recomputedDemand += instance.get_customer_demand(route[nodeIndex]);
        }
        assert(recomputedDemand == individual.demand_sum[routeIndex]);
        assert(recomputedDemand <= instance.maxC);
        recomputedCost += instance.fitness_evaluation(route);
    }

    const double tolerance = 1e-7 * std::max(1.0, std::fabs(recomputedCost));
    assert(std::fabs(recomputedCost - individual.get_upper_cost()) <= tolerance);
}

void assert_inter_route_relocate_removes_empty_route(Case& instance) {
    int firstCustomer = -1;
    int secondCustomer = -1;
    for (std::size_t first = 0;
         first < instance.customers.size() && firstCustomer < 0;
         ++first) {
        for (std::size_t second = first + 1;
             second < instance.customers.size();
             ++second) {
            const int firstCandidate = instance.customers[first];
            const int secondCandidate = instance.customers[second];
            if (instance.get_customer_demand(firstCandidate)
                    + instance.get_customer_demand(secondCandidate) > instance.maxC) {
                continue;
            }

            const std::vector<int> firstRoute = {
                instance.depot,
                firstCandidate,
                instance.depot};
            const std::vector<int> secondRoute = {
                instance.depot,
                secondCandidate,
                instance.depot};
            const std::vector<int> mergedRoute = {
                instance.depot,
                firstCandidate,
                secondCandidate,
                instance.depot};
            if (instance.fitness_evaluation(mergedRoute) + 1e-8
                < instance.fitness_evaluation(firstRoute)
                    + instance.fitness_evaluation(secondRoute)) {
                firstCustomer = firstCandidate;
                secondCustomer = secondCandidate;
                break;
            }
        }
    }
    assert(firstCustomer >= 0 && secondCustomer >= 0);

    const std::vector<std::vector<int>> routes = {
        {instance.depot, firstCustomer, instance.depot},
        {instance.depot, secondCustomer, instance.depot},
    };
    Individual individual(
        2,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));
    // Seed 2 selects inter-route relocate first from the five-neighborhood list.
    std::mt19937 randomEngine(2);
    Leader::improve_with_five_neighborhood_rvnd(individual, instance, randomEngine);

    assert(individual.route_num == 1);
    std::vector<int> actualCustomers = individual.get_chromosome();
    std::vector<int> expectedCustomers = {firstCustomer, secondCustomer};
    std::sort(actualCustomers.begin(), actualCustomers.end());
    std::sort(expectedCustomers.begin(), expectedCustomers.end());
    assert(actualCustomers == expectedCustomers);
    assert(individual.demand_sum[0]
           == instance.get_customer_demand(firstCustomer)
               + instance.get_customer_demand(secondCustomer));
    const auto mergedRoutes = individual.get_routes();
    assert(mergedRoutes.size() == 1);
    assert(std::fabs(
        instance.fitness_evaluation(mergedRoutes[0])
        - individual.get_upper_cost()) <= 1e-8);
}

void assert_refinement_handles_boundary_cases() {
    const std::string instancePath =
        std::string(TEST_DATA_DIRECTORY) + "/RC108-10.evrp";
    Case instance(instancePath, 8);
    const std::vector<std::vector<int>> routes = {
        {0, 6, 8, 1, 10, 2, 3, 5, 7, 9, 4, 0},
    };
    const double upperCost = instance.fitness_evaluation(routes);
    const std::vector<int> demandSums = instance.compute_demand_sum(routes);

    Individual standard(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        upperCost,
        demandSums);
    Follower::optimize_charging(standard, instance);
    assert(std::isfinite(standard.get_lower_cost()));
    assert(standard.get_lower_cost() < INFEASIBLE_COST);

    Individual refined(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        upperCost,
        demandSums);
    Follower::refine_charging_by_enumeration(refined, instance);
    assert(std::isfinite(refined.get_lower_cost()));
    assert(refined.get_lower_cost() < INFEASIBLE_COST);
    assert(refined.get_lower_cost() <= standard.get_lower_cost() + 1e-8);

    const auto [tour, steps] = refined.get_tour();
    assert(steps > 1);
    assert(tour[0] == instance.depot);
    assert(tour[steps - 1] == instance.depot);
    double distanceSinceCharge = 0.0;
    for (int index = 1; index < steps; ++index) {
        distanceSinceCharge += instance.get_distance(tour[index - 1], tour[index]);
        if (instance.is_charging_station(tour[index])) {
            assert(distanceSinceCharge <= instance.maxDis + 1e-8);
            distanceSinceCharge = 0.0;
        }
    }

    Individual malformed(1, 2, {{}}, 0.0, {0});
    Follower::refine_charging_by_enumeration(malformed, instance);
    assert(malformed.get_lower_cost() >= INFEASIBLE_COST);

    Individual empty(1, 2);
    Follower::refine_charging_by_enumeration(empty, instance);
    assert(empty.get_lower_cost() >= INFEASIBLE_COST);

    Case noStationInstance(instancePath, 9);
    noStationInstance.stations.clear();
    noStationInstance.stationSet.clear();
    noStationInstance.stationNumber = 0;
    Individual noStation(
        noStationInstance.vehicleNumber * 3,
        noStationInstance.customerNumber + 2,
        routes,
        upperCost,
        demandSums);
    Follower::refine_charging_by_enumeration(noStation, noStationInstance);
    assert(noStation.get_lower_cost() >= INFEASIBLE_COST);

    Individual expandableTour(1, 2);
    std::vector<int> longRoute(Individual::TOUR_SIZE + 100, 1);
    longRoute.front() = 0;
    longRoute.back() = 0;
    expandableTour.set_tour({longRoute});
    const auto [expandedTour, expandedSteps] = expandableTour.get_tour();
    assert(expandedSteps == static_cast<int>(longRoute.size()));
    assert(expandedTour[expandedSteps - 1] == 0);
}

void assert_swap_star_improves_a_seven_neighborhood_local_optimum() {
    const std::string instancePath =
        std::string(TEST_DATA_DIRECTORY) + "/E-n22-k4.evrp";
    Case instance(instancePath, 10);
    std::mt19937 initializationEngine(6);
    const auto routes = Initializer::build_with_clustering(
        instance,
        initializationEngine);
    Individual sevenNeighborhoodLocalOptimum(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));

    std::mt19937 sevenEngine(17);
    const LocalSearchResult sevenResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            sevenNeighborhoodLocalOptimum,
            instance,
            sevenEngine,
            LocalSearchIntensity::Strong);
    assert(sevenResult.reachedLocalOptimum);

    Individual repeatedEightNeighborhoodSearch(
        sevenNeighborhoodLocalOptimum);
    const double sevenNeighborhoodCost =
        sevenNeighborhoodLocalOptimum.get_upper_cost();
    std::mt19937 firstEightEngine(31);
    std::mt19937 secondEightEngine(31);
    LocalSearchWorkspace workspace;
    workspace.firstRouteBuffer.assign(32, -1);
    workspace.secondRouteBuffer.assign(32, -1);
    const LocalSearchResult firstEightResult =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            sevenNeighborhoodLocalOptimum,
            instance,
            firstEightEngine,
            LocalSearchIntensity::Strong,
            workspace,
            390.0);
    const LocalSearchResult secondEightResult =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            repeatedEightNeighborhoodSearch,
            instance,
            secondEightEngine,
            LocalSearchIntensity::Strong);

    assert(firstEightResult.acceptedMoves == 3);
    assert(firstEightResult.reachedLocalOptimum);
    assert(firstEightResult.distanceCallsUsed
           < 260U * static_cast<std::uint64_t>(
               instance.actualProblemSize));
    int operatorCalls = 0;
    int operatorAccepts = 0;
    int operatorGammaCrosses = 0;
    std::uint64_t operatorDistanceCalls = 0;
    double operatorUpperGain = 0.0;
    for (const auto& operatorStats : firstEightResult.operatorStats) {
        assert(operatorStats.calls >= operatorStats.accepts);
        assert(operatorStats.accepts >= operatorStats.gammaCrosses);
        assert(operatorStats.upperGain >= 0.0);
        operatorCalls += operatorStats.calls;
        operatorAccepts += operatorStats.accepts;
        operatorDistanceCalls += operatorStats.distanceCalls;
        operatorUpperGain += operatorStats.upperGain;
        operatorGammaCrosses += operatorStats.gammaCrosses;
    }
    assert(operatorCalls == firstEightResult.neighborhoodCalls);
    assert(operatorAccepts == firstEightResult.acceptedMoves);
    assert(operatorDistanceCalls == firstEightResult.distanceCallsUsed);
    assert(std::fabs(
        operatorUpperGain
        - (sevenNeighborhoodCost
           - sevenNeighborhoodLocalOptimum.get_upper_cost())) <= 1e-8);
    assert(operatorGammaCrosses == 1);
    assert(firstEightResult.acceptedMoves == secondEightResult.acceptedMoves);
    assert(firstEightResult.neighborhoodCalls
           == secondEightResult.neighborhoodCalls);
    assert(firstEightResult.distanceCallsUsed
           == secondEightResult.distanceCallsUsed);
    assert(sevenNeighborhoodLocalOptimum.get_upper_cost()
           < sevenNeighborhoodCost - 1e-8);
    assert(std::fabs(
        sevenNeighborhoodLocalOptimum.get_upper_cost()
        - 375.279787148012) <= 1e-8);
    assert(std::fabs(
        sevenNeighborhoodLocalOptimum.get_upper_cost()
        - repeatedEightNeighborhoodSearch.get_upper_cost()) <= 1e-8);
    assert(sevenNeighborhoodLocalOptimum.get_routes()
           == repeatedEightNeighborhoodSearch.get_routes());
    assert_individual_is_consistent(
        sevenNeighborhoodLocalOptimum,
        instance);

    std::mt19937 exhaustedEightEngine(37);
    const LocalSearchResult exhaustedEightResult =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            sevenNeighborhoodLocalOptimum,
            instance,
            exhaustedEightEngine,
            LocalSearchIntensity::Strong,
            workspace);
    assert(exhaustedEightResult.acceptedMoves == 0);
    assert(exhaustedEightResult.reachedLocalOptimum);
    assert_individual_is_consistent(
        sevenNeighborhoodLocalOptimum,
        instance);
}

void assert_progressive_eight_neighborhood_session_matches_strong() {
    const std::string instancePath =
        std::string(TEST_DATA_DIRECTORY) + "/E-n22-k4.evrp";
    Case instance(instancePath, 41);
    std::mt19937 initializationEngine(19);
    const auto routes = Initializer::build_with_clustering(
        instance,
        initializationEngine);
    Individual direct(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        routes,
        instance.fitness_evaluation(routes),
        instance.compute_demand_sum(routes));
    Individual progressive(direct);
    Individual distanceLimited(direct);

    std::mt19937 directEngine(29);
    std::mt19937 progressiveEngine(29);
    std::mt19937 distanceLimitedEngine(29);
    LocalSearchWorkspace directWorkspace;
    LocalSearchWorkspace progressiveWorkspace;
    LocalSearchWorkspace distanceLimitedWorkspace;
    LocalSearchSession distanceLimitedSession;
    Leader::begin_eight_neighborhood_rvnd_one_move_session(
        distanceLimited,
        distanceLimitedSession,
        distanceLimitedWorkspace);
    const LocalSearchResult distanceLimitedResult =
        Leader::continue_eight_neighborhood_rvnd_one_move_session(
            distanceLimited,
            instance,
            distanceLimitedEngine,
            distanceLimitedSession,
            -1,
            distanceLimitedWorkspace,
            std::numeric_limits<double>::infinity(),
            0);
    assert(!distanceLimitedResult.reachedLocalOptimum);
    assert(!distanceLimitedResult.hitMoveLimit);
    assert(distanceLimitedResult.hitDistanceCallLimit);
    assert(distanceLimitedResult.neighborhoodCalls == 0);
    assert(distanceLimitedResult.distanceCallsUsed == 0);

    const LocalSearchResult directResult =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            direct,
            instance,
            directEngine,
            LocalSearchIntensity::Strong,
            directWorkspace);

    LocalSearchSession session;
    Leader::begin_eight_neighborhood_rvnd_one_move_session(
        progressive,
        session,
        progressiveWorkspace);
    const int weakLimit = Leader::move_limit_for_intensity(
        progressive,
        instance,
        LocalSearchIntensity::Weak);
    const LocalSearchResult weakResult =
        Leader::continue_eight_neighborhood_rvnd_one_move_session(
            progressive,
            instance,
            progressiveEngine,
            session,
            weakLimit,
            progressiveWorkspace,
            std::numeric_limits<double>::infinity());
    const int mediumLimit = Leader::move_limit_for_intensity(
        progressive,
        instance,
        LocalSearchIntensity::Medium);
    const LocalSearchResult mediumResult =
        Leader::continue_eight_neighborhood_rvnd_one_move_session(
            progressive,
            instance,
            progressiveEngine,
            session,
            mediumLimit,
            progressiveWorkspace,
            std::numeric_limits<double>::infinity());
    const LocalSearchResult strongResult =
        Leader::continue_eight_neighborhood_rvnd_one_move_session(
            progressive,
            instance,
            progressiveEngine,
            session,
            -1,
            progressiveWorkspace,
            std::numeric_limits<double>::infinity());

    assert(strongResult.reachedLocalOptimum);
    assert(progressive.get_routes() == direct.get_routes());
    assert(std::fabs(
        progressive.get_upper_cost()
        - direct.get_upper_cost()) <= 1e-8);
    assert(
        weakResult.acceptedMoves
            + mediumResult.acceptedMoves
            + strongResult.acceptedMoves
        == directResult.acceptedMoves);
    assert(
        weakResult.neighborhoodCalls
            + mediumResult.neighborhoodCalls
            + strongResult.neighborhoodCalls
        == directResult.neighborhoodCalls);
    assert(
        weakResult.distanceCallsUsed
            + mediumResult.distanceCallsUsed
            + strongResult.distanceCallsUsed
        == directResult.distanceCallsUsed);
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string instanceName = argc > 1 ? argv[1] : "E-n22-k4.evrp";
    const std::string instancePath = std::string(TEST_DATA_DIRECTORY) + "/" + instanceName;
    Case instance(instancePath, 1);
    std::mt19937 randomEngine(1);
    const std::uint64_t callsBeforeDistanceLookup =
        instance.get_distance_calls();
    const double evalsBeforeDistanceLookup = instance.get_evals();
    const double depotSelfDistance = instance.get_distance(
        instance.depot,
        instance.depot);
    assert(std::fabs(depotSelfDistance) <= 1e-12);
    assert(instance.get_distance_calls()
           == callsBeforeDistanceLookup + 1);
    assert(std::fabs(
        instance.get_evals() - evalsBeforeDistanceLookup
        - 1.0 / static_cast<double>(instance.actualProblemSize))
        <= 1e-12);
    const std::uint64_t callsBeforeFullEvaluation =
        instance.get_distance_calls();
    const std::vector<std::vector<int>> depotOnlyRoutes = {
        {instance.depot, instance.depot},
    };
    instance.fitness_evaluation(depotOnlyRoutes);
    assert(instance.get_distance_calls() - callsBeforeFullEvaluation
           == static_cast<std::uint64_t>(
               instance.actualProblemSize));
    assert(instance.get_evaluation_limit_distance_calls()
           == instance.maxEvals
               * static_cast<std::uint64_t>(
                   instance.actualProblemSize));
    assert_inter_route_relocate_removes_empty_route(instance);
    assert_refinement_handles_boundary_cases();
    assert_swap_star_improves_a_seven_neighborhood_local_optimum();
    assert_progressive_eight_neighborhood_session_matches_strong();

    auto clusteredRoutes = Initializer::build_with_clustering(instance, randomEngine);
    auto splitRoutes = Initializer::build_with_random_split(instance, randomEngine);
    auto directRoutes = Initializer::build_with_direct_encoding(instance, randomEngine);
    assert_routes_cover_customers(clusteredRoutes, instance);
    assert_routes_cover_customers(splitRoutes, instance);
    assert_routes_cover_customers(directRoutes, instance);

    SplitWorkspace splitWorkspace;
    std::mt19937 firstSplitEngine(23);
    std::mt19937 secondSplitEngine(23);
    const auto independentlySplitRoutes = Initializer::build_with_random_split(
        instance,
        firstSplitEngine);
    const auto workspaceSplitRoutes = Initializer::build_with_random_split(
        instance,
        secondSplitEngine,
        splitWorkspace);
    assert(independentlySplitRoutes == workspaceSplitRoutes);
    const int* predecessorBuffer = splitWorkspace.predecessors.data();
    const double* costBuffer = splitWorkspace.costs.data();
    const int* giantTourBuffer = splitWorkspace.giantTour.data();
    Initializer::build_with_random_split(
        instance,
        secondSplitEngine,
        splitWorkspace);
    assert(splitWorkspace.predecessors.data() == predecessorBuffer);
    assert(splitWorkspace.costs.data() == costBuffer);
    assert(splitWorkspace.giantTour.data() == giantTourBuffer);

    Individual individual(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        clusteredRoutes,
        instance.fitness_evaluation(clusteredRoutes),
        instance.compute_demand_sum(clusteredRoutes));
    const double upperCostBeforeSearch = individual.get_upper_cost();
    Individual vndBaseline(individual);
    Leader::improve_with_three_neighborhood_vnd(vndBaseline, instance);
    assert(vndBaseline.get_upper_cost() <= upperCostBeforeSearch + 1e-8);

    Individual repeatedRvnd(individual);
    std::mt19937 firstRvndEngine(7);
    std::mt19937 secondRvndEngine(7);
    Leader::improve_with_three_neighborhood_rvnd(individual, instance, firstRvndEngine);
    Leader::improve_with_three_neighborhood_rvnd(repeatedRvnd, instance, secondRvndEngine);
    assert(individual.get_upper_cost() <= upperCostBeforeSearch + 1e-8);
    assert(std::fabs(individual.get_upper_cost() - repeatedRvnd.get_upper_cost()) <= 1e-8);
    assert(individual.get_routes() == repeatedRvnd.get_routes());
    assert_individual_is_consistent(individual, instance);

    Individual skippedSevenNeighborhoodSearch(vndBaseline);
    const auto routesBeforeSkip = skippedSevenNeighborhoodSearch.get_routes();
    const double upperCostBeforeSkip =
        skippedSevenNeighborhoodSearch.get_upper_cost();
    const bool locallyOptimalBeforeSkip =
        skippedSevenNeighborhoodSearch.is_upper_locally_optimal();
    const std::uint64_t callsBeforeSkip =
        instance.get_distance_calls();
    std::mt19937 skipSearchEngine(12);
    const LocalSearchResult skipSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            skippedSevenNeighborhoodSearch,
            instance,
            skipSearchEngine,
            LocalSearchIntensity::Skip);
    assert(skipSearchResult.moveLimit == 0);
    assert(skipSearchResult.acceptedMoves == 0);
    assert(skipSearchResult.neighborhoodCalls == 0);
    assert(skipSearchResult.distanceCallsUsed == 0);
    assert(std::fabs(skipSearchResult.relativeUpperImprovement) <= 1e-12);
    assert(!skipSearchResult.reachedLocalOptimum);
    assert(skippedSevenNeighborhoodSearch.is_upper_locally_optimal()
           == locallyOptimalBeforeSkip);
    assert(instance.get_distance_calls() == callsBeforeSkip);
    assert(std::fabs(
        skippedSevenNeighborhoodSearch.get_upper_cost()
        - upperCostBeforeSkip) <= 1e-12);
    assert(skippedSevenNeighborhoodSearch.get_routes() == routesBeforeSkip);

    Individual fiveNeighborhoodRvnd(vndBaseline);
    Individual repeatedFiveNeighborhoodRvnd(vndBaseline);
    const double upperCostBeforeFiveNeighborhoodSearch = fiveNeighborhoodRvnd.get_upper_cost();
    std::mt19937 firstFiveNeighborhoodEngine(11);
    std::mt19937 secondFiveNeighborhoodEngine(11);
    Leader::improve_with_five_neighborhood_rvnd(
        fiveNeighborhoodRvnd,
        instance,
        firstFiveNeighborhoodEngine);
    Leader::improve_with_five_neighborhood_rvnd(
        repeatedFiveNeighborhoodRvnd,
        instance,
        secondFiveNeighborhoodEngine);
    assert(fiveNeighborhoodRvnd.get_upper_cost()
           <= upperCostBeforeFiveNeighborhoodSearch + 1e-8);
    assert(std::fabs(
        fiveNeighborhoodRvnd.get_upper_cost()
        - repeatedFiveNeighborhoodRvnd.get_upper_cost()) <= 1e-8);
    assert(fiveNeighborhoodRvnd.get_routes()
           == repeatedFiveNeighborhoodRvnd.get_routes());
    assert_individual_is_consistent(fiveNeighborhoodRvnd, instance);

    Individual weakSevenNeighborhoodSearch(vndBaseline);
    const int weakSolutionScale =
        instance.customerNumber + weakSevenNeighborhoodSearch.route_num;
    std::mt19937 weakSearchEngine(12);
    const LocalSearchResult weakSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            weakSevenNeighborhoodSearch,
            instance,
            weakSearchEngine,
            LocalSearchIntensity::Weak);
    assert(weakSearchResult.moveLimit
           == std::max(1, static_cast<int>(std::ceil(0.02 * weakSolutionScale))));
    assert(weakSearchResult.acceptedMoves <= weakSearchResult.moveLimit);
    assert(weakSearchResult.neighborhoodCalls >= weakSearchResult.acceptedMoves);
    if (weakSearchResult.acceptedMoves < weakSearchResult.moveLimit) {
        assert(weakSearchResult.reachedLocalOptimum);
    }
    assert_individual_is_consistent(weakSevenNeighborhoodSearch, instance);

    Individual mediumSevenNeighborhoodSearch(vndBaseline);
    const int mediumSolutionScale =
        instance.customerNumber + mediumSevenNeighborhoodSearch.route_num;
    std::mt19937 mediumSearchEngine(12);
    const LocalSearchResult mediumSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            mediumSevenNeighborhoodSearch,
            instance,
            mediumSearchEngine,
            LocalSearchIntensity::Medium);
    assert(mediumSearchResult.moveLimit
           == std::max(1, static_cast<int>(std::ceil(0.10 * mediumSolutionScale))));
    assert(mediumSearchResult.acceptedMoves <= mediumSearchResult.moveLimit);
    assert(mediumSearchResult.neighborhoodCalls >= mediumSearchResult.acceptedMoves);
    if (mediumSearchResult.acceptedMoves < mediumSearchResult.moveLimit) {
        assert(mediumSearchResult.reachedLocalOptimum);
    }
    assert_individual_is_consistent(mediumSevenNeighborhoodSearch, instance);

    Individual boundedEightNeighborhoodSearch(vndBaseline);
    const int boundedSolutionScale =
        instance.customerNumber
        + boundedEightNeighborhoodSearch.route_num;
    std::mt19937 boundedSearchEngine(12);
    LocalSearchWorkspace boundedWorkspace;
    const LocalSearchResult boundedSearchResult =
        Leader::improve_with_eight_neighborhood_rvnd_one_move(
            boundedEightNeighborhoodSearch,
            instance,
            boundedSearchEngine,
            LocalSearchIntensity::BoundedStrong,
            boundedWorkspace);
    assert(
        boundedSearchResult.moveLimit
        == std::max(
            1,
            static_cast<int>(
                std::ceil(0.30 * boundedSolutionScale))));
    assert(
        boundedSearchResult.acceptedMoves
        <= boundedSearchResult.moveLimit);
    assert(
        static_cast<int>(boundedSearchResult.reachedLocalOptimum)
        + static_cast<int>(boundedSearchResult.hitMoveLimit)
        + static_cast<int>(boundedSearchResult.hitDistanceCallLimit)
        == 1);
    assert(
        Leader::bounded_strong_distance_call_limit(0) == 128);
    assert(
        Leader::bounded_strong_distance_call_limit(7)
        == 896);
    assert_individual_is_consistent(
        boundedEightNeighborhoodSearch,
        instance);

    Individual sevenNeighborhoodRvnd(fiveNeighborhoodRvnd);
    Individual repeatedSevenNeighborhoodRvnd(fiveNeighborhoodRvnd);
    const double upperCostBeforeSevenNeighborhoodSearch =
        sevenNeighborhoodRvnd.get_upper_cost();
    const int routesBeforeSevenNeighborhoodSearch = sevenNeighborhoodRvnd.route_num;
    std::mt19937 firstSevenNeighborhoodEngine(13);
    std::mt19937 secondSevenNeighborhoodEngine(13);
    Leader::improve_with_seven_neighborhood_rvnd(
        sevenNeighborhoodRvnd,
        instance,
        firstSevenNeighborhoodEngine);
    Leader::improve_with_seven_neighborhood_rvnd(
        repeatedSevenNeighborhoodRvnd,
        instance,
        secondSevenNeighborhoodEngine);
    assert(sevenNeighborhoodRvnd.get_upper_cost()
           <= upperCostBeforeSevenNeighborhoodSearch + 1e-8);
    assert(sevenNeighborhoodRvnd.route_num <= routesBeforeSevenNeighborhoodSearch);
    assert(std::fabs(
        sevenNeighborhoodRvnd.get_upper_cost()
        - repeatedSevenNeighborhoodRvnd.get_upper_cost()) <= 1e-8);
    assert(sevenNeighborhoodRvnd.get_routes()
           == repeatedSevenNeighborhoodRvnd.get_routes());
    assert_individual_is_consistent(sevenNeighborhoodRvnd, instance);

    Individual sevenNeighborhoodOneMove(fiveNeighborhoodRvnd);
    Individual repeatedSevenNeighborhoodOneMove(fiveNeighborhoodRvnd);
    const double upperCostBeforeOneMoveSearch =
        sevenNeighborhoodOneMove.get_upper_cost();
    const int routesBeforeOneMoveSearch = sevenNeighborhoodOneMove.route_num;
    std::mt19937 firstOneMoveEngine(17);
    std::mt19937 secondOneMoveEngine(17);
    LocalSearchWorkspace reusedWorkspace;
    reusedWorkspace.routeOrder.assign(32, -1);
    reusedWorkspace.activeRoutePairPools[
        static_cast<std::size_t>(
            LocalSearchOperator::InterRouteSwap)].pairs.assign(
                64,
                {-1, -1});
    const LocalSearchResult strongSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            sevenNeighborhoodOneMove,
            instance,
            firstOneMoveEngine,
            LocalSearchIntensity::Strong,
            reusedWorkspace);
    const LocalSearchResult repeatedStrongSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            repeatedSevenNeighborhoodOneMove,
            instance,
            secondOneMoveEngine,
            LocalSearchIntensity::Strong);
    assert(strongSearchResult.moveLimit == -1);
    assert(strongSearchResult.reachedLocalOptimum);
    assert(strongSearchResult.neighborhoodCalls >= strongSearchResult.acceptedMoves);
    assert(strongSearchResult.acceptedMoves
           == repeatedStrongSearchResult.acceptedMoves);
    assert(strongSearchResult.neighborhoodCalls
           == repeatedStrongSearchResult.neighborhoodCalls);
    assert(strongSearchResult.distanceCallsUsed
           == repeatedStrongSearchResult.distanceCallsUsed);
    assert(sevenNeighborhoodOneMove.get_upper_cost()
           <= upperCostBeforeOneMoveSearch + 1e-8);
    assert(sevenNeighborhoodOneMove.route_num <= routesBeforeOneMoveSearch);
    assert(std::fabs(
        sevenNeighborhoodOneMove.get_upper_cost()
        - repeatedSevenNeighborhoodOneMove.get_upper_cost()) <= 1e-8);
    assert(sevenNeighborhoodOneMove.get_routes()
           == repeatedSevenNeighborhoodOneMove.get_routes());
    assert_individual_is_consistent(sevenNeighborhoodOneMove, instance);

    Individual exhaustedOneMoveSearch(sevenNeighborhoodOneMove);
    std::mt19937 exhaustedOneMoveEngine(19);
    const LocalSearchResult exhaustedSearchResult =
        Leader::improve_with_seven_neighborhood_rvnd_one_move(
            exhaustedOneMoveSearch,
            instance,
            exhaustedOneMoveEngine,
            LocalSearchIntensity::Strong);
    assert(exhaustedSearchResult.acceptedMoves == 0);
    assert(exhaustedSearchResult.reachedLocalOptimum);
    assert(std::fabs(
        exhaustedOneMoveSearch.get_upper_cost()
        - sevenNeighborhoodOneMove.get_upper_cost()) <= 1e-8);
    assert(exhaustedOneMoveSearch.get_routes()
           == sevenNeighborhoodOneMove.get_routes());
    assert_individual_is_consistent(exhaustedOneMoveSearch, instance);

    Individual repeatedChargingEvaluation(sevenNeighborhoodOneMove);
    FollowerWorkspace followerWorkspace;
    const std::uint64_t callsBeforeFirstChargingEvaluation =
        instance.get_distance_calls();
    Follower::optimize_charging(
        sevenNeighborhoodOneMove,
        instance,
        followerWorkspace);
    const std::uint64_t firstChargingDistanceCalls =
        instance.get_distance_calls()
        - callsBeforeFirstChargingEvaluation;
    assert(std::isfinite(sevenNeighborhoodOneMove.get_lower_cost()));
    assert(sevenNeighborhoodOneMove.get_tour().second == 0);
    const double* cumulativeDistanceBuffer =
        followerWorkspace.cumulativeDistance.data();
    const std::uint64_t callsBeforeSecondChargingEvaluation =
        instance.get_distance_calls();
    Follower::optimize_charging(
        repeatedChargingEvaluation,
        instance,
        followerWorkspace);
    const std::uint64_t secondChargingDistanceCalls =
        instance.get_distance_calls()
        - callsBeforeSecondChargingEvaluation;
    assert(std::fabs(
        repeatedChargingEvaluation.get_lower_cost()
        - sevenNeighborhoodOneMove.get_lower_cost()) <= 1e-8);
    assert(repeatedChargingEvaluation.get_tour().second == 0);
    assert(firstChargingDistanceCalls == secondChargingDistanceCalls);
    assert(followerWorkspace.cumulativeDistance.data()
           == cumulativeDistanceBuffer);
    if (instance.customerNumber <= 30) {
        Follower::refine_charging_by_enumeration(sevenNeighborhoodOneMove, instance);
        assert(std::isfinite(sevenNeighborhoodOneMove.get_lower_cost()));
    }

    std::vector<std::shared_ptr<Individual>> rankedSolutions;
    rankedSolutions.push_back(std::make_shared<Individual>(sevenNeighborhoodOneMove));
    rankedSolutions.push_back(std::make_shared<Individual>(
        instance.vehicleNumber * 3,
        instance.customerNumber + 2,
        splitRoutes,
        instance.fitness_evaluation(splitRoutes),
        instance.compute_demand_sum(splitRoutes)));
    std::sort(
        rankedSolutions.begin(),
        rankedSolutions.end(),
        [](const auto& first, const auto& second) {
            return first->get_upper_cost() < second->get_upper_cost();
        });
    const auto parentPool = Reproduction::build_quality_diversity_parent_pool(
        rankedSolutions,
        2,
        0.5);
    assert(!parentPool.empty());
    if (parentPool.size() == 2) {
        const double distance = Reproduction::adjacency_distance(parentPool[0], parentPool[1]);
        assert(distance >= 0.0 && distance <= 1.0);
    }

    auto firstChild = Reproduction::make_random_immigrant(instance.customers, randomEngine);
    auto secondChild = Reproduction::make_random_immigrant(instance.customers, randomEngine);
    Reproduction::partially_matched_crossover(firstChild, secondChild, randomEngine);
    Reproduction::mutate_by_index_shuffle(firstChild, 0.2, randomEngine);
    assert_is_customer_permutation(firstChild, instance);
    assert_is_customer_permutation(secondChild, instance);

    std::vector<int> firstParent = instance.customers;
    std::vector<int> secondParent(instance.customers.rbegin(), instance.customers.rend());
    std::vector<int> workspaceFirstParent = firstParent;
    std::vector<int> workspaceSecondParent = secondParent;
    std::mt19937 firstCrossoverEngine(31);
    std::mt19937 secondCrossoverEngine(31);
    Reproduction::partially_matched_crossover(
        firstParent,
        secondParent,
        firstCrossoverEngine);
    ReproductionWorkspace reproductionWorkspace;
    Reproduction::partially_matched_crossover(
        workspaceFirstParent,
        workspaceSecondParent,
        secondCrossoverEngine,
        reproductionWorkspace);
    assert(firstParent == workspaceFirstParent);
    assert(secondParent == workspaceSecondParent);
    const int* firstMappingBuffer = reproductionWorkspace.firstMapping.data();
    workspaceFirstParent.assign(instance.customers.begin(), instance.customers.end());
    workspaceSecondParent.assign(instance.customers.rbegin(), instance.customers.rend());
    Reproduction::partially_matched_crossover(
        workspaceFirstParent,
        workspaceSecondParent,
        secondCrossoverEngine,
        reproductionWorkspace);
    assert(reproductionWorkspace.firstMapping.data() == firstMappingBuffer);

    std::mt19937 firstOffspringEngine(37);
    std::mt19937 secondOffspringEngine(37);
    std::uniform_real_distribution<double> firstProbabilityDistribution(0.0, 1.0);
    std::uniform_real_distribution<double> secondProbabilityDistribution(0.0, 1.0);
    const auto independentlyCreatedOffspring = Reproduction::create_offspring(
        parentPool,
        rankedSolutions.front().get(),
        true,
        instance.customers,
        10,
        std::min<int>(2, parentPool.size()),
        0.5,
        0.1,
        0.2,
        0.2,
        firstOffspringEngine,
        firstProbabilityDistribution);
    std::vector<int> parentUseCounts;
    const auto workspaceCreatedOffspring = Reproduction::create_offspring(
        parentPool,
        rankedSolutions.front().get(),
        true,
        instance.customers,
        10,
        std::min<int>(2, parentPool.size()),
        0.5,
        0.1,
        0.2,
        0.2,
        secondOffspringEngine,
        secondProbabilityDistribution,
        reproductionWorkspace,
        &parentUseCounts);
    assert(independentlyCreatedOffspring == workspaceCreatedOffspring);
    assert(parentUseCounts.size() == parentPool.size());
    assert(std::accumulate(
        parentUseCounts.begin(),
        parentUseCounts.end(),
        0) > 0);

    OnlineIntensityLearner archiveAllocator;
    archiveAllocator.reset();
    auto archiveSolution =
        std::make_shared<Individual>(sevenNeighborhoodOneMove);
    const auto firstArchiveCredits =
        archiveAllocator.update_lower_archive({archiveSolution});
    assert(firstArchiveCredits.size() == 1);
    assert(firstArchiveCredits.front().first
           == archiveSolution.get());
    assert(std::fabs(
        firstArchiveCredits.front().second - 1.0) <= 1e-12);
    const auto repeatedArchiveCredits =
        archiveAllocator.update_lower_archive({archiveSolution});
    assert(repeatedArchiveCredits.empty());

    LocalSearchAllocationRun rewardRun;
    AllocatedLocalSearchRecord rewardRecord;
    rewardRecord.terminalIntensity =
        LocalSearchIntensity::Medium;
    rewardRecord.costAfterWeak = 100.0;
    rewardRecord.costAfterTerminal = 99.0;
    rewardRecord.parentCredit = 1.0;
    rewardRecord.lowerCredit = 1.0;
    rewardRecord.postProbeGammaCross = true;
    rewardRecord.weakResult.distanceCallsUsed = 100;
    rewardRecord.continuationResult.distanceCallsUsed = 300;
    auto& firstOperatorReward =
        rewardRecord.continuationResult.operatorStats[
            static_cast<std::size_t>(
                LocalSearchOperator::NodeShift)];
    firstOperatorReward.calls = 2;
    firstOperatorReward.accepts = 1;
    firstOperatorReward.distanceCalls = 50;
    firstOperatorReward.upperGain = 0.75;
    firstOperatorReward.gammaCrosses = 1;
    auto& secondOperatorReward =
        rewardRecord.continuationResult.operatorStats[
            static_cast<std::size_t>(
                LocalSearchOperator::InterRouteRelocate)];
    secondOperatorReward.calls = 1;
    secondOperatorReward.accepts = 1;
    secondOperatorReward.distanceCalls = 250;
    secondOperatorReward.upperGain = 0.25;
    rewardRun.records.push_back(std::move(rewardRecord));
    OnlineIntensityLearner unusedRewardLearner;
    LocalSearchAllocationRunner::finalize_feedback(
        rewardRun,
        LocalSearchPolicy::MatchedRandom,
        unusedRewardLearner);
    const auto& finalizedReward = rewardRun.records.front();
    assert(std::fabs(finalizedReward.reward - 0.90) <= 1e-12);
    assert(std::fabs(finalizedReward.parentReward - 0.20) <= 1e-12);
    assert(std::fabs(finalizedReward.lowerReward - 0.45) <= 1e-12);
    assert(std::fabs(finalizedReward.gammaReward - 0.25) <= 1e-12);
    assert(std::fabs(
        finalizedReward.incrementalCostUnits - 3.0) <= 1e-12);
    const auto& mediumRewardStats = rewardRun.stats[
        LocalSearchAllocationRunner::intensity_index(
            LocalSearchIntensity::Medium)];
    assert(std::fabs(
        mediumRewardStats.parentReward - 0.20) <= 1e-12);
    assert(std::fabs(
        mediumRewardStats.lowerReward - 0.45) <= 1e-12);
    assert(std::fabs(
        mediumRewardStats.gammaReward - 0.25) <= 1e-12);
    assert(std::fabs(
        mediumRewardStats.continuationGainSignal - 0.05)
        <= 1e-12);

    BudgetAwareOperatorScheduler operatorScheduler;
    operatorScheduler.reset();
    const double initialOperatorProbability =
        1.0 / static_cast<double>(LOCAL_SEARCH_OPERATOR_COUNT);
    assert(std::fabs(
        operatorScheduler.selection_probability(
            LocalSearchOperator::NodeShift)
        - initialOperatorProbability) <= 1e-12);
    const auto operatorLearningStats =
        operatorScheduler.update(rewardRun);
    const auto& firstLearnedOperator = operatorLearningStats[
        static_cast<std::size_t>(
            LocalSearchOperator::NodeShift)];
    const auto& secondLearnedOperator = operatorLearningStats[
        static_cast<std::size_t>(
            LocalSearchOperator::InterRouteRelocate)];
    assert(firstLearnedOperator.operatorStats.calls == 2);
    assert(secondLearnedOperator.operatorStats.calls == 1);
    assert(std::fabs(
        firstLearnedOperator.relativeUpperGain - 0.0075)
        <= 1e-12);
    assert(std::fabs(
        secondLearnedOperator.relativeUpperGain - 0.0025)
        <= 1e-12);
    assert(std::fabs(
        operatorScheduler.estimated_cost_per_call(
            LocalSearchOperator::NodeShift) - 25.0)
        <= 1e-12);
    assert(std::fabs(
        operatorScheduler.estimated_cost_per_call(
            LocalSearchOperator::InterRouteRelocate) - 250.0)
        <= 1e-12);
    assert(std::fabs(
        operatorScheduler.efficiency(
            LocalSearchOperator::NodeShift) - 0.00015)
        <= 1e-12);
    assert(std::fabs(
        operatorScheduler.efficiency(
            LocalSearchOperator::InterRouteRelocate) - 0.00001)
        <= 1e-12);
    assert(
        operatorScheduler.target_budget_share(
            LocalSearchOperator::NodeShift)
        > operatorScheduler.target_budget_share(
            LocalSearchOperator::InterRouteRelocate));
    assert(
        operatorScheduler.selection_probability(
            LocalSearchOperator::NodeShift)
        > operatorScheduler.selection_probability(
            LocalSearchOperator::InterRouteRelocate));
    assert(std::fabs(
        operatorScheduler.effective_observations(
            LocalSearchOperator::NodeShift)
        - 1.0) <= 1e-12);
    assert(std::fabs(
        operatorScheduler.effective_observations(
            LocalSearchOperator::InterRouteRelocate)
        - 1.0) <= 1e-12);

    double operatorProbabilitySum = 0.0;
    double targetBudgetShareSum = 0.0;
    double impliedBudgetTotal = 0.0;
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        const auto localSearchOperator =
            static_cast<LocalSearchOperator>(operatorIndex);
        operatorProbabilitySum +=
            operatorScheduler.selection_probability(
                localSearchOperator);
        targetBudgetShareSum +=
            operatorScheduler.target_budget_share(
                localSearchOperator);
        impliedBudgetTotal +=
            operatorScheduler.selection_probability(
                localSearchOperator)
            * operatorScheduler.estimated_cost_per_call(
                localSearchOperator);
    }
    assert(std::fabs(operatorProbabilitySum - 1.0) <= 1e-12);
    assert(std::fabs(targetBudgetShareSum - 1.0) <= 1e-12);
    for (std::size_t operatorIndex = 0;
         operatorIndex < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operatorIndex) {
        const auto localSearchOperator =
            static_cast<LocalSearchOperator>(operatorIndex);
        const double impliedBudgetShare =
            operatorScheduler.selection_probability(
                localSearchOperator)
            * operatorScheduler.estimated_cost_per_call(
                localSearchOperator)
            / impliedBudgetTotal;
        assert(std::fabs(
            impliedBudgetShare
            - operatorScheduler.target_budget_share(
                localSearchOperator)) <= 1e-12);
    }

    LocalSearchAllocationRun failedOperatorRun;
    AllocatedLocalSearchRecord failedOperatorRecord;
    failedOperatorRecord.costAfterWeak = 100.0;
    auto& failedOperatorStats =
        failedOperatorRecord.continuationResult.operatorStats[
            static_cast<std::size_t>(
                LocalSearchOperator::NodeShift)];
    failedOperatorStats.calls = 4;
    failedOperatorStats.distanceCalls = 100;
    failedOperatorRun.records.push_back(
        std::move(failedOperatorRecord));
    const double efficiencyBeforeFailure =
        operatorScheduler.efficiency(
            LocalSearchOperator::NodeShift);
    const auto failedOperatorLearningStats =
        operatorScheduler.update(failedOperatorRun);
    assert(
        failedOperatorLearningStats[
            static_cast<std::size_t>(
                LocalSearchOperator::NodeShift)]
            .operatorStats.calls == 4);
    assert(
        operatorScheduler.efficiency(
            LocalSearchOperator::NodeShift)
        < efficiencyBeforeFailure);
    const auto uniformOperatorStats =
        operatorScheduler.update(rewardRun, false);
    for (const auto& stats : uniformOperatorStats) {
        assert(std::fabs(
            stats.targetBudgetShare
            - initialOperatorProbability) <= 1e-12);
        assert(std::fabs(
            stats.selectionProbability
            - initialOperatorProbability) <= 1e-12);
    }

    BudgetAwareOperatorScheduler::SelectionWeights
        dominantOperatorWeights{};
    dominantOperatorWeights[
        static_cast<std::size_t>(
            LocalSearchOperator::NodeShift)] = 1.0;
    const LocalSearchOperatorSelectionTable selectionTable =
        Leader::build_operator_selection_table(
            dominantOperatorWeights,
            0.0);
    const auto& fullSelectionEntry =
        selectionTable.entries[
            LOCAL_SEARCH_ALL_OPERATOR_MASK];
    assert(
        fullSelectionEntry.activeCount
        == LOCAL_SEARCH_OPERATOR_COUNT);
    assert(std::fabs(
        fullSelectionEntry.cumulativeProbabilities.front()
        - 1.0) <= 1e-12);
    const std::size_t firstTwoOperatorMask =
        (1U << static_cast<std::size_t>(
            LocalSearchOperator::NodeShift))
        | (1U << static_cast<std::size_t>(
            LocalSearchOperator::InterRouteRelocate));
    const auto& twoOperatorSelectionEntry =
        selectionTable.entries[firstTwoOperatorMask];
    assert(twoOperatorSelectionEntry.activeCount == 2);
    assert(std::fabs(
        twoOperatorSelectionEntry
            .cumulativeProbabilities[0]
        - 1.0) <= 1e-12);
    assert(std::fabs(
        twoOperatorSelectionEntry
            .cumulativeProbabilities[1]
        - 1.0) <= 1e-12);
    assert(
        std::string(operator_selection_policy_name(
            OperatorSelectionPolicy::BudgetAware))
        == "budget_aware");

    LocalSearchAllocationContext syntheticContext;
    syntheticContext.qualityGap = 0.01;
    syntheticContext.adjacencyDistance = 0.5;
    syntheticContext.budgetProgress = 0.25;
    syntheticContext.gammaMargin = 0.01;
    syntheticContext.probeSuccessRate = 0.5;
    syntheticContext.probeRelativeGain = 0.01;
    syntheticContext.probeEfficiency = 0.5;
    for (int observation = 0; observation < 20; ++observation) {
        archiveAllocator.update(
            syntheticContext,
            LocalSearchIntensity::Weak,
            1.0,
            1.0);
        archiveAllocator.update(
            syntheticContext,
            LocalSearchIntensity::Strong,
            0.0,
            100.0);
    }
    assert(
        archiveAllocator.observation_count(
            LocalSearchIntensity::Weak) == 20);
    assert(
        archiveAllocator.score(
            syntheticContext,
            LocalSearchIntensity::Weak)
        > archiveAllocator.score(
            syntheticContext,
            LocalSearchIntensity::Strong));

    Parameters algorithmParameters;
    algorithmParameters.seed = 1;
    algorithmParameters.stopCriteria = 1;
    algorithmParameters.popSize = 10;
    algorithmParameters.mutationProb = 0.35;
    algorithmParameters.mutationIndProb = 0.07;
    algorithmParameters.enableLogging = true;
    MA algorithm(&instance, algorithmParameters);
    assert(std::fabs(algorithm.mutationProb - 0.35) <= 1e-12);
    assert(std::fabs(algorithm.mutationIndProb - 0.07) <= 1e-12);
    algorithm.initialize_search();
    assert(algorithm.upperBestIndividual != nullptr);
    assert(std::fabs(
        algorithm.upperBestIndividual->get_upper_cost()
        - algorithm.globalBestUpperCost) <= 1e-8);
    const double initialArchivedUpperCost =
        algorithm.upperBestIndividual->get_upper_cost();
    std::set<const Individual*> initialPopulationAddresses;
    std::vector<std::pair<const Individual*, std::set<int*>>> initialRouteBuffers;
    for (const auto& individual : algorithm.population) {
        initialPopulationAddresses.insert(individual.get());
        std::set<int*> routeBuffers;
        for (int routeIndex = 0;
             routeIndex < individual->route_cap;
             ++routeIndex) {
            routeBuffers.insert(individual->routes[routeIndex]);
        }
        initialRouteBuffers.emplace_back(individual.get(), std::move(routeBuffers));
    }
    const Individual* verifiedBestAddress = algorithm.verifiedBest.get();
    algorithm.run_generation();
    assert(algorithm.population.size() == 10);
    std::set<const Individual*> reusedPopulationAddresses;
    for (const auto& individual : algorithm.population) {
        reusedPopulationAddresses.insert(individual.get());
        const auto originalBuffer = std::find_if(
            initialRouteBuffers.begin(),
            initialRouteBuffers.end(),
            [&](const auto& entry) {
                return entry.first == individual.get();
            });
        assert(originalBuffer != initialRouteBuffers.end());
        const std::set<int*> currentRouteBuffers(
            individual->routes,
            individual->routes + individual->route_cap);
        assert(originalBuffer->second == currentRouteBuffers);
    }
    assert(reusedPopulationAddresses == initialPopulationAddresses);
    assert(algorithm.verifiedBest.get() == verifiedBestAddress);
    assert(algorithm.verifiedBest != nullptr);
    assert(std::isfinite(algorithm.verifiedBest->get_lower_cost()));
    assert(algorithm.upperBestIndividual->get_upper_cost()
           <= initialArchivedUpperCost + 1e-8);
    assert(std::fabs(
        algorithm.upperBestIndividual->get_upper_cost()
        - algorithm.globalBestUpperCost) <= 1e-8);
    int finiteLowerCostCount = 0;
    for (const auto& individual : algorithm.population) {
        finiteLowerCostCount += std::isfinite(individual->get_lower_cost());
    }
    assert(finiteLowerCostCount == 1);
    assert(std::isfinite(algorithm.population.front()->get_lower_cost()));
    assert(std::string(MA::EVOLUTION_LOG_HEADER)
           == "iter,evals,best_upper_cost,best_lower_cost,progress,duration");
    std::string localSearchRow;
    assert(std::string(MA::LOCAL_SEARCH_OPERATOR_LOG_HEADER)
           == "iter\tgenerations\toperator\tcalls\taccepts\tevals\t"
              "upper_gain\tgamma_crosses");
    algorithm.write_local_search_operator_snapshot();
    const std::string operatorRows = algorithm.localSearchOperatorRows.str();
    std::istringstream operatorStream(operatorRows);
    std::string operatorRow;
    std::set<std::string> loggedOperators;
    int operatorRowCount = 0;
    int aggregateOperatorCalls = 0;
    int aggregateOperatorAccepts = 0;
    int aggregateOperatorGammaCrosses = 0;
    double aggregateOperatorEvals = 0.0;
    while (std::getline(operatorStream, operatorRow)) {
        assert(std::count(operatorRow.begin(), operatorRow.end(), '\t') == 7);
        std::istringstream rowStream(operatorRow);
        std::vector<std::string> columns;
        std::string column;
        while (std::getline(rowStream, column, '\t')) {
            columns.push_back(column);
        }
        assert(columns.size() == 8);
        assert(std::stoi(columns[0]) == 1);
        assert(std::stoi(columns[1]) == 1);
        assert(loggedOperators.insert(columns[2]).second);
        const int calls = std::stoi(columns[3]);
        const int accepts = std::stoi(columns[4]);
        const double evals = std::stod(columns[5]);
        const double upperGain = std::stod(columns[6]);
        const int gammaCrosses = std::stoi(columns[7]);
        assert(calls >= accepts);
        assert(accepts >= gammaCrosses);
        assert(evals >= 0.0);
        assert(upperGain >= 0.0);
        aggregateOperatorCalls += calls;
        aggregateOperatorAccepts += accepts;
        aggregateOperatorEvals += evals;
        aggregateOperatorGammaCrosses += gammaCrosses;
        ++operatorRowCount;
    }
    assert(operatorRowCount
           == static_cast<int>(LOCAL_SEARCH_OPERATOR_COUNT));
    assert(aggregateOperatorCalls > 0);
    assert(aggregateOperatorAccepts > 0);
    assert(aggregateOperatorEvals > 0.0);
    assert(aggregateOperatorGammaCrosses >= 0);
    algorithm.flush_row_into_evol_log();
    const std::string evolutionRow = algorithm.evolutionRows.str();
    assert(std::count(evolutionRow.begin(), evolutionRow.end(), ',') == 5);

    const double archivedLowerCost = algorithm.verifiedBest->get_lower_cost();
    algorithm.localSearchIntensity = LocalSearchIntensity::Weak;
    algorithm.globalBestUpperCost = 0.0;
    algorithm.run_generation();
    reusedPopulationAddresses.clear();
    for (const auto& individual : algorithm.population) {
        reusedPopulationAddresses.insert(individual.get());
    }
    assert(reusedPopulationAddresses == initialPopulationAddresses);
    assert(algorithm.verifiedBest.get() == verifiedBestAddress);
    assert(std::fabs(
        algorithm.verifiedBest->get_lower_cost() - archivedLowerCost) <= 1e-8);
    finiteLowerCostCount = 0;
    for (const auto& individual : algorithm.population) {
        finiteLowerCostCount += std::isfinite(individual->get_lower_cost());
    }
    assert(finiteLowerCostCount == 1);
    assert(std::fabs(
        algorithm.population.front()->get_lower_cost() - archivedLowerCost) <= 1e-8);

    // Crossing the former 30-generation boundary must not reduce the set of
    // individuals receiving local search.
    Case fullPopulationInstance(instancePath, 2);
    Parameters fullPopulationParameters;
    fullPopulationParameters.seed = 2;
    fullPopulationParameters.popSize = 4;
    fullPopulationParameters.localSearchIntensity = LocalSearchIntensity::Weak;
    fullPopulationParameters.enableLogging = true;
    MA fullPopulationAlgorithm(&fullPopulationInstance, fullPopulationParameters);
    fullPopulationAlgorithm.initialize_search();
    for (int iter = 0; iter < 31; ++iter) {
        fullPopulationAlgorithm.run_generation();
    }
    fullPopulationAlgorithm.write_local_search_operator_snapshot();
    const std::string fullPopulationOperatorRows =
        fullPopulationAlgorithm.localSearchOperatorRows.str();
    assert(std::count(
        fullPopulationOperatorRows.begin(),
        fullPopulationOperatorRows.end(),
        '\n') == static_cast<int>(LOCAL_SEARCH_OPERATOR_COUNT));

    Case gammaOnlyInstance(instancePath, 3);
    Parameters gammaOnlyParameters;
    gammaOnlyParameters.seed = 3;
    gammaOnlyParameters.popSize = 4;
    gammaOnlyParameters.localSearchIntensity = LocalSearchIntensity::Weak;
    gammaOnlyParameters.enableLogging = true;
    MA gammaOnlyAlgorithm(&gammaOnlyInstance, gammaOnlyParameters);
    gammaOnlyAlgorithm.initialize_search();
    gammaOnlyAlgorithm.globalBestUpperCost = 0.0;
    gammaOnlyAlgorithm.run_generation();
    gammaOnlyAlgorithm.write_local_search_operator_snapshot();
    assert(std::isinf(gammaOnlyAlgorithm.verifiedBest->get_lower_cost()));
    for (const auto& individual : gammaOnlyAlgorithm.population) {
        assert(std::isinf(individual->get_lower_cost()));
    }
    const std::string gammaOnlyOperatorRows =
        gammaOnlyAlgorithm.localSearchOperatorRows.str();
    assert(std::count(
        gammaOnlyOperatorRows.begin(),
        gammaOnlyOperatorRows.end(),
        '\n') == static_cast<int>(LOCAL_SEARCH_OPERATOR_COUNT));

    Case skipFollowerInstance(instancePath, 5);
    Parameters skipFollowerParameters;
    skipFollowerParameters.seed = 5;
    skipFollowerParameters.popSize = 4;
    skipFollowerParameters.localSearchIntensity = LocalSearchIntensity::Skip;
    MA skipFollowerAlgorithm(&skipFollowerInstance, skipFollowerParameters);
    skipFollowerAlgorithm.initialize_search();
    skipFollowerAlgorithm.run_generation();
    assert(std::isfinite(skipFollowerAlgorithm.verifiedBest->get_lower_cost()));

    Case finalFallbackInstance(instancePath, 6);
    finalFallbackInstance.maxEvals = 0;
    Parameters finalFallbackParameters;
    finalFallbackParameters.seed = 6;
    finalFallbackParameters.enableLogging = false;
    MA finalFallbackAlgorithm(&finalFallbackInstance, finalFallbackParameters);
    finalFallbackAlgorithm.run();
    assert(finalFallbackAlgorithm.verifiedBest != nullptr);
    assert(finalFallbackAlgorithm.verifiedBest->get_lower_cost()
           < INFEASIBLE_COST);

    Case retainedEliteInstance(instancePath, 4);
    Parameters retainedEliteParameters;
    retainedEliteParameters.seed = 4;
    retainedEliteParameters.popSize = 1;
    retainedEliteParameters.enableLogging = true;
    MA retainedEliteAlgorithm(&retainedEliteInstance, retainedEliteParameters);
    retainedEliteAlgorithm.initialize_search();
    retainedEliteAlgorithm.run_generation();
    assert(retainedEliteAlgorithm.retainedLowerElite != nullptr);
    assert(std::isfinite(
        retainedEliteAlgorithm.retainedLowerElite->get_lower_cost()));

    const auto retainedRoutes =
        retainedEliteAlgorithm.retainedLowerElite->get_routes();
    const double retainedUpperCost =
        retainedEliteAlgorithm.retainedLowerElite->get_upper_cost();
    const double retainedLowerCost =
        retainedEliteAlgorithm.retainedLowerElite->get_lower_cost();
    const std::uint64_t callsBeforeReuse =
        retainedEliteInstance.get_distance_calls();
    retainedEliteAlgorithm.localSearchOperatorRows.str("");
    retainedEliteAlgorithm.localSearchOperatorRows.clear();
    retainedEliteAlgorithm.write_local_search_operator_snapshot();
    retainedEliteAlgorithm.localSearchOperatorRows.str("");
    retainedEliteAlgorithm.localSearchOperatorRows.clear();

    retainedEliteAlgorithm.run_generation();
    retainedEliteAlgorithm.write_local_search_operator_snapshot();

    assert(retainedEliteInstance.get_distance_calls() == callsBeforeReuse);
    assert(retainedEliteAlgorithm.population.size() == 1);
    assert(retainedEliteAlgorithm.population.front()
           == retainedEliteAlgorithm.retainedLowerElite);
    assert(retainedEliteAlgorithm.retainedLowerElite->get_routes() == retainedRoutes);
    assert(std::fabs(
        retainedEliteAlgorithm.retainedLowerElite->get_upper_cost()
        - retainedUpperCost) <= 1e-8);
    assert(std::fabs(
        retainedEliteAlgorithm.retainedLowerElite->get_lower_cost()
        - retainedLowerCost) <= 1e-8);

    std::istringstream retainedEliteOperatorRows(
        retainedEliteAlgorithm.localSearchOperatorRows.str());
    int retainedEliteOperatorRowCount = 0;
    while (std::getline(
        retainedEliteOperatorRows,
        localSearchRow)) {
        std::istringstream rowStream(localSearchRow);
        std::vector<std::string> columns;
        std::string column;
        while (std::getline(rowStream, column, '\t')) {
            columns.push_back(column);
        }
        assert(columns.size() == 8);
        assert(std::stoi(columns[0]) == 2);
        assert(std::stoi(columns[1]) == 1);
        assert(std::stoi(columns[3]) == 0);
        assert(std::stoi(columns[4]) == 0);
        assert(std::fabs(std::stod(columns[5])) <= 1e-12);
        ++retainedEliteOperatorRowCount;
    }
    assert(
        retainedEliteOperatorRowCount
        == static_cast<int>(LOCAL_SEARCH_OPERATOR_COUNT));

    Case randomAllocationInstance(instancePath, 43);
    Parameters randomAllocationParameters;
    randomAllocationParameters.seed = 43;
    randomAllocationParameters.popSize = 10;
    randomAllocationParameters.enableLogging = true;
    randomAllocationParameters.localSearchPolicy =
        LocalSearchPolicy::RandomMixed;
    MA randomAllocationAlgorithm(
        &randomAllocationInstance,
        randomAllocationParameters);
    randomAllocationAlgorithm.initialize_search();
    randomAllocationAlgorithm.run_generation();
    assert(
        randomAllocationAlgorithm
            .localSearchAllocationRows.str().empty());
    assert(
        randomAllocationAlgorithm
            .pendingLocalSearchAllocationGenerations == 1);
    randomAllocationAlgorithm
        .write_local_search_allocation_snapshot();
    std::istringstream allocationRows(
        randomAllocationAlgorithm
            .localSearchAllocationRows.str());
    int allocationRowCount = 0;
    int selectionCount = 0;
    while (std::getline(allocationRows, localSearchRow)) {
        std::istringstream rowStream(localSearchRow);
        std::vector<std::string> columns;
        std::string column;
        while (std::getline(rowStream, column, '\t')) {
            columns.push_back(column);
        }
        assert(columns.size() == 24);
        assert(columns[1] == "1");
        assert(columns[2] == "random");
        const double rewardComponentSum =
            std::stod(columns[17])
            + std::stod(columns[18])
            + std::stod(columns[19]);
        assert(std::fabs(
            rewardComponentSum - std::stod(columns[21]))
            <= 1e-8);
        assert(std::stod(columns[20]) >= 0.0);
        assert(std::stod(columns[22]) >= 0.0);
        if (columns[3] == "weak") {
            assert(std::fabs(std::stod(columns[22])) <= 1e-12);
        }
        const int terminationCount =
            std::stoi(columns[7])
            + std::stoi(columns[8])
            + std::stoi(columns[9]);
        assert(terminationCount == std::stoi(columns[4]));
        selectionCount += std::stoi(columns[4]);
        ++allocationRowCount;
    }
    assert(allocationRowCount == 3);
    assert(selectionCount == randomAllocationParameters.popSize);
    assert(randomAllocationAlgorithm.upperBestIndividual != nullptr);
    assert(std::fabs(
        randomAllocationAlgorithm
            .upperBestIndividual->get_upper_cost()
        - randomAllocationAlgorithm.globalBestUpperCost) <= 1e-8);

    Case boundedAllocationInstance(instancePath, 45);
    Parameters boundedAllocationParameters =
        randomAllocationParameters;
    boundedAllocationParameters.seed = 45;
    boundedAllocationParameters.localSearchIntensity =
        LocalSearchIntensity::BoundedStrong;
    MA boundedAllocationAlgorithm(
        &boundedAllocationInstance,
        boundedAllocationParameters);
    boundedAllocationAlgorithm.initialize_search();
    boundedAllocationAlgorithm.run_generation();
    boundedAllocationAlgorithm
        .write_local_search_allocation_snapshot();
    const std::string boundedAllocationRows =
        boundedAllocationAlgorithm
            .localSearchAllocationRows.str();
    assert(
        boundedAllocationRows.find("\tbounded_strong\t")
        != std::string::npos);
    assert(
        boundedAllocationRows.find("\tstrong\t")
        == std::string::npos);

    Case matchedAllocationInstance(instancePath, 46);
    Parameters matchedAllocationParameters =
        boundedAllocationParameters;
    matchedAllocationParameters.seed = 46;
    matchedAllocationParameters.localSearchPolicy =
        LocalSearchPolicy::MatchedRandom;
    matchedAllocationParameters.operatorSelectionPolicy =
        OperatorSelectionPolicy::BudgetAware;
    MA matchedAllocationAlgorithm(
        &matchedAllocationInstance,
        matchedAllocationParameters);
    matchedAllocationAlgorithm.initialize_search();
    matchedAllocationAlgorithm.run_generation();
    matchedAllocationAlgorithm
        .write_local_search_allocation_snapshot();
    const std::string matchedAllocationRows =
        matchedAllocationAlgorithm
            .localSearchAllocationRows.str();
    std::istringstream matchedRows(matchedAllocationRows);
    int matchedRowCount = 0;
    int matchedSelectionCount = 0;
    while (std::getline(matchedRows, localSearchRow)) {
        std::istringstream rowStream(localSearchRow);
        std::vector<std::string> columns;
        std::string column;
        while (std::getline(rowStream, column, '\t')) {
            columns.push_back(column);
        }
        assert(columns.size() == 24);
        assert(columns[1] == "1");
        assert(columns[2] == "matched_random");
        const int terminationCount =
            std::stoi(columns[7])
            + std::stoi(columns[8])
            + std::stoi(columns[9]);
        assert(terminationCount == std::stoi(columns[4]));
        matchedSelectionCount += std::stoi(columns[4]);
        ++matchedRowCount;
    }
    assert(matchedRowCount == 3);
    assert(
        matchedSelectionCount
        == matchedAllocationParameters.popSize);
    assert(
        matchedAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Weak)
        + matchedAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Medium)
        + matchedAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Strong)
        == 0);
    assert(std::string(MA::OPERATOR_LEARNING_LOG_HEADER)
           == "iter\tgenerations\toperator\tcalls\taccepts\t"
              "distance_calls\tupper_gain\trelative_upper_gain\t"
              "gamma_crosses\t"
              "relative_gain_per_million_distance_calls\t"
              "avg_estimated_cost_per_call\tavg_efficiency\t"
              "avg_target_budget_share\trealized_budget_share\t"
              "avg_selection_probability");
    matchedAllocationAlgorithm.write_operator_learning_snapshot();
    std::istringstream operatorLearningRows(
        matchedAllocationAlgorithm.operatorLearningRows.str());
    int operatorLearningRowCount = 0;
    double loggedTargetBudgetShare = 0.0;
    double loggedSelectionProbability = 0.0;
    while (std::getline(
        operatorLearningRows,
        localSearchRow)) {
        std::istringstream rowStream(localSearchRow);
        std::vector<std::string> columns;
        std::string column;
        while (std::getline(rowStream, column, '\t')) {
            columns.push_back(column);
        }
        assert(columns.size() == 15);
        assert(columns[1] == "1");
        assert(
            std::stoull(columns[5])
            <= matchedAllocationInstance.get_distance_calls());
        assert(std::stod(columns[9]) >= 0.0);
        assert(std::stod(columns[10]) >= 1.0);
        assert(std::stod(columns[11]) >= 0.0);
        loggedTargetBudgetShare += std::stod(columns[12]);
        loggedSelectionProbability += std::stod(columns[14]);
        ++operatorLearningRowCount;
    }
    assert(
        operatorLearningRowCount
        == static_cast<int>(LOCAL_SEARCH_OPERATOR_COUNT));
    assert(std::fabs(loggedTargetBudgetShare - 1.0) <= 1e-8);
    assert(std::fabs(loggedSelectionProbability - 1.0) <= 1e-8);

    Case onlineAllocationInstance(instancePath, 47);
    Parameters onlineAllocationParameters =
        randomAllocationParameters;
    onlineAllocationParameters.seed = 47;
    onlineAllocationParameters.localSearchPolicy =
        LocalSearchPolicy::OnlineIndividual;
    MA onlineAllocationAlgorithm(
        &onlineAllocationInstance,
        onlineAllocationParameters);
    onlineAllocationAlgorithm.initialize_search();
    onlineAllocationAlgorithm.run_generation();
    onlineAllocationAlgorithm.run_generation();
    onlineAllocationAlgorithm
        .write_local_search_allocation_snapshot();
    const std::string onlineAllocationRows =
        onlineAllocationAlgorithm
            .localSearchAllocationRows.str();
    assert(std::count(
        onlineAllocationRows.begin(),
        onlineAllocationRows.end(),
        '\n') == 3);
    const int onlineObservationCount =
        onlineAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Weak)
        + onlineAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Medium)
        + onlineAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Strong);
    assert(onlineObservationCount >= 10);

    Case nonContextualAllocationInstance(instancePath, 49);
    Parameters nonContextualAllocationParameters =
        boundedAllocationParameters;
    nonContextualAllocationParameters.seed = 49;
    nonContextualAllocationParameters.localSearchPolicy =
        LocalSearchPolicy::OnlineNonContextual;
    MA nonContextualAllocationAlgorithm(
        &nonContextualAllocationInstance,
        nonContextualAllocationParameters);
    nonContextualAllocationAlgorithm.initialize_search();
    nonContextualAllocationAlgorithm.run_generation();
    nonContextualAllocationAlgorithm.run_generation();
    nonContextualAllocationAlgorithm
        .write_local_search_allocation_snapshot();
    const std::string nonContextualAllocationRows =
        nonContextualAllocationAlgorithm
            .localSearchAllocationRows.str();
    assert(std::count(
        nonContextualAllocationRows.begin(),
        nonContextualAllocationRows.end(),
        '\n') == 3);
    assert(
        nonContextualAllocationRows.find("\tnon_contextual\t")
        != std::string::npos);
    const int nonContextualObservationCount =
        nonContextualAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Weak)
        + nonContextualAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Medium)
        + nonContextualAllocationAlgorithm.localSearchAllocator
            .observation_count(LocalSearchIntensity::Strong);
    assert(nonContextualObservationCount >= 10);
    return 0;
}
