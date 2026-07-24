#include "leader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "case.hpp"
#include "individual.hpp"

namespace {

enum class Neighborhood {
    TwoOpt,
    TwoOptStar,
    NodeShift,
    InterRouteRelocate,
    InterRouteSwap,
    SwapStar,
    IntraRouteSwap,
    TwoOptStarHeadToHead,
    TwoOptStarHeadToTail,
};

constexpr double kImprovementTolerance = 0.00000001;
constexpr double kWeakMoveFraction = 0.02;
constexpr double kMediumMoveFraction = 0.10;

constexpr std::array<Neighborhood, 3> kThreeNeighborhoods = {
    Neighborhood::TwoOpt,
    Neighborhood::TwoOptStar,
    Neighborhood::NodeShift,
};

constexpr std::array<Neighborhood, 5> kFiveNeighborhoods = {
    Neighborhood::TwoOpt,
    Neighborhood::TwoOptStar,
    Neighborhood::NodeShift,
    Neighborhood::InterRouteRelocate,
    Neighborhood::InterRouteSwap,
};

constexpr std::array<Neighborhood, 7> kSevenNeighborhoods = {
    Neighborhood::NodeShift,
    Neighborhood::InterRouteRelocate,
    Neighborhood::IntraRouteSwap,
    Neighborhood::InterRouteSwap,
    Neighborhood::TwoOpt,
    Neighborhood::TwoOptStarHeadToHead,
    Neighborhood::TwoOptStarHeadToTail,
};

constexpr std::array<LocalSearchOperator, 8> kEightOperators = {
    LocalSearchOperator::NodeShift,
    LocalSearchOperator::InterRouteRelocate,
    LocalSearchOperator::IntraRouteSwap,
    LocalSearchOperator::InterRouteSwap,
    LocalSearchOperator::SwapStar,
    LocalSearchOperator::TwoOpt,
    LocalSearchOperator::TwoOptStarHeadToHead,
    LocalSearchOperator::TwoOptStarHeadToTail,
};

LocalSearchOperator local_search_operator(Neighborhood neighborhood) {
    switch (neighborhood) {
        case Neighborhood::NodeShift:
            return LocalSearchOperator::NodeShift;
        case Neighborhood::InterRouteRelocate:
            return LocalSearchOperator::InterRouteRelocate;
        case Neighborhood::IntraRouteSwap:
            return LocalSearchOperator::IntraRouteSwap;
        case Neighborhood::InterRouteSwap:
            return LocalSearchOperator::InterRouteSwap;
        case Neighborhood::SwapStar:
            return LocalSearchOperator::SwapStar;
        case Neighborhood::TwoOpt:
            return LocalSearchOperator::TwoOpt;
        case Neighborhood::TwoOptStarHeadToHead:
            return LocalSearchOperator::TwoOptStarHeadToHead;
        case Neighborhood::TwoOptStarHeadToTail:
            return LocalSearchOperator::TwoOptStarHeadToTail;
        case Neighborhood::TwoOptStar:
            return LocalSearchOperator::Count;
    }
    return LocalSearchOperator::Count;
}

Neighborhood neighborhood_for_operator(
    LocalSearchOperator localSearchOperator) {
    switch (localSearchOperator) {
        case LocalSearchOperator::NodeShift:
            return Neighborhood::NodeShift;
        case LocalSearchOperator::InterRouteRelocate:
            return Neighborhood::InterRouteRelocate;
        case LocalSearchOperator::IntraRouteSwap:
            return Neighborhood::IntraRouteSwap;
        case LocalSearchOperator::InterRouteSwap:
            return Neighborhood::InterRouteSwap;
        case LocalSearchOperator::SwapStar:
            return Neighborhood::SwapStar;
        case LocalSearchOperator::TwoOpt:
            return Neighborhood::TwoOpt;
        case LocalSearchOperator::TwoOptStarHeadToHead:
            return Neighborhood::TwoOptStarHeadToHead;
        case LocalSearchOperator::TwoOptStarHeadToTail:
            return Neighborhood::TwoOptStarHeadToTail;
        case LocalSearchOperator::Count:
            break;
    }
    throw std::logic_error("unknown local-search operator");
}

std::size_t operator_index(LocalSearchOperator localSearchOperator) {
    return static_cast<std::size_t>(localSearchOperator);
}

void clear_failure_stamps(LocalSearchWorkspace& workspace) {
    for (auto& failedVersions : workspace.failedRouteVersions) {
        std::fill(failedVersions.begin(), failedVersions.end(), 0);
    }
    for (auto& routePairPool : workspace.activeRoutePairPools) {
        routePairPool.topologyVersion = 0;
    }
    workspace.nextRouteVersion = 1;
}

void advance_route_topology_version(LocalSearchWorkspace& workspace) {
    if (workspace.routeTopologyVersion
        == std::numeric_limits<std::uint64_t>::max()) {
        for (auto& routePairPool : workspace.activeRoutePairPools) {
            routePairPool.topologyVersion = 0;
        }
        workspace.routeTopologyVersion = 1;
        return;
    }
    ++workspace.routeTopologyVersion;
}

std::uint64_t next_route_version(LocalSearchWorkspace& workspace) {
    if (workspace.nextRouteVersion
        == std::numeric_limits<std::uint64_t>::max()) {
        clear_failure_stamps(workspace);
        advance_route_topology_version(workspace);
    }
    return workspace.nextRouteVersion++;
}

void begin_failure_cache_session(
    int routeCount,
    LocalSearchWorkspace& workspace) {
    const std::size_t requiredStride =
        static_cast<std::size_t>(routeCount);
    if (workspace.routePairCacheStride < requiredStride) {
        workspace.routePairCacheStride = requiredStride;
    }

    advance_route_topology_version(workspace);
    workspace.routeVersions.resize(workspace.routePairCacheStride);
    for (auto& failedVersions : workspace.failedRouteVersions) {
        failedVersions.resize(workspace.routePairCacheStride, 0);
    }
    for (int routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
        workspace.routeVersions[static_cast<std::size_t>(routeIndex)] =
            next_route_version(workspace);
    }
    workspace.changedRouteCount = 0;
    workspace.allRoutesChanged = false;
}

bool route_failure_is_cached(
    LocalSearchOperator localSearchOperator,
    int routeIndex,
    const LocalSearchWorkspace& workspace) {
    const auto& failedVersions =
        workspace.failedRouteVersions[operator_index(localSearchOperator)];
    return failedVersions[static_cast<std::size_t>(routeIndex)]
        == workspace.routeVersions[static_cast<std::size_t>(routeIndex)];
}

void cache_route_failure(
    LocalSearchOperator localSearchOperator,
    int routeIndex,
    LocalSearchWorkspace& workspace) {
    workspace.failedRouteVersions[operator_index(localSearchOperator)]
        [static_cast<std::size_t>(routeIndex)] =
            workspace.routeVersions[static_cast<std::size_t>(routeIndex)];
}

std::size_t route_pair_cache_index(
    int firstRoute,
    int secondRoute,
    const LocalSearchWorkspace& workspace) {
    return static_cast<std::size_t>(firstRoute)
        * workspace.routePairCacheStride
        + static_cast<std::size_t>(secondRoute);
}

void add_active_route_pair(
    LocalSearchWorkspace::ActiveRoutePairPool& routePairPool,
    int firstRoute,
    int secondRoute,
    const LocalSearchWorkspace& workspace) {
    const std::size_t membershipIndex =
        route_pair_cache_index(firstRoute, secondRoute, workspace);
    if (routePairPool.membership[membershipIndex] != 0) {
        return;
    }
    routePairPool.membership[membershipIndex] = 1;
    routePairPool.pairs.emplace_back(firstRoute, secondRoute);
}

void initialize_active_route_pair_pool(
    LocalSearchOperator localSearchOperator,
    int routeCount,
    bool directed,
    LocalSearchWorkspace& workspace) {
    auto& routePairPool =
        workspace.activeRoutePairPools[operator_index(localSearchOperator)];
    routePairPool.pairs.clear();
    routePairPool.membership.assign(
        workspace.routePairCacheStride * workspace.routePairCacheStride,
        0);
    const int pairCount = directed
        ? routeCount * (routeCount - 1)
        : routeCount * (routeCount - 1) / 2;
    routePairPool.pairs.reserve(static_cast<std::size_t>(pairCount));

    for (int firstRoute = 0; firstRoute < routeCount; ++firstRoute) {
        const int secondRouteBegin = directed ? 0 : firstRoute + 1;
        for (int secondRoute = secondRouteBegin;
             secondRoute < routeCount;
             ++secondRoute) {
            if (firstRoute != secondRoute) {
                add_active_route_pair(
                    routePairPool,
                    firstRoute,
                    secondRoute,
                    workspace);
            }
        }
    }

    routePairPool.observedRouteVersions.resize(
        workspace.routePairCacheStride);
    for (int routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
        routePairPool.observedRouteVersions[
            static_cast<std::size_t>(routeIndex)] =
            workspace.routeVersions[
                static_cast<std::size_t>(routeIndex)];
    }
    routePairPool.topologyVersion = workspace.routeTopologyVersion;
    routePairPool.routeCount = routeCount;
    routePairPool.directed = directed;
}

void reactivate_incident_route_pairs(
    LocalSearchWorkspace::ActiveRoutePairPool& routePairPool,
    int changedRoute,
    const LocalSearchWorkspace& workspace) {
    for (int otherRoute = 0;
         otherRoute < routePairPool.routeCount;
         ++otherRoute) {
        if (otherRoute == changedRoute) {
            continue;
        }
        if (routePairPool.directed) {
            add_active_route_pair(
                routePairPool,
                changedRoute,
                otherRoute,
                workspace);
            add_active_route_pair(
                routePairPool,
                otherRoute,
                changedRoute,
                workspace);
        } else {
            add_active_route_pair(
                routePairPool,
                std::min(changedRoute, otherRoute),
                std::max(changedRoute, otherRoute),
                workspace);
        }
    }
}

LocalSearchWorkspace::ActiveRoutePairPool& synchronize_active_route_pairs(
    LocalSearchOperator localSearchOperator,
    int routeCount,
    bool directed,
    LocalSearchWorkspace& workspace) {
    auto& routePairPool =
        workspace.activeRoutePairPools[operator_index(localSearchOperator)];
    if (routePairPool.topologyVersion
            != workspace.routeTopologyVersion
        || routePairPool.routeCount != routeCount
        || routePairPool.directed != directed) {
        initialize_active_route_pair_pool(
            localSearchOperator,
            routeCount,
            directed,
            workspace);
    } else {
        for (int routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
            const std::size_t versionIndex =
                static_cast<std::size_t>(routeIndex);
            if (routePairPool.observedRouteVersions[versionIndex]
                == workspace.routeVersions[versionIndex]) {
                continue;
            }
            reactivate_incident_route_pairs(
                routePairPool,
                routeIndex,
                workspace);
            routePairPool.observedRouteVersions[versionIndex] =
                workspace.routeVersions[versionIndex];
        }
    }
    return routePairPool;
}

std::size_t select_random_active_route_pair(
    const LocalSearchWorkspace::ActiveRoutePairPool& routePairPool,
    std::mt19937& randomEngine) {
    // Removing failed samples makes this a lazy random permutation prefix.
    std::uniform_int_distribution<std::size_t> selectPair(
        0,
        routePairPool.pairs.size() - 1);
    return selectPair(randomEngine);
}

void discard_active_route_pair(
    LocalSearchWorkspace::ActiveRoutePairPool& routePairPool,
    std::size_t pairIndex,
    const LocalSearchWorkspace& workspace) {
    const auto [firstRoute, secondRoute] =
        routePairPool.pairs[pairIndex];
    routePairPool.membership[
        route_pair_cache_index(
            firstRoute,
            secondRoute,
            workspace)] = 0;
    if (pairIndex + 1 != routePairPool.pairs.size()) {
        routePairPool.pairs[pairIndex] =
            routePairPool.pairs.back();
    }
    routePairPool.pairs.pop_back();
}

void reset_move_impact(LocalSearchWorkspace& workspace) {
    workspace.changedRouteCount = 0;
    workspace.changedRoutes = {-1, -1};
    workspace.allRoutesChanged = false;
}

void record_changed_route(
    int routeIndex,
    LocalSearchWorkspace& workspace) {
    for (int changedIndex = 0;
         changedIndex < workspace.changedRouteCount;
         ++changedIndex) {
        if (workspace.changedRoutes[
                static_cast<std::size_t>(changedIndex)] == routeIndex) {
            return;
        }
    }
    if (workspace.changedRouteCount
        >= static_cast<int>(workspace.changedRoutes.size())) {
        throw std::logic_error(
            "local-search move changed more than two routes");
    }
    workspace.changedRoutes[
        static_cast<std::size_t>(workspace.changedRouteCount++)] =
            routeIndex;
}

void record_changed_route_pair(
    int firstRoute,
    int secondRoute,
    LocalSearchWorkspace& workspace) {
    record_changed_route(firstRoute, workspace);
    record_changed_route(secondRoute, workspace);
}

void record_all_routes_changed(LocalSearchWorkspace& workspace) {
    workspace.allRoutesChanged = true;
}

void invalidate_failure_cache_after_move(
    int routeCount,
    LocalSearchWorkspace& workspace) {
    if (workspace.allRoutesChanged) {
        advance_route_topology_version(workspace);
        for (int routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
            workspace.routeVersions[static_cast<std::size_t>(routeIndex)] =
                next_route_version(workspace);
        }
        return;
    }
    if (workspace.changedRouteCount == 0) {
        throw std::logic_error(
            "improving local-search move did not report changed routes");
    }
    for (int changedIndex = 0;
         changedIndex < workspace.changedRouteCount;
         ++changedIndex) {
        const int routeIndex = workspace.changedRoutes[
            static_cast<std::size_t>(changedIndex)];
        workspace.routeVersions[static_cast<std::size_t>(routeIndex)] =
            next_route_version(workspace);
    }
}

struct RoutePairHash {
    std::size_t operator()(const std::pair<int, int>& routePair) const {
        return routePair.first * 256 + routePair.second;
    }
};

void move_node(int* route, int fromIndex, int toIndex) {
    const int node = route[fromIndex];
    if (fromIndex < toIndex) {
        for (int i = fromIndex; i < toIndex; ++i) {
            route[i] = route[i + 1];
        }
        route[toIndex] = node;
    } else if (fromIndex > toIndex) {
        for (int i = fromIndex; i > toIndex; --i) {
            route[i] = route[i - 1];
        }
        route[toIndex] = node;
    }
}

bool shift_node_within_route(int* route, int length, double& upperCost, Case& instance) {
    if (length <= 4) {
        return false;
    }

    double bestImprovement = 0;
    bool improved = false;
    do {
        bestImprovement = 0;
        int bestFromIndex = 0;
        int bestToIndex = 0;
        for (int fromIndex = 1; fromIndex < length - 1; ++fromIndex) {
            for (int toIndex = 1; toIndex < length - 1; ++toIndex) {
                double improvement = 0;
                if (fromIndex < toIndex) {
                    const double oldCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex], route[toIndex + 1]);
                    const double newCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[toIndex + 1]);
                    improvement = oldCost - newCost;
                } else if (fromIndex > toIndex) {
                    const double oldCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex - 1], route[toIndex]);
                    const double newCost =
                        instance.get_distance(route[toIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[toIndex])
                        + instance.get_distance(route[fromIndex - 1], route[fromIndex + 1]);
                    improvement = oldCost - newCost;
                }

                if (std::fabs(improvement) < 0.00000001) {
                    improvement = 0;
                }
                if (bestImprovement < improvement) {
                    bestImprovement = improvement;
                    bestFromIndex = fromIndex;
                    bestToIndex = toIndex;
                    improved = true;
                }
            }
        }

        if (bestImprovement > 0) {
            move_node(route, bestFromIndex, bestToIndex);
            upperCost -= bestImprovement;
        }
    } while (bestImprovement > 0);

    return improved;
}

double improve_route_with_two_opt(std::vector<int>& route, Case& instance) {
    bool improved = true;
    double totalChange = 0.0;

    while (improved) {
        improved = false;
        for (std::size_t i = 1; i < route.size() - 2; ++i) {
            for (std::size_t j = i + 1; j < route.size() - 1; ++j) {
                const double oldCost = instance.get_distance(route[i - 1], route[i])
                    + instance.get_distance(route[j], route[j + 1]);
                const double newCost = instance.get_distance(route[i - 1], route[j])
                    + instance.get_distance(route[i], route[j + 1]);

                if (newCost < oldCost) {
                    std::reverse(route.begin() + i, route.begin() + j + 1);
                    improved = true;
                    totalChange += newCost - oldCost;
                }
            }
        }
    }
    return totalChange;
}

bool improve_with_two_opt(Individual& individual, Case& instance) {
    std::vector<std::vector<int>> routes = individual.get_routes();
    double totalChange = 0;
    for (auto& route : routes) {
        totalChange += improve_route_with_two_opt(route, instance);
    }
    if (totalChange != 0.0) {
        individual.set_routes(routes);
        individual.set_upper_cost(individual.get_upper_cost() + totalChange);
    }
    return totalChange != 0;
}

bool improve_with_two_opt_star(Individual& individual, Case& instance) {
    if (individual.route_num == 1) {
        return false;
    }

    std::unordered_set<std::pair<int, int>, RoutePairHash> routePairs;
    for (int firstRoute = 0; firstRoute < individual.route_num - 1; ++firstRoute) {
        for (int secondRoute = firstRoute + 1; secondRoute < individual.route_num; ++secondRoute) {
            routePairs.insert(std::make_pair(firstRoute, secondRoute));
        }
    }

    int* firstRouteBuffer = new int[individual.node_cap];
    int* secondRouteBuffer = new int[individual.node_cap];
    bool improved = false;
    bool routePairImproved = false;
    while (!routePairs.empty()) {
        routePairImproved = false;
        const int firstRoute = routePairs.begin()->first;
        const int secondRoute = routePairs.begin()->second;
        routePairs.erase(routePairs.begin());
        int firstPrefixDemand = 0;

        for (int firstNode = 0; firstNode < individual.node_num[firstRoute] - 1; ++firstNode) {
            firstPrefixDemand += instance.get_customer_demand(individual.routes[firstRoute][firstNode]);
            int secondPrefixDemand = 0;
            for (int secondNode = 0; secondNode < individual.node_num[secondRoute] - 1; ++secondNode) {
                secondPrefixDemand += instance.get_customer_demand(individual.routes[secondRoute][secondNode]);

                const bool exchangeTailsFeasible =
                    firstPrefixDemand + individual.demand_sum[secondRoute] - secondPrefixDemand <= instance.maxC
                    && secondPrefixDemand + individual.demand_sum[firstRoute] - firstPrefixDemand <= instance.maxC;
                if (exchangeTailsFeasible) {
                    const double oldCost =
                        instance.get_distance(
                            individual.routes[firstRoute][firstNode],
                            individual.routes[firstRoute][firstNode + 1])
                        + instance.get_distance(
                            individual.routes[secondRoute][secondNode],
                            individual.routes[secondRoute][secondNode + 1]);
                    const double newCost =
                        instance.get_distance(
                            individual.routes[firstRoute][firstNode],
                            individual.routes[secondRoute][secondNode + 1])
                        + instance.get_distance(
                            individual.routes[secondRoute][secondNode],
                            individual.routes[firstRoute][firstNode + 1]);
                    const double improvement = oldCost - newCost;
                    if (improvement > 0.00000001) {
                        individual.set_upper_cost(individual.get_upper_cost() - improvement);
                        std::memcpy(
                            firstRouteBuffer,
                            individual.routes[firstRoute],
                            sizeof(int) * individual.node_cap);

                        int firstRouteLength = firstNode + 1;
                        for (int i = secondNode + 1; i < individual.node_num[secondRoute]; ++i) {
                            individual.routes[firstRoute][firstRouteLength++] = individual.routes[secondRoute][i];
                        }
                        int secondRouteLength = secondNode + 1;
                        for (int i = firstNode + 1; i < individual.node_num[firstRoute]; ++i) {
                            individual.routes[secondRoute][secondRouteLength++] = firstRouteBuffer[i];
                        }
                        individual.node_num[firstRoute] = firstRouteLength;
                        individual.node_num[secondRoute] = secondRouteLength;

                        const int firstRouteDemand = firstPrefixDemand
                            + individual.demand_sum[secondRoute] - secondPrefixDemand;
                        const int secondRouteDemand = secondPrefixDemand
                            + individual.demand_sum[firstRoute] - firstPrefixDemand;
                        individual.demand_sum[firstRoute] = firstRouteDemand;
                        individual.demand_sum[secondRoute] = secondRouteDemand;
                        improved = true;
                        routePairImproved = true;
                    }
                } else {
                    const bool reverseExchangeFeasible =
                        firstPrefixDemand + secondPrefixDemand <= instance.maxC
                        && individual.demand_sum[firstRoute] - firstPrefixDemand
                            + individual.demand_sum[secondRoute] - secondPrefixDemand <= instance.maxC;
                    if (reverseExchangeFeasible) {
                        const double oldCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[firstRoute][firstNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode],
                                individual.routes[secondRoute][secondNode + 1]);
                        const double newCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[secondRoute][secondNode])
                            + instance.get_distance(
                                individual.routes[firstRoute][firstNode + 1],
                                individual.routes[secondRoute][secondNode + 1]);
                        const double improvement = oldCost - newCost;
                        if (improvement > 0.00000001) {
                            individual.set_upper_cost(individual.get_upper_cost() - improvement);
                            std::memcpy(
                                firstRouteBuffer,
                                individual.routes[firstRoute],
                                sizeof(int) * individual.node_cap);

                            int firstRouteLength = firstNode + 1;
                            for (int i = secondNode; i >= 0; --i) {
                                individual.routes[firstRoute][firstRouteLength++] = individual.routes[secondRoute][i];
                            }
                            int secondRouteLength = 0;
                            for (int i = individual.node_num[firstRoute] - 1; i >= firstNode + 1; --i) {
                                secondRouteBuffer[secondRouteLength++] = firstRouteBuffer[i];
                            }
                            for (int i = secondNode + 1; i < individual.node_num[secondRoute]; ++i) {
                                secondRouteBuffer[secondRouteLength++] = individual.routes[secondRoute][i];
                            }
                            std::memcpy(
                                individual.routes[secondRoute],
                                secondRouteBuffer,
                                sizeof(int) * individual.node_cap);
                            individual.node_num[firstRoute] = firstRouteLength;
                            individual.node_num[secondRoute] = secondRouteLength;

                            const int firstRouteDemand = firstPrefixDemand + secondPrefixDemand;
                            const int secondRouteDemand = individual.demand_sum[firstRoute]
                                + individual.demand_sum[secondRoute]
                                - firstPrefixDemand - secondPrefixDemand;
                            individual.demand_sum[firstRoute] = firstRouteDemand;
                            individual.demand_sum[secondRoute] = secondRouteDemand;
                            improved = true;
                            routePairImproved = true;
                        }
                    }
                }

                if (routePairImproved) {
                    for (int i = 0; i < firstRoute; ++i) {
                        routePairs.insert({i, firstRoute});
                    }
                    for (int i = 0; i < secondRoute; ++i) {
                        routePairs.insert({i, secondRoute});
                    }

                    if (individual.demand_sum[firstRoute] == 0) {
                        int* emptyRoute = individual.routes[firstRoute];
                        individual.routes[firstRoute] = individual.routes[individual.route_num - 1];
                        individual.routes[individual.route_num - 1] = emptyRoute;
                        individual.demand_sum[firstRoute] = individual.demand_sum[individual.route_num - 1];
                        individual.node_num[firstRoute] = individual.node_num[individual.route_num - 1];
                        --individual.route_num;
                        for (int i = 0; i < individual.route_num; ++i) {
                            routePairs.erase({i, individual.route_num});
                        }
                    }
                    if (individual.demand_sum[secondRoute] == 0) {
                        int* emptyRoute = individual.routes[secondRoute];
                        individual.routes[secondRoute] = individual.routes[individual.route_num - 1];
                        individual.routes[individual.route_num - 1] = emptyRoute;
                        individual.demand_sum[secondRoute] = individual.demand_sum[individual.route_num - 1];
                        individual.node_num[secondRoute] = individual.node_num[individual.route_num - 1];
                        --individual.route_num;
                        for (int i = 0; i < individual.route_num; ++i) {
                            routePairs.erase({i, individual.route_num});
                        }
                    }
                    break;
                }
            }
            if (routePairImproved) {
                break;
            }
        }
    }

    delete[] firstRouteBuffer;
    delete[] secondRouteBuffer;
    return improved;
}

bool improve_with_node_shift(Individual& individual, Case& instance) {
    double upperCost = individual.get_upper_cost();
    bool improved = false;
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        improved = shift_node_within_route(
            individual.routes[routeIndex],
            individual.node_num[routeIndex],
            upperCost,
            instance) || improved;
    }
    if (improved) {
        individual.set_upper_cost(upperCost);
    }
    return improved;
}

bool swap_nodes_within_route(
    int* route,
    int length,
    double& upperCost,
    Case& instance) {
    if (length < 5) {
        return false;
    }

    bool improved = false;
    bool moveApplied;
    do {
        moveApplied = false;
        for (int firstNode = 1;
             firstNode < length - 2 && !moveApplied;
             ++firstNode) {
            for (int secondNode = firstNode + 2;
                 secondNode < length - 1;
                 ++secondNode) {
                const double oldCost =
                    instance.get_distance(route[firstNode - 1], route[firstNode])
                    + instance.get_distance(route[firstNode], route[firstNode + 1])
                    + instance.get_distance(route[secondNode - 1], route[secondNode])
                    + instance.get_distance(route[secondNode], route[secondNode + 1]);
                const double newCost =
                    instance.get_distance(route[firstNode - 1], route[secondNode])
                    + instance.get_distance(route[secondNode], route[firstNode + 1])
                    + instance.get_distance(route[secondNode - 1], route[firstNode])
                    + instance.get_distance(route[firstNode], route[secondNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::swap(route[firstNode], route[secondNode]);
                upperCost -= improvement;
                improved = true;
                moveApplied = true;
                break;
            }
        }
    } while (moveApplied);

    return improved;
}

bool improve_with_intra_route_swap(Individual& individual, Case& instance) {
    double upperCost = individual.get_upper_cost();
    bool improved = false;
    for (int routeIndex = 0; routeIndex < individual.route_num; ++routeIndex) {
        improved = swap_nodes_within_route(
            individual.routes[routeIndex],
            individual.node_num[routeIndex],
            upperCost,
            instance) || improved;
    }
    if (improved) {
        individual.set_upper_cost(upperCost);
    }
    return improved;
}

void remove_empty_route(Individual& individual, int routeIndex) {
    const int lastRoute = individual.route_num - 1;
    if (routeIndex != lastRoute) {
        std::swap(individual.routes[routeIndex], individual.routes[lastRoute]);
        std::swap(individual.node_num[routeIndex], individual.node_num[lastRoute]);
        std::swap(individual.demand_sum[routeIndex], individual.demand_sum[lastRoute]);
    }

    individual.node_num[lastRoute] = 0;
    individual.demand_sum[lastRoute] = 0;
    --individual.route_num;
}

void replace_route(
    Individual& individual,
    int routeIndex,
    const std::vector<int>& route,
    int demand) {
    std::copy(route.begin(), route.end(), individual.routes[routeIndex]);
    individual.node_num[routeIndex] = static_cast<int>(route.size());
    individual.demand_sum[routeIndex] = demand;
}

void relocate_customer(
    Individual& individual,
    int sourceRoute,
    int sourceNode,
    int targetRoute,
    int targetEdge,
    int customer,
    int customerDemand) {
    int* source = individual.routes[sourceRoute];
    for (int i = sourceNode; i < individual.node_num[sourceRoute] - 1; ++i) {
        source[i] = source[i + 1];
    }
    --individual.node_num[sourceRoute];
    individual.demand_sum[sourceRoute] -= customerDemand;

    int* target = individual.routes[targetRoute];
    for (int i = individual.node_num[targetRoute]; i > targetEdge + 1; --i) {
        target[i] = target[i - 1];
    }
    target[targetEdge + 1] = customer;
    ++individual.node_num[targetRoute];
    individual.demand_sum[targetRoute] += customerDemand;
}

bool improve_with_inter_route_relocate(Individual& individual, Case& instance) {
    if (individual.route_num <= 1) {
        return false;
    }

    bool improved = false;
    bool moveApplied;
    do {
        moveApplied = false;
        for (int sourceRoute = 0;
             sourceRoute < individual.route_num && !moveApplied;
             ++sourceRoute) {
            const int sourceLength = individual.node_num[sourceRoute];
            for (int sourceNode = 1;
                 sourceNode < sourceLength - 1 && !moveApplied;
                 ++sourceNode) {
                const int customer = individual.routes[sourceRoute][sourceNode];
                const int customerDemand = instance.get_customer_demand(customer);
                const double sourceImprovement =
                    instance.get_distance(
                        individual.routes[sourceRoute][sourceNode - 1],
                        customer)
                    + instance.get_distance(
                        customer,
                        individual.routes[sourceRoute][sourceNode + 1])
                    - instance.get_distance(
                        individual.routes[sourceRoute][sourceNode - 1],
                        individual.routes[sourceRoute][sourceNode + 1]);

                for (int targetRoute = 0;
                     targetRoute < individual.route_num && !moveApplied;
                     ++targetRoute) {
                    if (targetRoute == sourceRoute
                        || individual.demand_sum[targetRoute] + customerDemand > instance.maxC) {
                        continue;
                    }

                    const int targetLength = individual.node_num[targetRoute];
                    for (int targetEdge = 0; targetEdge < targetLength - 1; ++targetEdge) {
                        const int targetFrom = individual.routes[targetRoute][targetEdge];
                        const int targetTo = individual.routes[targetRoute][targetEdge + 1];
                        const double improvement = sourceImprovement
                            + instance.get_distance(targetFrom, targetTo)
                            - instance.get_distance(targetFrom, customer)
                            - instance.get_distance(customer, targetTo);
                        if (improvement <= kImprovementTolerance) {
                            continue;
                        }

                        relocate_customer(
                            individual,
                            sourceRoute,
                            sourceNode,
                            targetRoute,
                            targetEdge,
                            customer,
                            customerDemand);
                        individual.set_upper_cost(individual.get_upper_cost() - improvement);
                        if (individual.node_num[sourceRoute] == 2) {
                            remove_empty_route(individual, sourceRoute);
                        }
                        improved = true;
                        moveApplied = true;
                        break;
                    }
                }
            }
        }
    } while (moveApplied && individual.route_num > 1);

    return improved;
}

bool improve_with_inter_route_swap(Individual& individual, Case& instance) {
    if (individual.route_num <= 1) {
        return false;
    }

    bool improved = false;
    bool moveApplied;
    do {
        moveApplied = false;
        for (int firstRoute = 0;
             firstRoute < individual.route_num - 1 && !moveApplied;
             ++firstRoute) {
            for (int secondRoute = firstRoute + 1;
                 secondRoute < individual.route_num && !moveApplied;
                 ++secondRoute) {
                for (int firstNode = 1;
                     firstNode < individual.node_num[firstRoute] - 1 && !moveApplied;
                     ++firstNode) {
                    const int firstCustomer = individual.routes[firstRoute][firstNode];
                    const int firstDemand = instance.get_customer_demand(firstCustomer);
                    for (int secondNode = 1;
                         secondNode < individual.node_num[secondRoute] - 1;
                         ++secondNode) {
                        const int secondCustomer = individual.routes[secondRoute][secondNode];
                        const int secondDemand = instance.get_customer_demand(secondCustomer);
                        if (individual.demand_sum[firstRoute] - firstDemand + secondDemand > instance.maxC
                            || individual.demand_sum[secondRoute] - secondDemand + firstDemand > instance.maxC) {
                            continue;
                        }

                        const double oldCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode - 1],
                                firstCustomer)
                            + instance.get_distance(
                                firstCustomer,
                                individual.routes[firstRoute][firstNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode - 1],
                                secondCustomer)
                            + instance.get_distance(
                                secondCustomer,
                                individual.routes[secondRoute][secondNode + 1]);
                        const double newCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode - 1],
                                secondCustomer)
                            + instance.get_distance(
                                secondCustomer,
                                individual.routes[firstRoute][firstNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode - 1],
                                firstCustomer)
                            + instance.get_distance(
                                firstCustomer,
                                individual.routes[secondRoute][secondNode + 1]);
                        const double improvement = oldCost - newCost;
                        if (improvement <= kImprovementTolerance) {
                            continue;
                        }

                        std::swap(
                            individual.routes[firstRoute][firstNode],
                            individual.routes[secondRoute][secondNode]);
                        individual.demand_sum[firstRoute] =
                            individual.demand_sum[firstRoute] - firstDemand + secondDemand;
                        individual.demand_sum[secondRoute] =
                            individual.demand_sum[secondRoute] - secondDemand + firstDemand;
                        individual.set_upper_cost(individual.get_upper_cost() - improvement);
                        improved = true;
                        moveApplied = true;
                        break;
                    }
                }
            }
        }
    } while (moveApplied);

    return improved;
}

bool improve_with_two_opt_star_head_to_head(
    Individual& individual,
    Case& instance) {
    if (individual.route_num <= 1) {
        return false;
    }

    bool improved = false;
    bool moveApplied;
    do {
        moveApplied = false;
        for (int firstRoute = 0;
             firstRoute < individual.route_num - 1 && !moveApplied;
             ++firstRoute) {
            for (int secondRoute = firstRoute + 1;
                 secondRoute < individual.route_num && !moveApplied;
                 ++secondRoute) {
                const int firstLength = individual.node_num[firstRoute];
                const int secondLength = individual.node_num[secondRoute];
                int firstPrefixDemand = 0;
                for (int firstNode = 0;
                     firstNode < firstLength - 1 && !moveApplied;
                     ++firstNode) {
                    firstPrefixDemand += instance.get_customer_demand(
                        individual.routes[firstRoute][firstNode]);
                    int secondPrefixDemand = 0;
                    for (int secondNode = 0;
                         secondNode < secondLength - 1;
                         ++secondNode) {
                        secondPrefixDemand += instance.get_customer_demand(
                            individual.routes[secondRoute][secondNode]);
                        const bool leavesRoutesUnchanged =
                            (firstNode == firstLength - 2 && secondNode == 0)
                            || (firstNode == 0 && secondNode == secondLength - 2);
                        if (leavesRoutesUnchanged) {
                            continue;
                        }

                        const int firstNewDemand = firstPrefixDemand + secondPrefixDemand;
                        const int secondNewDemand =
                            individual.demand_sum[firstRoute] - firstPrefixDemand
                            + individual.demand_sum[secondRoute] - secondPrefixDemand;
                        if (firstNewDemand > instance.maxC
                            || secondNewDemand > instance.maxC) {
                            continue;
                        }

                        const double oldCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[firstRoute][firstNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode],
                                individual.routes[secondRoute][secondNode + 1]);
                        const double newCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[secondRoute][secondNode])
                            + instance.get_distance(
                                individual.routes[firstRoute][firstNode + 1],
                                individual.routes[secondRoute][secondNode + 1]);
                        const double improvement = oldCost - newCost;
                        if (improvement <= kImprovementTolerance) {
                            continue;
                        }

                        const std::vector<int> firstOriginal(
                            individual.routes[firstRoute],
                            individual.routes[firstRoute] + firstLength);
                        const std::vector<int> secondOriginal(
                            individual.routes[secondRoute],
                            individual.routes[secondRoute] + secondLength);
                        std::vector<int> firstNew(
                            firstOriginal.begin(),
                            firstOriginal.begin() + firstNode + 1);
                        for (int node = secondNode; node >= 0; --node) {
                            firstNew.push_back(secondOriginal[node]);
                        }
                        std::vector<int> secondNew;
                        secondNew.reserve(firstLength + secondLength);
                        for (int node = firstLength - 1; node >= firstNode + 1; --node) {
                            secondNew.push_back(firstOriginal[node]);
                        }
                        secondNew.insert(
                            secondNew.end(),
                            secondOriginal.begin() + secondNode + 1,
                            secondOriginal.end());

                        replace_route(individual, firstRoute, firstNew, firstNewDemand);
                        replace_route(individual, secondRoute, secondNew, secondNewDemand);
                        individual.set_upper_cost(individual.get_upper_cost() - improvement);
                        if (individual.node_num[firstRoute] == 2) {
                            remove_empty_route(individual, firstRoute);
                        } else if (individual.node_num[secondRoute] == 2) {
                            remove_empty_route(individual, secondRoute);
                        }
                        improved = true;
                        moveApplied = true;
                        break;
                    }
                }
            }
        }
    } while (moveApplied && individual.route_num > 1);

    return improved;
}

bool improve_with_two_opt_star_head_to_tail(
    Individual& individual,
    Case& instance) {
    if (individual.route_num <= 1) {
        return false;
    }

    bool improved = false;
    bool moveApplied;
    do {
        moveApplied = false;
        for (int firstRoute = 0;
             firstRoute < individual.route_num - 1 && !moveApplied;
             ++firstRoute) {
            for (int secondRoute = firstRoute + 1;
                 secondRoute < individual.route_num && !moveApplied;
                 ++secondRoute) {
                const int firstLength = individual.node_num[firstRoute];
                const int secondLength = individual.node_num[secondRoute];
                int firstPrefixDemand = 0;
                for (int firstNode = 0;
                     firstNode < firstLength - 1 && !moveApplied;
                     ++firstNode) {
                    firstPrefixDemand += instance.get_customer_demand(
                        individual.routes[firstRoute][firstNode]);
                    int secondPrefixDemand = 0;
                    for (int secondNode = 0;
                         secondNode < secondLength - 1;
                         ++secondNode) {
                        secondPrefixDemand += instance.get_customer_demand(
                            individual.routes[secondRoute][secondNode]);
                        const bool leavesRoutesUnchanged =
                            (firstNode == 0 && secondNode == 0)
                            || (firstNode == firstLength - 2
                                && secondNode == secondLength - 2);
                        if (leavesRoutesUnchanged) {
                            continue;
                        }

                        const int firstNewDemand = firstPrefixDemand
                            + individual.demand_sum[secondRoute] - secondPrefixDemand;
                        const int secondNewDemand = secondPrefixDemand
                            + individual.demand_sum[firstRoute] - firstPrefixDemand;
                        if (firstNewDemand > instance.maxC
                            || secondNewDemand > instance.maxC) {
                            continue;
                        }

                        const double oldCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[firstRoute][firstNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode],
                                individual.routes[secondRoute][secondNode + 1]);
                        const double newCost =
                            instance.get_distance(
                                individual.routes[firstRoute][firstNode],
                                individual.routes[secondRoute][secondNode + 1])
                            + instance.get_distance(
                                individual.routes[secondRoute][secondNode],
                                individual.routes[firstRoute][firstNode + 1]);
                        const double improvement = oldCost - newCost;
                        if (improvement <= kImprovementTolerance) {
                            continue;
                        }

                        const std::vector<int> firstOriginal(
                            individual.routes[firstRoute],
                            individual.routes[firstRoute] + firstLength);
                        const std::vector<int> secondOriginal(
                            individual.routes[secondRoute],
                            individual.routes[secondRoute] + secondLength);
                        std::vector<int> firstNew(
                            firstOriginal.begin(),
                            firstOriginal.begin() + firstNode + 1);
                        firstNew.insert(
                            firstNew.end(),
                            secondOriginal.begin() + secondNode + 1,
                            secondOriginal.end());
                        std::vector<int> secondNew(
                            secondOriginal.begin(),
                            secondOriginal.begin() + secondNode + 1);
                        secondNew.insert(
                            secondNew.end(),
                            firstOriginal.begin() + firstNode + 1,
                            firstOriginal.end());

                        replace_route(individual, firstRoute, firstNew, firstNewDemand);
                        replace_route(individual, secondRoute, secondNew, secondNewDemand);
                        individual.set_upper_cost(individual.get_upper_cost() - improvement);
                        if (individual.node_num[firstRoute] == 2) {
                            remove_empty_route(individual, firstRoute);
                        } else if (individual.node_num[secondRoute] == 2) {
                            remove_empty_route(individual, secondRoute);
                        }
                        improved = true;
                        moveApplied = true;
                        break;
                    }
                }
            }
        }
    } while (moveApplied && individual.route_num > 1);

    return improved;
}

const std::vector<int>& shuffled_route_order(
    int routeCount,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    auto& routeOrder = workspace.routeOrder;
    routeOrder.resize(routeCount);
    std::iota(routeOrder.begin(), routeOrder.end(), 0);
    std::shuffle(routeOrder.begin(), routeOrder.end(), randomEngine);
    return routeOrder;
}

bool improve_with_node_shift_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::NodeShift;
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
        if (route_failure_is_cached(
                localSearchOperator,
                routeIndex,
                workspace)) {
            continue;
        }
        int* route = individual.routes[routeIndex];
        const int length = individual.node_num[routeIndex];
        if (length <= 4) {
            cache_route_failure(
                localSearchOperator,
                routeIndex,
                workspace);
            continue;
        }

        for (int fromIndex = 1; fromIndex < length - 1; ++fromIndex) {
            for (int toIndex = 1; toIndex < length - 1; ++toIndex) {
                double improvement = 0.0;
                if (fromIndex < toIndex) {
                    const double oldCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex], route[toIndex + 1]);
                    const double newCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[toIndex + 1]);
                    improvement = oldCost - newCost;
                } else if (fromIndex > toIndex) {
                    const double oldCost =
                        instance.get_distance(route[fromIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[fromIndex + 1])
                        + instance.get_distance(route[toIndex - 1], route[toIndex]);
                    const double newCost =
                        instance.get_distance(route[toIndex - 1], route[fromIndex])
                        + instance.get_distance(route[fromIndex], route[toIndex])
                        + instance.get_distance(route[fromIndex - 1], route[fromIndex + 1]);
                    improvement = oldCost - newCost;
                }

                if (improvement <= kImprovementTolerance) {
                    continue;
                }
                move_node(route, fromIndex, toIndex);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                record_changed_route(routeIndex, workspace);
                return true;
            }
        }
        cache_route_failure(
            localSearchOperator,
            routeIndex,
            workspace);
    }
    return false;
}

bool improve_with_inter_route_relocate_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::InterRouteRelocate;
    auto& routePairPool = synchronize_active_route_pairs(
        localSearchOperator,
        individual.route_num,
        true,
        workspace);
    while (!routePairPool.pairs.empty()) {
        const std::size_t pairIndex =
            select_random_active_route_pair(
                routePairPool,
                randomEngine);
        const auto [sourceRoute, targetRoute] =
            routePairPool.pairs[pairIndex];
        const int sourceLength = individual.node_num[sourceRoute];
        const int targetLength = individual.node_num[targetRoute];
        for (int sourceNode = 1; sourceNode < sourceLength - 1; ++sourceNode) {
            const int customer = individual.routes[sourceRoute][sourceNode];
            const int customerDemand = instance.get_customer_demand(customer);
            if (individual.demand_sum[targetRoute] + customerDemand > instance.maxC) {
                continue;
            }

            const double sourceImprovement =
                instance.get_distance(
                    individual.routes[sourceRoute][sourceNode - 1],
                    customer)
                + instance.get_distance(
                    customer,
                    individual.routes[sourceRoute][sourceNode + 1])
                - instance.get_distance(
                    individual.routes[sourceRoute][sourceNode - 1],
                    individual.routes[sourceRoute][sourceNode + 1]);
            for (int targetEdge = 0; targetEdge < targetLength - 1; ++targetEdge) {
                const int targetFrom = individual.routes[targetRoute][targetEdge];
                const int targetTo = individual.routes[targetRoute][targetEdge + 1];
                const double improvement = sourceImprovement
                    + instance.get_distance(targetFrom, targetTo)
                    - instance.get_distance(targetFrom, customer)
                    - instance.get_distance(customer, targetTo);
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                relocate_customer(
                    individual,
                    sourceRoute,
                    sourceNode,
                    targetRoute,
                    targetEdge,
                    customer,
                    customerDemand);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                if (individual.node_num[sourceRoute] == 2) {
                    remove_empty_route(individual, sourceRoute);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        sourceRoute,
                        targetRoute,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            routePairPool,
            pairIndex,
            workspace);
    }
    return false;
}

bool improve_with_intra_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::IntraRouteSwap;
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
        if (route_failure_is_cached(
                localSearchOperator,
                routeIndex,
                workspace)) {
            continue;
        }
        int* route = individual.routes[routeIndex];
        const int length = individual.node_num[routeIndex];
        if (length < 5) {
            cache_route_failure(
                localSearchOperator,
                routeIndex,
                workspace);
            continue;
        }

        for (int firstNode = 1; firstNode < length - 2; ++firstNode) {
            for (int secondNode = firstNode + 2;
                 secondNode < length - 1;
                 ++secondNode) {
                const double oldCost =
                    instance.get_distance(route[firstNode - 1], route[firstNode])
                    + instance.get_distance(route[firstNode], route[firstNode + 1])
                    + instance.get_distance(route[secondNode - 1], route[secondNode])
                    + instance.get_distance(route[secondNode], route[secondNode + 1]);
                const double newCost =
                    instance.get_distance(route[firstNode - 1], route[secondNode])
                    + instance.get_distance(route[secondNode], route[firstNode + 1])
                    + instance.get_distance(route[secondNode - 1], route[firstNode])
                    + instance.get_distance(route[firstNode], route[secondNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::swap(route[firstNode], route[secondNode]);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                record_changed_route(routeIndex, workspace);
                return true;
            }
        }
        cache_route_failure(
            localSearchOperator,
            routeIndex,
            workspace);
    }
    return false;
}

bool improve_with_inter_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::InterRouteSwap;
    auto& routePairPool = synchronize_active_route_pairs(
        localSearchOperator,
        individual.route_num,
        false,
        workspace);
    while (!routePairPool.pairs.empty()) {
        const std::size_t pairIndex =
            select_random_active_route_pair(
                routePairPool,
                randomEngine);
        const auto [firstRoute, secondRoute] =
            routePairPool.pairs[pairIndex];
        for (int firstNode = 1;
             firstNode < individual.node_num[firstRoute] - 1;
             ++firstNode) {
            const int firstCustomer = individual.routes[firstRoute][firstNode];
            const int firstDemand = instance.get_customer_demand(firstCustomer);
            for (int secondNode = 1;
                 secondNode < individual.node_num[secondRoute] - 1;
                 ++secondNode) {
                const int secondCustomer = individual.routes[secondRoute][secondNode];
                const int secondDemand = instance.get_customer_demand(secondCustomer);
                if (individual.demand_sum[firstRoute] - firstDemand + secondDemand > instance.maxC
                    || individual.demand_sum[secondRoute] - secondDemand + firstDemand > instance.maxC) {
                    continue;
                }

                const double oldCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode - 1],
                        firstCustomer)
                    + instance.get_distance(
                        firstCustomer,
                        individual.routes[firstRoute][firstNode + 1])
                    + instance.get_distance(
                        individual.routes[secondRoute][secondNode - 1],
                        secondCustomer)
                    + instance.get_distance(
                        secondCustomer,
                        individual.routes[secondRoute][secondNode + 1]);
                const double newCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode - 1],
                        secondCustomer)
                    + instance.get_distance(
                        secondCustomer,
                        individual.routes[firstRoute][firstNode + 1])
                    + instance.get_distance(
                        individual.routes[secondRoute][secondNode - 1],
                        firstCustomer)
                    + instance.get_distance(
                        firstCustomer,
                        individual.routes[secondRoute][secondNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::swap(
                    individual.routes[firstRoute][firstNode],
                    individual.routes[secondRoute][secondNode]);
                individual.demand_sum[firstRoute] =
                    individual.demand_sum[firstRoute] - firstDemand + secondDemand;
                individual.demand_sum[secondRoute] =
                    individual.demand_sum[secondRoute] - secondDemand + firstDemand;
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                record_changed_route_pair(
                    firstRoute,
                    secondRoute,
                    workspace);
                return true;
            }
        }
        discard_active_route_pair(
            routePairPool,
            pairIndex,
            workspace);
    }
    return false;
}

struct BestInsertionAfterRemoval {
    double cost = std::numeric_limits<double>::infinity();
    int insertionIndex = -1;
};

bool insertion_precedes(
    double cost,
    int insertionIndex,
    const LocalSearchWorkspace::InsertionCandidate& candidate) {
    return cost < candidate.cost
        || (cost == candidate.cost
            && insertionIndex < candidate.insertionIndex);
}

void insert_into_top_three(
    double cost,
    int insertionIndex,
    LocalSearchWorkspace::TopThreeInsertions& topThree) {
    for (std::size_t rank = 0; rank < topThree.size(); ++rank) {
        if (!insertion_precedes(cost, insertionIndex, topThree[rank])) {
            continue;
        }

        for (std::size_t shiftedRank = topThree.size() - 1;
             shiftedRank > rank;
             --shiftedRank) {
            topThree[shiftedRank] = topThree[shiftedRank - 1];
        }
        topThree[rank] = {cost, insertionIndex};
        return;
    }
}

void cache_top_three_insertions(
    const int* customerRoute,
    int customerRouteLength,
    const int* targetRoute,
    int targetRouteLength,
    Case& instance,
    std::vector<LocalSearchWorkspace::TopThreeInsertions>& cache) {
    const int customerCount = customerRouteLength - 2;
    cache.resize(static_cast<std::size_t>(customerCount));
    for (int customerIndex = 1;
         customerIndex < customerRouteLength - 1;
         ++customerIndex) {
        auto& topThree = cache[static_cast<std::size_t>(customerIndex - 1)];
        for (auto& candidate : topThree) {
            candidate = {
                std::numeric_limits<double>::infinity(),
                -1};
        }

        const int customer = customerRoute[customerIndex];
        for (int insertionIndex = 1;
             insertionIndex < targetRouteLength;
             ++insertionIndex) {
            const int from = targetRoute[insertionIndex - 1];
            const int to = targetRoute[insertionIndex];
            const double insertionCost =
                instance.get_distance(from, customer)
                + instance.get_distance(customer, to)
                - instance.get_distance(from, to);
            insert_into_top_three(
                insertionCost,
                insertionIndex,
                topThree);
        }
    }
}

void cache_removal_costs(
    const int* route,
    int length,
    Case& instance,
    std::vector<double>& removalCosts) {
    removalCosts.resize(static_cast<std::size_t>(length - 2));
    for (int removedIndex = 1;
         removedIndex < length - 1;
         ++removedIndex) {
        const int customer = route[removedIndex];
        removalCosts[static_cast<std::size_t>(removedIndex - 1)] =
            instance.get_distance(
                route[removedIndex - 1],
                route[removedIndex + 1])
            - instance.get_distance(
                route[removedIndex - 1],
                customer)
            - instance.get_distance(
                customer,
                route[removedIndex + 1]);
    }
}

void consider_insertion(
    double cost,
    int insertionIndex,
    BestInsertionAfterRemoval& best) {
    if (cost < best.cost
        || (cost == best.cost
            && insertionIndex < best.insertionIndex)) {
        best.cost = cost;
        best.insertionIndex = insertionIndex;
    }
}

BestInsertionAfterRemoval best_insertion_after_removal(
    const LocalSearchWorkspace::TopThreeInsertions& topThree,
    const int* route,
    int removedIndex,
    int customer,
    Case& instance) {
    BestInsertionAfterRemoval best;

    // Removing one customer invalidates only its two incident insertion edges,
    // so top-3 retains the best unaffected edge whenever one exists.
    for (const auto& candidate : topThree) {
        if (candidate.insertionIndex < 0
            || candidate.insertionIndex == removedIndex
            || candidate.insertionIndex == removedIndex + 1) {
            continue;
        }
        const int reducedInsertionIndex =
            candidate.insertionIndex < removedIndex
            ? candidate.insertionIndex
            : candidate.insertionIndex - 1;
        consider_insertion(
            candidate.cost,
            reducedInsertionIndex,
            best);
    }

    const int from = route[removedIndex - 1];
    const int to = route[removedIndex + 1];
    const double insertionCost =
        instance.get_distance(from, customer)
        + instance.get_distance(customer, to)
        - instance.get_distance(from, to);
    consider_insertion(
        insertionCost,
        removedIndex,
        best);
    return best;
}

void build_route_after_swap_star(
    const int* route,
    int length,
    int removedIndex,
    int insertedCustomer,
    int insertionIndex,
    std::vector<int>& result) {
    result.clear();
    result.reserve(static_cast<std::size_t>(length));
    for (int nodeIndex = 0; nodeIndex < length; ++nodeIndex) {
        if (nodeIndex != removedIndex) {
            result.push_back(route[nodeIndex]);
        }
    }
    result.insert(result.begin() + insertionIndex, insertedCustomer);
}

bool improve_with_swap_star_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::SwapStar;
    auto& routePairPool = synchronize_active_route_pairs(
        localSearchOperator,
        individual.route_num,
        false,
        workspace);
    while (!routePairPool.pairs.empty()) {
        const std::size_t pairIndex =
            select_random_active_route_pair(
                routePairPool,
                randomEngine);
        const auto [firstRoute, secondRoute] =
            routePairPool.pairs[pairIndex];
        const int firstLength = individual.node_num[firstRoute];
        const int secondLength = individual.node_num[secondRoute];
        cache_top_three_insertions(
            individual.routes[firstRoute],
            firstLength,
            individual.routes[secondRoute],
            secondLength,
            instance,
            workspace.firstCustomersIntoSecond);
        cache_top_three_insertions(
            individual.routes[secondRoute],
            secondLength,
            individual.routes[firstRoute],
            firstLength,
            instance,
            workspace.secondCustomersIntoFirst);
        cache_removal_costs(
            individual.routes[firstRoute],
            firstLength,
            instance,
            workspace.firstRemovalCosts);
        cache_removal_costs(
            individual.routes[secondRoute],
            secondLength,
            instance,
            workspace.secondRemovalCosts);

        double bestImprovement = 0.0;
        int bestFirstNode = -1;
        int bestSecondNode = -1;
        int bestFirstInsertion = -1;
        int bestSecondInsertion = -1;

        for (int firstNode = 1; firstNode < firstLength - 1; ++firstNode) {
            const int firstCustomer = individual.routes[firstRoute][firstNode];
            const int firstDemand = instance.get_customer_demand(firstCustomer);
            const double firstRemovalCost =
                workspace.firstRemovalCosts[
                    static_cast<std::size_t>(firstNode - 1)];

            for (int secondNode = 1;
                 secondNode < secondLength - 1;
                 ++secondNode) {
                const int secondCustomer =
                    individual.routes[secondRoute][secondNode];
                const int secondDemand =
                    instance.get_customer_demand(secondCustomer);
                const int firstNewDemand =
                    individual.demand_sum[firstRoute] - firstDemand + secondDemand;
                const int secondNewDemand =
                    individual.demand_sum[secondRoute] - secondDemand + firstDemand;
                if (firstNewDemand > instance.maxC
                    || secondNewDemand > instance.maxC) {
                    continue;
                }

                const double secondRemovalCost =
                    workspace.secondRemovalCosts[
                        static_cast<std::size_t>(secondNode - 1)];
                const BestInsertionAfterRemoval secondIntoFirst =
                    best_insertion_after_removal(
                        workspace.secondCustomersIntoFirst[
                            static_cast<std::size_t>(secondNode - 1)],
                        individual.routes[firstRoute],
                        firstNode,
                        secondCustomer,
                        instance);
                const BestInsertionAfterRemoval firstIntoSecond =
                    best_insertion_after_removal(
                        workspace.firstCustomersIntoSecond[
                            static_cast<std::size_t>(firstNode - 1)],
                        individual.routes[secondRoute],
                        secondNode,
                        firstCustomer,
                        instance);
                const double improvement = -(
                    firstRemovalCost
                    + secondIntoFirst.cost
                    + secondRemovalCost
                    + firstIntoSecond.cost);
                if (improvement > bestImprovement + kImprovementTolerance) {
                    bestImprovement = improvement;
                    bestFirstNode = firstNode;
                    bestSecondNode = secondNode;
                    bestFirstInsertion = secondIntoFirst.insertionIndex;
                    bestSecondInsertion = firstIntoSecond.insertionIndex;
                }
            }
        }

        if (bestImprovement <= kImprovementTolerance) {
            discard_active_route_pair(
                routePairPool,
                pairIndex,
                workspace);
            continue;
        }

        const int firstCustomer =
            individual.routes[firstRoute][bestFirstNode];
        const int secondCustomer =
            individual.routes[secondRoute][bestSecondNode];
        const int firstNewDemand =
            individual.demand_sum[firstRoute]
            - instance.get_customer_demand(firstCustomer)
            + instance.get_customer_demand(secondCustomer);
        const int secondNewDemand =
            individual.demand_sum[secondRoute]
            - instance.get_customer_demand(secondCustomer)
            + instance.get_customer_demand(firstCustomer);
        build_route_after_swap_star(
            individual.routes[firstRoute],
            firstLength,
            bestFirstNode,
            secondCustomer,
            bestFirstInsertion,
            workspace.firstRouteBuffer);
        build_route_after_swap_star(
            individual.routes[secondRoute],
            secondLength,
            bestSecondNode,
            firstCustomer,
            bestSecondInsertion,
            workspace.secondRouteBuffer);
        replace_route(
            individual,
            firstRoute,
            workspace.firstRouteBuffer,
            firstNewDemand);
        replace_route(
            individual,
            secondRoute,
            workspace.secondRouteBuffer,
            secondNewDemand);
        individual.set_upper_cost(
            individual.get_upper_cost() - bestImprovement);
        record_changed_route_pair(
            firstRoute,
            secondRoute,
            workspace);
        return true;
    }
    return false;
}

bool improve_with_two_opt_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::TwoOpt;
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
        if (route_failure_is_cached(
                localSearchOperator,
                routeIndex,
                workspace)) {
            continue;
        }
        int* route = individual.routes[routeIndex];
        const int length = individual.node_num[routeIndex];
        for (int firstNode = 1; firstNode < length - 2; ++firstNode) {
            for (int secondNode = firstNode + 1;
                 secondNode < length - 1;
                 ++secondNode) {
                const double oldCost =
                    instance.get_distance(route[firstNode - 1], route[firstNode])
                    + instance.get_distance(route[secondNode], route[secondNode + 1]);
                const double newCost =
                    instance.get_distance(route[firstNode - 1], route[secondNode])
                    + instance.get_distance(route[firstNode], route[secondNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::reverse(route + firstNode, route + secondNode + 1);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                record_changed_route(routeIndex, workspace);
                return true;
            }
        }
        cache_route_failure(
            localSearchOperator,
            routeIndex,
            workspace);
    }
    return false;
}

void build_two_opt_star_head_to_head_routes(
    const int* firstRoute,
    int firstLength,
    int firstNode,
    const int* secondRoute,
    int secondLength,
    int secondNode,
    std::vector<int>& firstResult,
    std::vector<int>& secondResult) {
    firstResult.clear();
    firstResult.reserve(
        static_cast<std::size_t>(firstNode + secondNode + 2));
    firstResult.insert(
        firstResult.end(),
        firstRoute,
        firstRoute + firstNode + 1);
    for (int node = secondNode; node >= 0; --node) {
        firstResult.push_back(secondRoute[node]);
    }

    secondResult.clear();
    secondResult.reserve(static_cast<std::size_t>(
        firstLength - firstNode - 1
        + secondLength - secondNode - 1));
    for (int node = firstLength - 1; node >= firstNode + 1; --node) {
        secondResult.push_back(firstRoute[node]);
    }
    secondResult.insert(
        secondResult.end(),
        secondRoute + secondNode + 1,
        secondRoute + secondLength);
}

void build_two_opt_star_head_to_tail_routes(
    const int* firstRoute,
    int firstLength,
    int firstNode,
    const int* secondRoute,
    int secondLength,
    int secondNode,
    std::vector<int>& firstResult,
    std::vector<int>& secondResult) {
    firstResult.clear();
    firstResult.reserve(static_cast<std::size_t>(
        firstNode + 1
        + secondLength - secondNode - 1));
    firstResult.insert(
        firstResult.end(),
        firstRoute,
        firstRoute + firstNode + 1);
    firstResult.insert(
        firstResult.end(),
        secondRoute + secondNode + 1,
        secondRoute + secondLength);

    secondResult.clear();
    secondResult.reserve(static_cast<std::size_t>(
        secondNode + 1
        + firstLength - firstNode - 1));
    secondResult.insert(
        secondResult.end(),
        secondRoute,
        secondRoute + secondNode + 1);
    secondResult.insert(
        secondResult.end(),
        firstRoute + firstNode + 1,
        firstRoute + firstLength);
}

bool improve_with_two_opt_star_head_to_head_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::TwoOptStarHeadToHead;
    auto& routePairPool = synchronize_active_route_pairs(
        localSearchOperator,
        individual.route_num,
        false,
        workspace);
    while (!routePairPool.pairs.empty()) {
        const std::size_t pairIndex =
            select_random_active_route_pair(
                routePairPool,
                randomEngine);
        const auto [firstRoute, secondRoute] =
            routePairPool.pairs[pairIndex];
        const int firstLength = individual.node_num[firstRoute];
        const int secondLength = individual.node_num[secondRoute];
        int firstPrefixDemand = 0;
        for (int firstNode = 0; firstNode < firstLength - 1; ++firstNode) {
            firstPrefixDemand += instance.get_customer_demand(
                individual.routes[firstRoute][firstNode]);
            int secondPrefixDemand = 0;
            for (int secondNode = 0; secondNode < secondLength - 1; ++secondNode) {
                secondPrefixDemand += instance.get_customer_demand(
                    individual.routes[secondRoute][secondNode]);
                const bool leavesRoutesUnchanged =
                    (firstNode == firstLength - 2 && secondNode == 0)
                    || (firstNode == 0 && secondNode == secondLength - 2);
                if (leavesRoutesUnchanged) {
                    continue;
                }

                const int firstNewDemand = firstPrefixDemand + secondPrefixDemand;
                const int secondNewDemand =
                    individual.demand_sum[firstRoute] - firstPrefixDemand
                    + individual.demand_sum[secondRoute] - secondPrefixDemand;
                if (firstNewDemand > instance.maxC
                    || secondNewDemand > instance.maxC) {
                    continue;
                }

                const double oldCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode],
                        individual.routes[firstRoute][firstNode + 1])
                    + instance.get_distance(
                        individual.routes[secondRoute][secondNode],
                        individual.routes[secondRoute][secondNode + 1]);
                const double newCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode],
                        individual.routes[secondRoute][secondNode])
                    + instance.get_distance(
                        individual.routes[firstRoute][firstNode + 1],
                        individual.routes[secondRoute][secondNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                build_two_opt_star_head_to_head_routes(
                    individual.routes[firstRoute],
                    firstLength,
                    firstNode,
                    individual.routes[secondRoute],
                    secondLength,
                    secondNode,
                    workspace.firstRouteBuffer,
                    workspace.secondRouteBuffer);
                replace_route(
                    individual,
                    firstRoute,
                    workspace.firstRouteBuffer,
                    firstNewDemand);
                replace_route(
                    individual,
                    secondRoute,
                    workspace.secondRouteBuffer,
                    secondNewDemand);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                if (individual.node_num[firstRoute] == 2) {
                    remove_empty_route(individual, firstRoute);
                    record_all_routes_changed(workspace);
                } else if (individual.node_num[secondRoute] == 2) {
                    remove_empty_route(individual, secondRoute);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        firstRoute,
                        secondRoute,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            routePairPool,
            pairIndex,
            workspace);
    }
    return false;
}

bool improve_with_two_opt_star_head_to_tail_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    constexpr LocalSearchOperator localSearchOperator =
        LocalSearchOperator::TwoOptStarHeadToTail;
    auto& routePairPool = synchronize_active_route_pairs(
        localSearchOperator,
        individual.route_num,
        false,
        workspace);
    while (!routePairPool.pairs.empty()) {
        const std::size_t pairIndex =
            select_random_active_route_pair(
                routePairPool,
                randomEngine);
        const auto [firstRoute, secondRoute] =
            routePairPool.pairs[pairIndex];
        const int firstLength = individual.node_num[firstRoute];
        const int secondLength = individual.node_num[secondRoute];
        int firstPrefixDemand = 0;
        for (int firstNode = 0; firstNode < firstLength - 1; ++firstNode) {
            firstPrefixDemand += instance.get_customer_demand(
                individual.routes[firstRoute][firstNode]);
            int secondPrefixDemand = 0;
            for (int secondNode = 0; secondNode < secondLength - 1; ++secondNode) {
                secondPrefixDemand += instance.get_customer_demand(
                    individual.routes[secondRoute][secondNode]);
                const bool leavesRoutesUnchanged =
                    (firstNode == 0 && secondNode == 0)
                    || (firstNode == firstLength - 2
                        && secondNode == secondLength - 2);
                if (leavesRoutesUnchanged) {
                    continue;
                }

                const int firstNewDemand = firstPrefixDemand
                    + individual.demand_sum[secondRoute] - secondPrefixDemand;
                const int secondNewDemand = secondPrefixDemand
                    + individual.demand_sum[firstRoute] - firstPrefixDemand;
                if (firstNewDemand > instance.maxC
                    || secondNewDemand > instance.maxC) {
                    continue;
                }

                const double oldCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode],
                        individual.routes[firstRoute][firstNode + 1])
                    + instance.get_distance(
                        individual.routes[secondRoute][secondNode],
                        individual.routes[secondRoute][secondNode + 1]);
                const double newCost =
                    instance.get_distance(
                        individual.routes[firstRoute][firstNode],
                        individual.routes[secondRoute][secondNode + 1])
                    + instance.get_distance(
                        individual.routes[secondRoute][secondNode],
                        individual.routes[firstRoute][firstNode + 1]);
                const double improvement = oldCost - newCost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                build_two_opt_star_head_to_tail_routes(
                    individual.routes[firstRoute],
                    firstLength,
                    firstNode,
                    individual.routes[secondRoute],
                    secondLength,
                    secondNode,
                    workspace.firstRouteBuffer,
                    workspace.secondRouteBuffer);
                replace_route(
                    individual,
                    firstRoute,
                    workspace.firstRouteBuffer,
                    firstNewDemand);
                replace_route(
                    individual,
                    secondRoute,
                    workspace.secondRouteBuffer,
                    secondNewDemand);
                individual.set_upper_cost(individual.get_upper_cost() - improvement);
                if (individual.node_num[firstRoute] == 2) {
                    remove_empty_route(individual, firstRoute);
                    record_all_routes_changed(workspace);
                } else if (individual.node_num[secondRoute] == 2) {
                    remove_empty_route(individual, secondRoute);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        firstRoute,
                        secondRoute,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            routePairPool,
            pairIndex,
            workspace);
    }
    return false;
}

bool improve_with_neighborhood_one_move(
    Neighborhood neighborhood,
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchWorkspace& workspace) {
    switch (neighborhood) {
        case Neighborhood::TwoOpt:
            return improve_with_two_opt_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::NodeShift:
            return improve_with_node_shift_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::InterRouteRelocate:
            return improve_with_inter_route_relocate_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::InterRouteSwap:
            return improve_with_inter_route_swap_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::SwapStar:
            return improve_with_swap_star_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::IntraRouteSwap:
            return improve_with_intra_route_swap_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::TwoOptStarHeadToHead:
            return improve_with_two_opt_star_head_to_head_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::TwoOptStarHeadToTail:
            return improve_with_two_opt_star_head_to_tail_one_move(
                individual,
                instance,
                randomEngine,
                workspace);
        case Neighborhood::TwoOptStar:
            return false;
    }
    return false;
}

bool improve_with_neighborhood(
    Neighborhood neighborhood,
    Individual& individual,
    Case& instance) {
    switch (neighborhood) {
        case Neighborhood::TwoOpt:
            return improve_with_two_opt(individual, instance);
        case Neighborhood::TwoOptStar:
            return improve_with_two_opt_star(individual, instance);
        case Neighborhood::NodeShift:
            return improve_with_node_shift(individual, instance);
        case Neighborhood::InterRouteRelocate:
            return improve_with_inter_route_relocate(individual, instance);
        case Neighborhood::InterRouteSwap:
            return improve_with_inter_route_swap(individual, instance);
        case Neighborhood::SwapStar:
            return false;
        case Neighborhood::IntraRouteSwap:
            return improve_with_intra_route_swap(individual, instance);
        case Neighborhood::TwoOptStarHeadToHead:
            return improve_with_two_opt_star_head_to_head(individual, instance);
        case Neighborhood::TwoOptStarHeadToTail:
            return improve_with_two_opt_star_head_to_tail(individual, instance);
    }
    return false;
}

template <std::size_t NeighborhoodCount>
void improve_with_rvnd(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    const std::array<Neighborhood, NeighborhoodCount>& neighborhoods) {
    std::vector<Neighborhood> activeNeighborhoods(
        neighborhoods.begin(),
        neighborhoods.end());

    while (!activeNeighborhoods.empty()) {
        std::uniform_int_distribution<std::size_t> selectNeighborhood(
            0,
            activeNeighborhoods.size() - 1);
        const std::size_t selectedIndex = selectNeighborhood(randomEngine);
        const bool improved = improve_with_neighborhood(
            activeNeighborhoods[selectedIndex],
            individual,
            instance);

        if (improved) {
            activeNeighborhoods.assign(neighborhoods.begin(), neighborhoods.end());
        } else {
            activeNeighborhoods.erase(activeNeighborhoods.begin() + selectedIndex);
        }
    }
}

template <std::size_t NeighborhoodCount>
LocalSearchResult improve_with_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    const std::array<Neighborhood, NeighborhoodCount>& neighborhoods,
    int moveLimit,
    LocalSearchWorkspace& workspace,
    double gammaUpperBound) {
    const double upperCostBefore = individual.get_upper_cost();
    const std::uint64_t distanceCallsBefore =
        instance.get_distance_calls();
    LocalSearchResult result;
    result.moveLimit = moveLimit;
    if (moveLimit == 0) {
        return result;
    }

    begin_failure_cache_session(individual.route_num, workspace);
    std::vector<Neighborhood> activeNeighborhoods(
        neighborhoods.begin(),
        neighborhoods.end());

    while (!activeNeighborhoods.empty()) {
        if (moveLimit >= 0 && result.acceptedMoves >= moveLimit) {
            break;
        }

        std::uniform_int_distribution<std::size_t> selectNeighborhood(
            0,
            activeNeighborhoods.size() - 1);
        const std::size_t selectedIndex = selectNeighborhood(randomEngine);
        const LocalSearchOperator selectedOperator =
            local_search_operator(activeNeighborhoods[selectedIndex]);
        if (selectedOperator == LocalSearchOperator::Count) {
            throw std::logic_error(
                "RVND OneMove selected an untracked neighborhood");
        }
        const std::size_t operatorIndex =
            static_cast<std::size_t>(selectedOperator);
        LocalSearchOperatorStats& operatorStats =
            result.operatorStats[operatorIndex];
        const std::uint64_t operatorDistanceCallsBefore =
            instance.get_distance_calls();
        const double operatorUpperCostBefore = individual.get_upper_cost();
        const bool outsideGammaBefore =
            operatorUpperCostBefore > gammaUpperBound;

        ++result.neighborhoodCalls;
        ++operatorStats.calls;
        reset_move_impact(workspace);
        const bool improved = improve_with_neighborhood_one_move(
            activeNeighborhoods[selectedIndex],
            individual,
            instance,
            randomEngine,
            workspace);
        operatorStats.distanceCalls +=
            instance.get_distance_calls()
            - operatorDistanceCallsBefore;

        if (improved) {
            invalidate_failure_cache_after_move(
                individual.route_num,
                workspace);
            ++result.acceptedMoves;
            ++operatorStats.accepts;
            operatorStats.upperGain +=
                operatorUpperCostBefore - individual.get_upper_cost();
            if (outsideGammaBefore
                && individual.get_upper_cost() <= gammaUpperBound) {
                ++operatorStats.gammaCrosses;
            }
            activeNeighborhoods.assign(neighborhoods.begin(), neighborhoods.end());
        } else {
            activeNeighborhoods.erase(activeNeighborhoods.begin() + selectedIndex);
        }
    }

    result.distanceCallsUsed =
        instance.get_distance_calls() - distanceCallsBefore;
    result.relativeUpperImprovement = upperCostBefore > 0.0
        ? (upperCostBefore - individual.get_upper_cost()) / upperCostBefore
        : 0.0;
    result.reachedLocalOptimum = activeNeighborhoods.empty();
    individual.set_upper_locally_optimal(result.reachedLocalOptimum);
    return result;
}

int scaled_move_limit_for_intensity(
    const Individual& individual,
    const Case& instance,
    LocalSearchIntensity intensity) {
    const int solutionScale = instance.customerNumber + individual.route_num;
    switch (intensity) {
        case LocalSearchIntensity::Skip:
            return 0;
        case LocalSearchIntensity::Weak:
            return std::max(
                1,
                static_cast<int>(std::ceil(kWeakMoveFraction * solutionScale)));
        case LocalSearchIntensity::Medium:
            return std::max(
                1,
                static_cast<int>(std::ceil(kMediumMoveFraction * solutionScale)));
        case LocalSearchIntensity::Strong:
            return -1;
    }
    return -1;
}

}  // namespace

const char* Leader::operator_name(
    LocalSearchOperator localSearchOperator) {
    switch (localSearchOperator) {
        case LocalSearchOperator::NodeShift:
            return "node_shift";
        case LocalSearchOperator::InterRouteRelocate:
            return "inter_route_relocate";
        case LocalSearchOperator::IntraRouteSwap:
            return "intra_route_swap";
        case LocalSearchOperator::InterRouteSwap:
            return "inter_route_swap";
        case LocalSearchOperator::SwapStar:
            return "swap_star";
        case LocalSearchOperator::TwoOpt:
            return "two_opt";
        case LocalSearchOperator::TwoOptStarHeadToHead:
            return "two_opt_star_head_to_head";
        case LocalSearchOperator::TwoOptStarHeadToTail:
            return "two_opt_star_head_to_tail";
        case LocalSearchOperator::Count:
            return "unknown";
    }
    return "unknown";
}

void Leader::improve_with_three_neighborhood_vnd(Individual& individual, Case& instance) {
    bool improvedInRound;
    do {
        improvedInRound = false;
        if (improve_with_two_opt(individual, instance)) {
            improvedInRound = true;
        }
        if (improve_with_two_opt_star(individual, instance)) {
            improvedInRound = true;
        }
        if (improve_with_node_shift(individual, instance)) {
            improvedInRound = true;
        }
    } while (improvedInRound);
}

void Leader::improve_with_three_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kThreeNeighborhoods);
}

void Leader::improve_with_five_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kFiveNeighborhoods);
}

void Leader::improve_with_seven_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kSevenNeighborhoods);
}

void Leader::improve_with_seven_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine) {
    LocalSearchWorkspace workspace;
    improve_with_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        kSevenNeighborhoods,
        -1,
        workspace,
        std::numeric_limits<double>::infinity());
}

LocalSearchResult Leader::improve_with_seven_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchIntensity intensity) {
    LocalSearchWorkspace workspace;
    return improve_with_seven_neighborhood_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        intensity,
        workspace);
}

LocalSearchResult Leader::improve_with_seven_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchIntensity intensity,
    LocalSearchWorkspace& workspace) {
    return improve_with_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        kSevenNeighborhoods,
        scaled_move_limit_for_intensity(individual, instance, intensity),
        workspace,
        std::numeric_limits<double>::infinity());
}

void Leader::improve_with_eight_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine) {
    LocalSearchWorkspace workspace;
    improve_with_eight_neighborhood_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        LocalSearchIntensity::Strong,
        workspace);
}

LocalSearchResult Leader::improve_with_eight_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchIntensity intensity) {
    LocalSearchWorkspace workspace;
    return improve_with_eight_neighborhood_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        intensity,
        workspace);
}

LocalSearchResult Leader::improve_with_eight_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchIntensity intensity,
    LocalSearchWorkspace& workspace) {
    return improve_with_eight_neighborhood_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        intensity,
        workspace,
        std::numeric_limits<double>::infinity());
}

LocalSearchResult Leader::improve_with_eight_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchIntensity intensity,
    LocalSearchWorkspace& workspace,
    double gammaUpperBound) {
    const int moveLimit = scaled_move_limit_for_intensity(
        individual,
        instance,
        intensity);
    LocalSearchResult result;
    result.moveLimit = moveLimit;
    if (moveLimit == 0) {
        return result;
    }

    LocalSearchSession session;
    begin_eight_neighborhood_rvnd_one_move_session(
        individual,
        session,
        workspace);
    return continue_eight_neighborhood_rvnd_one_move_session(
        individual,
        instance,
        randomEngine,
        session,
        moveLimit,
        workspace,
        gammaUpperBound);
}

void Leader::begin_eight_neighborhood_rvnd_one_move_session(
    Individual& individual,
    LocalSearchSession& session,
    LocalSearchWorkspace& workspace) {
    begin_failure_cache_session(individual.route_num, workspace);
    session.initialized = true;
    session.totalAcceptedMoves = 0;
    session.totalNeighborhoodCalls = 0;
    session.activeOperators.assign(
        kEightOperators.begin(),
        kEightOperators.end());
}

LocalSearchResult Leader::continue_eight_neighborhood_rvnd_one_move_session(
    Individual& individual,
    Case& instance,
    std::mt19937& randomEngine,
    LocalSearchSession& session,
    int cumulativeMoveLimit,
    LocalSearchWorkspace& workspace,
    double gammaUpperBound) {
    if (!session.initialized) {
        throw std::logic_error(
            "local-search session must be initialized before continuation");
    }

    const double upperCostBefore = individual.get_upper_cost();
    const std::uint64_t distanceCallsBefore =
        instance.get_distance_calls();
    LocalSearchResult result;
    result.moveLimit = cumulativeMoveLimit;

    while (!session.activeOperators.empty()) {
        if (cumulativeMoveLimit >= 0
            && session.totalAcceptedMoves >= cumulativeMoveLimit) {
            break;
        }

        std::uniform_int_distribution<std::size_t> selectOperator(
            0,
            session.activeOperators.size() - 1);
        const std::size_t selectedIndex = selectOperator(randomEngine);
        const LocalSearchOperator selectedOperator =
            session.activeOperators[selectedIndex];
        const Neighborhood selectedNeighborhood =
            neighborhood_for_operator(selectedOperator);
        LocalSearchOperatorStats& operatorStats =
            result.operatorStats[operator_index(selectedOperator)];
        const std::uint64_t operatorDistanceCallsBefore =
            instance.get_distance_calls();
        const double operatorUpperCostBefore =
            individual.get_upper_cost();
        const bool outsideGammaBefore =
            operatorUpperCostBefore > gammaUpperBound;

        ++result.neighborhoodCalls;
        ++session.totalNeighborhoodCalls;
        ++operatorStats.calls;
        reset_move_impact(workspace);
        const bool improved = improve_with_neighborhood_one_move(
            selectedNeighborhood,
            individual,
            instance,
            randomEngine,
            workspace);
        operatorStats.distanceCalls +=
            instance.get_distance_calls()
            - operatorDistanceCallsBefore;

        if (improved) {
            invalidate_failure_cache_after_move(
                individual.route_num,
                workspace);
            ++result.acceptedMoves;
            ++session.totalAcceptedMoves;
            ++operatorStats.accepts;
            operatorStats.upperGain +=
                operatorUpperCostBefore - individual.get_upper_cost();
            if (outsideGammaBefore
                && individual.get_upper_cost() <= gammaUpperBound) {
                ++operatorStats.gammaCrosses;
            }
            session.activeOperators.assign(
                kEightOperators.begin(),
                kEightOperators.end());
        } else {
            session.activeOperators.erase(
                session.activeOperators.begin() + selectedIndex);
        }
    }

    result.distanceCallsUsed =
        instance.get_distance_calls() - distanceCallsBefore;
    result.relativeUpperImprovement = upperCostBefore > 0.0
        ? (upperCostBefore - individual.get_upper_cost())
            / upperCostBefore
        : 0.0;
    result.reachedLocalOptimum = session.activeOperators.empty();
    individual.set_upper_locally_optimal(result.reachedLocalOptimum);
    return result;
}

int Leader::move_limit_for_intensity(
    const Individual& individual,
    const Case& instance,
    LocalSearchIntensity intensity) {
    return scaled_move_limit_for_intensity(
        individual,
        instance,
        intensity);
}
