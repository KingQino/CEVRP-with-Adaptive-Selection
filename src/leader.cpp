#include "../include/leader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../include/case.hpp"
#include "../include/individual.hpp"

namespace {

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

}  // namespace

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
