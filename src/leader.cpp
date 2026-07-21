#include "../include/leader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../include/case.hpp"
#include "../include/individual.hpp"

namespace {

enum class Neighborhood {
    TwoOpt,
    TwoOptStar,
    NodeShift,
    InterRouteRelocate,
    InterRouteSwap,
    IntraRouteSwap,
    TwoOptStarHeadToHead,
    TwoOptStarHeadToTail,
};

constexpr double kImprovementTolerance = 0.00000001;

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

struct OneMoveWorkspace {
    std::vector<int> routeOrder;
    std::vector<std::pair<int, int>> routePairs;
};

const std::vector<int>& shuffled_route_order(
    int routeCount,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    auto& routeOrder = workspace.routeOrder;
    routeOrder.resize(routeCount);
    std::iota(routeOrder.begin(), routeOrder.end(), 0);
    std::shuffle(routeOrder.begin(), routeOrder.end(), randomEngine);
    return routeOrder;
}

const std::vector<std::pair<int, int>>& shuffled_route_pairs(
    int routeCount,
    bool directed,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    auto& routePairs = workspace.routePairs;
    routePairs.clear();
    const int pairCount = directed
        ? routeCount * (routeCount - 1)
        : routeCount * (routeCount - 1) / 2;
    routePairs.reserve(pairCount);

    for (int firstRoute = 0; firstRoute < routeCount; ++firstRoute) {
        const int secondRouteBegin = directed ? 0 : firstRoute + 1;
        for (int secondRoute = secondRouteBegin;
             secondRoute < routeCount;
             ++secondRoute) {
            if (firstRoute != secondRoute) {
                routePairs.emplace_back(firstRoute, secondRoute);
            }
        }
    }
    std::shuffle(routePairs.begin(), routePairs.end(), randomEngine);
    return routePairs;
}

bool improve_with_node_shift_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
        int* route = individual.routes[routeIndex];
        const int length = individual.node_num[routeIndex];
        if (length <= 4) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_inter_route_relocate_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    const auto& routePairs = shuffled_route_pairs(
        individual.route_num,
        true,
        randomEngine,
        workspace);
    for (const auto& [sourceRoute, targetRoute] : routePairs) {
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
                }
                return true;
            }
        }
    }
    return false;
}

bool improve_with_intra_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
        int* route = individual.routes[routeIndex];
        const int length = individual.node_num[routeIndex];
        if (length < 5) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_inter_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    const auto& routePairs = shuffled_route_pairs(
        individual.route_num,
        false,
        randomEngine,
        workspace);
    for (const auto& [firstRoute, secondRoute] : routePairs) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_two_opt_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    const auto& routeOrder = shuffled_route_order(
        individual.route_num,
        randomEngine,
        workspace);
    for (const int routeIndex : routeOrder) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_two_opt_star_head_to_head_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    const auto& routePairs = shuffled_route_pairs(
        individual.route_num,
        false,
        randomEngine,
        workspace);
    for (const auto& [firstRoute, secondRoute] : routePairs) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_two_opt_star_head_to_tail_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
    if (individual.route_num <= 1) {
        return false;
    }

    const auto& routePairs = shuffled_route_pairs(
        individual.route_num,
        false,
        randomEngine,
        workspace);
    for (const auto& [firstRoute, secondRoute] : routePairs) {
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
                return true;
            }
        }
    }
    return false;
}

bool improve_with_neighborhood_one_move(
    Neighborhood neighborhood,
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    OneMoveWorkspace& workspace) {
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
    std::default_random_engine& randomEngine,
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
void improve_with_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine,
    const std::array<Neighborhood, NeighborhoodCount>& neighborhoods) {
    std::vector<Neighborhood> activeNeighborhoods(
        neighborhoods.begin(),
        neighborhoods.end());
    OneMoveWorkspace workspace;

    while (!activeNeighborhoods.empty()) {
        std::uniform_int_distribution<std::size_t> selectNeighborhood(
            0,
            activeNeighborhoods.size() - 1);
        const std::size_t selectedIndex = selectNeighborhood(randomEngine);
        const bool improved = improve_with_neighborhood_one_move(
            activeNeighborhoods[selectedIndex],
            individual,
            instance,
            randomEngine,
            workspace);

        if (improved) {
            activeNeighborhoods.assign(neighborhoods.begin(), neighborhoods.end());
        } else {
            activeNeighborhoods.erase(activeNeighborhoods.begin() + selectedIndex);
        }
    }
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

void Leader::improve_with_three_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kThreeNeighborhoods);
}

void Leader::improve_with_five_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kFiveNeighborhoods);
}

void Leader::improve_with_seven_neighborhood_rvnd(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine) {
    improve_with_rvnd(individual, instance, randomEngine, kSevenNeighborhoods);
}

void Leader::improve_with_seven_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::default_random_engine& randomEngine) {
    improve_with_rvnd_one_move(
        individual,
        instance,
        randomEngine,
        kSevenNeighborhoods);
}
