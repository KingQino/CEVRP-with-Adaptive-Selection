//
// Created by Yinghao Qin on 16/11/2023.
//

#include <algorithm>
#include <cfloat>
#include <list>
#include <unordered_set>
#include <optional>


#include "../include/utils.hpp"



/****************************************************************/
/*                  Population Initialization                   */
/****************************************************************/

// Prins, C., 2004. A simple and effective evolutionary algorithm for the vehicle routing problem. Computers & operations research, 31(12), pp.1985-2002.
vector<vector<int>> prins_split(const vector<int>& x, Case& instance) {
    int arr_length = instance.customerNumber + 1;
    int* pp = new int[arr_length];
    auto* vv = new double[arr_length];
    memset(pp, 0, sizeof(int) * arr_length);
    vv[0] = 0;
    for (int i = 1; i < arr_length; ++i) {
        vv[i] = DBL_MAX;
    }
    for (int i = 1; i < x.size(); ++i) {
        int load = 0;
        double cost = 0;
        int j = i;
        do
        {
            load += instance.get_customer_demand(x[j]);
            if (i == j) {
                cost = instance.get_distance(instance.depot, x[j]) * 2;
            } else {
                cost -= instance.get_distance(x[j -1], instance.depot);
                cost += instance.get_distance(x[j -1], x[j]);
                cost += instance.get_distance(instance.depot, x[j]);
            }

            if (load <= instance.maxC) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load >instance.maxC));
    }

    vector<vector<int>> all_routes;
    int j = x.size() - 1;
    while (true) {
        int i = pp[j];
        vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
        all_routes.push_back(temp);
        j = i;
        if (i == 0) {
            break;
        }
    }
    delete[] pp;
    delete[] vv;
    return all_routes;
}

// Hien et al., "A greedy search based evolutionary algorithm for electric vehicle routing problem", 2023.
vector<vector<int>> hien_clustering(const Case& instance, std::default_random_engine& rng) {
    vector<int> customers(instance.customers);

    std::shuffle(customers.begin(), customers.end(), rng);

    vector<vector<int>> tours;

    vector<int> tour;
    while (!customers.empty()) {
        tour.clear();

        int anchor = customers.front();
        customers.erase(customers.begin());
        tour.push_back(anchor);
        int cap = instance.get_customer_demand(anchor);

        const vector<int> &nearby_customers = instance.customerClustersMap.at(anchor);
        for (int node: nearby_customers) {
            auto it = find(customers.begin(), customers.end(), node);
            if (it == customers.end()) {
                continue;
            }
            if (cap + instance.get_customer_demand(node) <= instance.maxC) {
                tour.push_back(node);
                cap += instance.get_customer_demand(node);
                customers.erase(it);
            } else {
                tours.push_back(tour);
                break;
            }
        }
    }

    tours.push_back(tour);

    return tours;
}

void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng) {
    vector<int>& lastRoute = routes.back();

    uniform_int_distribution<> distribution(0, lastRoute.size() -1 );
    int customer = lastRoute[distribution(rng)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (int node : lastRoute) {
        cap1 += instance.get_customer_demand(node);
    }

    for (int x : instance.customerClustersMap.at(customer)) {
        if (find(lastRoute.begin(), lastRoute.end(), x) != lastRoute.end()) {
            continue;
        }

        auto route2It = find_if(routes.begin(), routes.end(), [x](const vector<int>& route) {
            return find(route.begin(), route.end(), x) != route.end();
        });

        if (route2It != routes.end()) {
            vector<int>& route2 = *route2It;
            int cap2 = 0;
            for (int node : route2) {
                cap2 += instance.get_customer_demand(node);
            }

            int demandX = instance.get_customer_demand(x);

            if (demandX + cap1 <= instance.maxC && abs((cap1 + demandX) - (cap2 - demandX)) < abs(cap1 - cap2)) {
                route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
                lastRoute.push_back(x);
                cap1 += demandX;
            } else {
                break;
            }
        }
    }
}

vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng) {
    vector<int> a_giant_tour(instance.customers);

    shuffle(a_giant_tour.begin(), a_giant_tour.end(), rng);

    a_giant_tour.insert(a_giant_tour.begin(), instance.depot);

    vector<vector<int>> all_routes = prins_split(a_giant_tour, instance);
    for (auto& route : all_routes) {
        route.insert(route.begin(), 0);
        route.push_back(0);
    }

    return all_routes;
}

vector<vector<int>> routes_constructor_with_hien_method(const Case& instance, std::default_random_engine& rng){
    vector<vector<int>> routes = hien_clustering(instance, rng);
    hien_balancing(routes, instance, rng);

    for (auto& route : routes) {
        route.insert(route.begin(), 0);
        route.push_back(0);
    }

    return routes;
}

// Jia Ya-Hui, et al., "Confidence-Based Ant Colony Optimization for Capacitated Electric Vehicle Routing Problem With Comparison of Different Encoding Schemes", 2022
vector<vector<int>> routes_construct_with_direct_encoding(const Case& instance, std::default_random_engine& rng) {
    vector<int> customers(instance.customers);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance.depot}; // the first route starts from depot 0

    vector<vector<int>> all_routes;
    while(!customers.empty()) {
        vector<int> all_temp;
        for(int i : customers) {
            if(instance.get_customer_demand(i) <= instance.maxC - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        int remain_total_demand = accumulate(customers.begin(), customers.end(), 0, [&](int total, int i) {
            return total + instance.get_customer_demand(i);
        });
        if(remain_total_demand <= instance.maxC * (instance.vehicleNumber - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance.depot); // add depot node into the all_temp
        }

        int cur = route.back();
        uniform_int_distribution<> distribution(0, all_temp.size() - 1);
        int next = all_temp[distribution(rng)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance.depot) {
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance.get_customer_demand(next);
            customers.erase(remove(customers.begin(), customers.end(), next), customers.end());
        }
    }

    route.push_back(instance.depot);
    all_routes.push_back(route);

    return all_routes;
}


/****************************************************************/
/*                    Local search Operators                    */
/****************************************************************/

double two_opt_for_single_route(vector<int>& route, Case& instance) {
    bool improved = true;
    double totalChange = 0.0;

    while (improved) {
        improved = false;

        for (size_t i = 1; i < route.size() - 2; ++i) {
            for (size_t j = i + 1; j <route.size() - 1; ++j) {
                // Calculate the cost difference between the old route and the new route obtained by swapping edges
                double oldCost = instance.get_distance(route[i - 1], route[i]) +
                                 instance.get_distance(route[j], route[j + 1]);

                double newCost = instance.get_distance(route[i - 1], route[j]) +
                                 instance.get_distance(route[i], route[j + 1]);

                if (newCost < oldCost) {
                    // The cost variation should be considered
                    reverse(route.begin() + i, route.begin() + j + 1);
                    improved = true;
                    totalChange += newCost - oldCost;
                }
            }
        }
    }

    return totalChange;
}

// Croes, Georges A. "A method for solving traveling-salesman problems." Operations research 6, no. 6 (1958): 791-812.
bool two_opt_for_individual(Individual& individual, Case& instance) {
    vector<vector<int>> routes = individual.get_routes();
    double totalChange = 0;
    for (auto& route : routes) {
        double change = two_opt_for_single_route(route, instance);
        totalChange += change;
    }
    individual.set_fit(individual.get_fit() + totalChange);
    individual.set_routes(routes);

    return totalChange != 0;
}

// Jia Ya-Hui, et al.
bool two_opt_star_for_individual(Individual& individual, Case& instance) {
    if (individual.route_num == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> routepairs;
    for (int i = 0; i < individual.route_num - 1; i++) {
        for (int j = i + 1; j < individual.route_num; j++) {
            routepairs.insert(make_pair(i, j));
        }
    }
    int* tempr = new int[individual.node_cap];
    int* tempr2 = new int[individual.node_cap];
    bool updated = false;
    bool updated2 = false;
    while (!routepairs.empty())
    {
        updated2 = false;
        int r1 = routepairs.begin()->first;
        int r2 = routepairs.begin()->second;
        routepairs.erase(routepairs.begin());
        int frdem = 0;
        for (int n1 = 0; n1 < individual.node_num[r1] - 1; n1++) {
            frdem += instance.get_customer_demand(individual.routes[r1][n1]);
            int srdem = 0;
            for (int n2 = 0; n2 < individual.node_num[r2] - 1; n2++) {
                srdem += instance.get_customer_demand(individual.routes[r2][n2]);
                if (frdem + individual.demand_sum[r2] - srdem <= instance.maxC && srdem + individual.demand_sum[r1] - frdem <= instance.maxC) {
                    double xx1 = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1]) +
                            instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double xx2 = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2 + 1]) +
                            instance.get_distance(individual.routes[r2][n2], individual.routes[r1][n1 + 1]);
                    double change = xx1 - xx2;
                    if (change > 0.00000001) {
                        individual.fit -= change;
                        memcpy(tempr, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2 + 1; i < individual.node_num[r2]; i++) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = n2 + 1;
                        for (int i = n1 + 1; i < individual.node_num[r1]; i++) {
                            individual.routes[r2][counter2] = tempr[i];
                            counter2++;
                        }
                        individual.node_num[r1] = counter1;
                        individual.node_num[r2] = counter2;
                        int newdemsum1 = frdem + individual.demand_sum[r2] - srdem;
                        int newdemsum2 = srdem + individual.demand_sum[r1] - frdem;
                        individual.demand_sum[r1] = newdemsum1;
                        individual.demand_sum[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            routepairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            routepairs.insert({i, r2});
                        }
                        if (individual.demand_sum[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r1] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r1] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                routepairs.erase({i, individual.route_num});
                            }
                        }
                        if (individual.demand_sum[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r2] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r2] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                routepairs.erase({i, individual.route_num});
                            }
                        }
                        break;
                    }
                }
                else if (frdem + srdem <= instance.maxC && individual.demand_sum[r1] - frdem + individual.demand_sum[r2] - srdem <= instance.maxC) {
                    double xx1 = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1])
                                 + instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double xx2 = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2])
                                 + instance.get_distance(individual.routes[r1][n1 + 1], individual.routes[r2][n2 + 1]);
                    double change = xx1 - xx2;
                    if (change > 0.00000001) {
                        individual.fit -= change;
                        memcpy(tempr, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2; i >= 0; i--) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = 0;
                        for (int i = individual.node_num[r1] - 1; i >= n1 + 1; i--) {
                            tempr2[counter2] = tempr[i];
                            counter2++;
                        }
                        for (int i = n2 + 1; i < individual.node_num[r2]; i++) {
                            tempr2[counter2] = individual.routes[r2][i];
                            counter2++;
                        }
                        memcpy(individual.routes[r2], tempr2, sizeof(int) * individual.node_cap);
                        individual.node_num[r1] = counter1;
                        individual.node_num[r2] = counter2;

                        int newdemsum1 = frdem + srdem;
                        int newdemsum2 = individual.demand_sum[r1] + individual.demand_sum[r2] - frdem - srdem;
                        individual.demand_sum[r1] = newdemsum1;
                        individual.demand_sum[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            routepairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            routepairs.insert({i, r2});
                        }
                        if (individual.demand_sum[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r1] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r1] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                routepairs.erase({i, individual.route_num});
                            }
                        }
                        if (individual.demand_sum[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r2] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r2] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                routepairs.erase({i, individual.route_num});
                            }
                        }
                        break;
                    }
                }
            }
            if (updated2) break;
        }
    }
    delete[] tempr;
    delete[] tempr2;
    return updated;
}

void node_shift_for_individual(Individual& individual, Case& instance) {
    for (int i = 0; i < individual.route_num; i++) {
        node_shift(individual.routes[i], individual.node_num[i], individual.fit, instance);
    }
}

bool node_shift(int* route, int length, double& fitv, Case& instance) {
    if (length <= 4) return false;
    double minchange = 0;
    bool flag = false;
    do
    {
        minchange = 0;
        int mini = 0, minj = 0;
        for (int i = 1; i < length - 1; i++) {
            for (int j = 1; j < length - 1; j++) {
                if (i < j) {
                    double xx1 = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1]) + instance.get_distance(route[j], route[j + 1]);
                    double xx2 = instance.get_distance(route[i - 1], route[i + 1]) + instance.get_distance(route[j], route[i]) + instance.get_distance(route[i], route[j + 1]);
                    double change = xx1 - xx2;
                    if (fabs(change) < 0.00000001) change = 0;
                    if (minchange < change) {
                        minchange = change;
                        mini = i;
                        minj = j;
                        flag = true;
                    }
                }
                else if (i > j) {
                    double xx1 = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1]) + instance.get_distance(route[j - 1], route[j]);
                    double xx2 = instance.get_distance(route[j - 1], route[i]) + instance.get_distance(route[i], route[j]) + instance.get_distance(route[i - 1], route[i + 1]);
                    double change = xx1 - xx2;
                    if (fabs(change) < 0.00000001) change = 0;
                    if (minchange < change) {
                        minchange = change;
                        mini = i;
                        minj = j;
                        flag = true;
                    }
                }
            }
        }
        if (minchange > 0) {
            moveItoJ(route, mini, minj);
            fitv -= minchange;
        }
    } while (minchange > 0);
    return flag;
}

void moveItoJ(int* route, int a, int b) {
    int x = route[a];
    if (a < b) {
        for (int i = a; i < b; i++) {
            route[i] = route[i + 1];
        }
        route[b] = x;
    }
    else if (a > b) {
        for (int i = a; i > b; i--) {
            route[i] = route[i - 1];
        }
        route[b] = x;
    }
}


/****************************************************************/
/*                   Recharging Optimization                    */
/****************************************************************/

double fix_one_solution(Individual &individual, Case& instance) {
    double updated_fit = 0;
    vector<vector<int>> repaired_routes;
    bool isFeasible = true;
    for (int i = 0; i < individual.route_num; i++) {
        pair<double, vector<int>> res_xx = insert_station_by_simple_enumeration_array(individual.routes[i], individual.node_num[i], instance);
        double xx = res_xx.first;

        if (xx == -1) {
            pair<double, vector<int>> res_yy = insert_station_by_remove_array(individual.routes[i], individual.node_num[i], instance);
            double yy = res_yy.first;
            if (yy == -1) {
                updated_fit += INFEASIBLE;
                isFeasible = false;
            }
            else {
                updated_fit += yy;
                repaired_routes.push_back(res_yy.second);
            }
        }
        else {
            updated_fit += xx;
            repaired_routes.push_back(res_xx.second);
        }
    }
    individual.set_fit(updated_fit);
    if (isFeasible) {
        individual.set_tour(repaired_routes);
    }
    return updated_fit;
}

pair<double, vector<int>> insert_station_by_simple_enumeration_array(int *route, int length, Case& instance) {
    vector<int> full_route;
    vector<double> accumulateDistance(length, 0);
    for (int i = 1; i < length; i++) {
        accumulateDistance[i] = accumulateDistance[i - 1] + instance.get_distance(route[i], route[i - 1]);
    }
    if (accumulateDistance.back() <= instance.maxDis) {
        for (int i = 0; i < length; ++i) {
            full_route.push_back(route[i]);
        }
        return make_pair(accumulateDistance.back(), full_route);
    }

    int ub = (int)(accumulateDistance.back() / instance.maxDis + 1);
    int lb = (int)(accumulateDistance.back() / instance.maxDis);
    int* chosenPos = new int[length];
    int* bestChosenPos = new int[length]; // customized variable
    double finalfit = DBL_MAX;
    double bestfit = finalfit; // customized variable
    for (int i = lb; i <= ub; i++) {
        tryACertainNArray(0, i, chosenPos, bestChosenPos, finalfit, i, route, length, accumulateDistance, instance);

        if (finalfit < bestfit) {
            full_route.clear();
            int idx = 0;
            for (int j = 0; j < i; ++j) {
                int from = route[bestChosenPos[j]];
                int to = route[bestChosenPos[j] + 1];
                int station = instance.bestStation[from][to];

                full_route.insert(full_route.end(), route + idx, route + bestChosenPos[j] + 1);
                full_route.push_back(station);

                idx = bestChosenPos[j] + 1;
            }
            full_route.insert(full_route.end(), route + idx, route + length);
            bestfit = finalfit;
        }

    }
    delete[] chosenPos;
    delete[] bestChosenPos;
    if (finalfit != DBL_MAX) {
        return make_pair(finalfit, full_route);
    }
    else {
        return make_pair(-1, full_route);
    }
}

pair<double, vector<int>> insert_station_by_remove_array(int *route, int length, Case& instance) {
    vector<int> full_route;

    list<pair<int, int>> stationInserted;
    for (int i = 0; i < length - 1; i++) {
        double allowedDis = instance.maxDis;
        if (i != 0) {
            allowedDis = instance.maxDis - instance.get_distance(stationInserted.back().second, route[i]);
        }
        int onestation = instance.find_best_station_feasible(route[i], route[i + 1], allowedDis);
        if (onestation == -1) return make_pair(-1, full_route);
        stationInserted.push_back(make_pair(i, onestation));
    }
    while (!stationInserted.empty())
    {
        bool change = false;
        auto delone = stationInserted.begin();
        double savedis = 0;
        auto itr = stationInserted.begin();
        auto next = itr;
        next++;
        if (next != stationInserted.end()) {
            int endInd = next->first;
            int endstation = next->second;
            double sumdis = 0;
            for (int i = 0; i < endInd; i++) {
                sumdis += instance.get_distance(route[i], route[i + 1]);
            }
            sumdis += instance.get_distance(route[endInd], endstation);
            if (sumdis <= instance.maxDis) {
                savedis = instance.get_distance(route[itr->first], itr->second)
                        + instance.get_distance(itr->second, route[itr->first + 1])
                        - instance.get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        else {
            double sumdis = 0;
            for (int i = 0; i < length - 1; i++) {
                sumdis += instance.get_distance(route[i], route[i + 1]);
            }
            if (sumdis <= instance.maxDis) {
                savedis = instance.get_distance(route[itr->first], itr->second)
                        + instance.get_distance(itr->second, route[itr->first + 1])
                        - instance.get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        itr++;
        while (itr != stationInserted.end())
        {
            int startInd, endInd;
            next = itr;
            next++;
            auto prev = itr;
            prev--;
            double sumdis = 0;
            if (next != stationInserted.end()) {
                startInd = prev->first + 1;
                endInd = next->first;
                sumdis += instance.get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < endInd; i++) {
                    sumdis += instance.get_distance(route[i], route[i + 1]);
                }
                sumdis += instance.get_distance(route[endInd], next->second);
                if (sumdis <= instance.maxDis) {
                    double savedistemp = instance.get_distance(route[itr->first], itr->second)
                            + instance.get_distance(itr->second, route[itr->first + 1])
                            - instance.get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            else {
                startInd = prev->first + 1;
                sumdis += instance.get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < length - 1; i++) {
                    sumdis += instance.get_distance(route[i], route[i + 1]);
                }
                if (sumdis <= instance.maxDis) {
                    double savedistemp = instance.get_distance(route[itr->first], itr->second)
                            + instance.get_distance(itr->second, route[itr->first + 1])
                            - instance.get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            itr++;
        }
        if (savedis != 0) {
            stationInserted.erase(delone);
            change = true;
        }
        if (!change) {
            break;
        }
    }
    double sum = 0;
    for (int i = 0; i < length - 1; i++) {
        sum += instance.get_distance(route[i], route[i + 1]);
    }
    int idx = 0;
    for (auto& e : stationInserted) {
        int pos = e.first;
        int stat = e.second;
        sum -= instance.get_distance(route[pos], route[pos + 1]);
        sum += instance.get_distance(route[pos], stat);
        sum += instance.get_distance(stat, route[pos + 1]);
        full_route.insert(full_route.end(), route + idx, route + pos + 1);
        full_route.push_back(stat);
        idx = pos + 1;
    }
    full_route.insert(full_route.end(), route + idx, route + length);
    return make_pair(sum, full_route);
}

void tryACertainNArray(int mlen, int nlen, int* chosenPos, int* bestChosenPos, double& finalfit, int curub, int* route, int length, vector<double>& accumulateDis, Case& instance) {
    for (int i = mlen; i <= length - 1 - nlen; i++) {
        if (curub == nlen) {
            double onedis = instance.get_distance(route[i], instance.bestStation[route[i]][route[i + 1]]);
            if (accumulateDis[i] + onedis > instance.maxDis) {
                break;
            }
        }
        else {
            int lastpos = chosenPos[curub - nlen - 1];
            double onedis = instance.get_distance(route[lastpos + 1], instance.bestStation[route[lastpos]][route[lastpos + 1]]);
            double twodis = instance.get_distance(route[i], instance.bestStation[route[i]][route[i + 1]]);
            if (accumulateDis[i] - accumulateDis[lastpos + 1] + onedis + twodis > instance.maxDis) {
                break;
            }
        }
        if (nlen == 1) {
            double onedis = accumulateDis.back() - accumulateDis[i + 1] + instance.get_distance(instance.bestStation[route[i]][route[i + 1]], route[i + 1]);
            if (onedis > instance.maxDis) {
                continue;
            }
        }

        chosenPos[curub - nlen] = i;
        if (nlen > 1) {
            tryACertainNArray(i + 1, nlen - 1, chosenPos,  bestChosenPos, finalfit, curub, route, length, accumulateDis, instance);
        }
        else {
            double disum = accumulateDis.back();
            for (int j = 0; j < curub; j++) {
                int firstnode = route[chosenPos[j]];
                int secondnode = route[chosenPos[j] + 1];
                int thestation = instance.bestStation[firstnode][secondnode];
                disum -= instance.get_distance(firstnode, secondnode);
                disum += instance.get_distance(firstnode, thestation);
                disum += instance.get_distance(secondnode, thestation);
            }
            if (disum < finalfit) {
                finalfit = disum;
                for (int j = 0; j < length; ++j) {
                    bestChosenPos[j] = chosenPos[j];
                }
            }
        }
    }
}

pair<double, vector<int>> simple_repair_target_one_station(const int* route, int length, Case& instance) {
    vector<int> fullRoute;

    vector<double> distances(length, 0);
    for (int i = 1; i < length; i++) {
        distances[i] = instance.get_distance(route[i], route[i - 1]);
    }
    double dumbTotalDistance = std::accumulate(distances.begin(), distances.end(), 0.0);

//    if (dumbTotalDistance/MAX_DIS <= 1 || dumbTotalDistance/MAX_DIS > 2)
//        throw std::runtime_error("This operator (simple repair) is just designed for one station\n");
    if (dumbTotalDistance/instance.maxDis <= 1 || dumbTotalDistance/instance.maxDis > 2) {
        return make_pair(-1, fullRoute);
    }


    int current = 0;
    int next = current + 1;

    fullRoute.push_back(route[current]);
    double accumulatedTotalDistance = 0;
    double availableRange = instance.maxDis;

    while (next <= length - 1) {
        if(availableRange >= distances[next]) {

            // 判断是否从current可以到达next
            if (availableRange - distances[next] >= instance.customerNearestStationMap[route[next]].second || next == length - 1 ) {
                // 判断是否next可以到达最近的充电站，假设EV到达next 或者 下一个节点为仓库
                fullRoute.push_back(route[next]);
                accumulatedTotalDistance += distances[next];
                availableRange -= distances[next];
            } else {
                // 若从next出发无法到达充电站，那么我们应该在前一段旅程中充电，即在current后面充电
                // 我们希望找到一个充电站，从current出发可达，且最靠近next （goal - 用尽可能少的充电站）
                int station = instance.find_nearest_station_to_y_feasible(route[current], route[next], availableRange);
                if (station == -1) throw std::runtime_error("Cannot find feasible station!"); // in theory, it shouldn't happen
                fullRoute.push_back(station);
                fullRoute.push_back(route[next]);
                double current2station = instance.get_distance(route[current], station);
                double station2next = instance.get_distance(station, route[next]);
                accumulatedTotalDistance += current2station + station2next;
                availableRange = instance.maxDis - station2next;
            }
        } else {
            int station = instance.find_nearest_station_to_y_feasible(route[current], route[next], availableRange);
            if (station == -1) throw std::runtime_error("Cannot find feasible station from current!"); // in theory, it shouldn't happen
            fullRoute.push_back(station);
            fullRoute.push_back(route[next]);
            double current2station = instance.get_distance(route[current], station);
            double station2next = instance.get_distance(station, route[next]);
            accumulatedTotalDistance += current2station + station2next;
            availableRange = instance.maxDis - instance.get_distance(station, route[next]);
        }
        current = next;
        next = current + 1;
    }


    return make_pair(accumulatedTotalDistance, fullRoute);
}

pair<double, vector<int>> station_reallocate_one(vector<int>& repairedForwardRoute, double forwardFit, Case& instance) {
    // input arguments check
    int stationNum = 0;
    vector<int> dumbForwardRoute;
    int posStationForward = -1;
    for (int i = 0; i < repairedForwardRoute.size(); ++i) {
        int node = repairedForwardRoute[i];
        if (node != instance.depot && instance.is_charging_station(node)) {
            stationNum++;
            posStationForward = i;
        } else {
            dumbForwardRoute.push_back(node);
        }
    }

    if (stationNum != 1) throw std::runtime_error("The input argument \"repairedForwardRoute\" should only contains one station!");

    int* route = new int[dumbForwardRoute.size()];
    std::copy(dumbForwardRoute.rbegin(), dumbForwardRoute.rend(), route);
    pair<double, vector<int>> reverseRouteInfo = simple_repair_target_one_station(route,dumbForwardRoute.size(), instance);
    delete []route;
    vector<int> repairedBackwardRoute = reverseRouteInfo.second;
    double backwardFit = reverseRouteInfo.first;

    vector<int> dumbBackwardRoute;
    int posStationBackward = -1;
    for (int i = 0; i < repairedBackwardRoute.size(); ++i) {
        int node = repairedBackwardRoute[i];
        if (node != instance.depot && instance.is_charging_station(node)) {
            posStationBackward = i;
        } else{
            dumbBackwardRoute.push_back(node);
        }
    }

    vector<vector<double>> distanceMatrix(instance.customerNumber + 1, vector<double>(instance.customerNumber + 1, 0.0));
    for (int i = 0; i < dumbForwardRoute.size() - 1; ++i) {
        double distance = instance.get_distance(dumbForwardRoute[i], dumbForwardRoute[i+1]);
        distanceMatrix[dumbForwardRoute[i]][dumbForwardRoute[i+1]] = distance;
        distanceMatrix[dumbForwardRoute[i+1]][dumbForwardRoute[i]] = distance;
    }
    double totalDumbDistance = forwardFit
            - instance.get_distance(repairedForwardRoute[posStationForward - 1], repairedForwardRoute[posStationForward])
            - instance.get_distance(repairedForwardRoute[posStationForward], repairedForwardRoute[posStationForward + 1])
            + distanceMatrix[repairedForwardRoute[posStationForward - 1]][repairedForwardRoute[posStationForward + 1]];

    double bestFitForward = forwardFit;
    int bestPosForward = posStationForward;
    int bestStationForward = repairedForwardRoute[posStationForward];
    // forward reallocate station O(n)
    int left = 0;
    for (int i = 0; i < dumbForwardRoute.size(); ++i) {
        if (dumbForwardRoute[i] == repairedBackwardRoute[posStationBackward + 1]) {
            left = i;
            break;
        }
    }
    for (int idx = left; idx < posStationForward; ++idx) {
        // insert station after idx
        int station = instance.bestStation[dumbForwardRoute[idx]][dumbForwardRoute[idx + 1]];

        double from2station = instance.get_distance(dumbForwardRoute[idx], station);
        double station2to   = instance.get_distance(station, dumbForwardRoute[idx + 1]);

        // better? if worse, just continue
        double updatedTotalDumbDistance = totalDumbDistance - distanceMatrix[dumbForwardRoute[idx]][dumbForwardRoute[idx + 1]];
        updatedTotalDumbDistance += from2station + station2to;
        if (updatedTotalDumbDistance > forwardFit) {
            continue; // if worse, just continue
        }

        // feasible? if better, check the feasibility
        bool isFeasible = true;
        double availableRange = instance.maxDis;
        for (int index = 0; index < dumbForwardRoute.size() - 1; ++index) {
            if (index == idx) {
                availableRange -= from2station;
                if (availableRange < 0) {
                    isFeasible = false;
                    break;
                }
                availableRange = instance.maxDis - station2to;
            } else {
                availableRange -= distanceMatrix[dumbForwardRoute[index]][dumbForwardRoute[index+1]];
                if (availableRange < 0) {
                    isFeasible = false;
                    break;
                }
            }
        }

        if (!isFeasible) continue;

        // record the best info
        if (bestFitForward > updatedTotalDumbDistance) {
            bestFitForward = updatedTotalDumbDistance;
            bestPosForward = idx;
            bestStationForward = station;
        }
    }

    // backward reallocate station O(n)
    double bestFitBackward = backwardFit;
    int bestPosBackward = posStationBackward;
    int bestStationBackward = repairedBackwardRoute[posStationBackward];
    for (int i = 0; i < dumbBackwardRoute.size(); ++i) {
        if (dumbBackwardRoute[i] == repairedForwardRoute[posStationForward + 1]) {
            left = i;
            break;
        }
    }
    for (int idx = left; idx < posStationBackward; ++idx) {
        // insert station after idx
        int station = instance.bestStation[dumbBackwardRoute[idx]][dumbBackwardRoute[idx + 1]];

        double from2station = instance.get_distance(dumbBackwardRoute[idx], station);
        double station2to   = instance.get_distance(station, dumbBackwardRoute[idx + 1]);

        // better? if worse, just continue
        double updatedTotalDumbDistance = totalDumbDistance - distanceMatrix[dumbBackwardRoute[idx]][dumbBackwardRoute[idx + 1]];
        updatedTotalDumbDistance += from2station + station2to;
        if (updatedTotalDumbDistance > backwardFit) {
            continue; // if worse, just continue
        }

        // feasible? if better, check the feasibility
        bool isFeasible = true;
        double availableRange = instance.maxDis;
        for (int index = 0; index < dumbBackwardRoute.size() - 1; ++index) {
            if (index == idx) {
                availableRange -= from2station;
                if (availableRange < 0) {
                    isFeasible = false;
                    break;
                }
                availableRange = instance.maxDis - station2to;
            } else {
                availableRange -= distanceMatrix[dumbBackwardRoute[index]][dumbBackwardRoute[index+1]];
                if (availableRange < 0) {
                    isFeasible = false;
                    break;
                }
            }
        }

        if (!isFeasible) continue;

        // record the best info
        if (bestFitBackward > updatedTotalDumbDistance) {
            bestFitBackward = updatedTotalDumbDistance;
            bestPosBackward = idx + 1;
            bestStationBackward = station;
        }
    }

    if (forwardFit <= bestFitForward && forwardFit <= bestFitBackward)
        return make_pair(forwardFit, repairedForwardRoute);
    if (bestFitForward < bestFitBackward) {
        dumbForwardRoute.insert(dumbForwardRoute.begin() + bestPosForward, bestStationForward);
        return make_pair(bestFitForward, dumbForwardRoute);
    } else {
        dumbBackwardRoute.insert(dumbBackwardRoute.begin() + bestPosBackward, bestStationBackward);
        return make_pair(bestFitBackward, dumbBackwardRoute);
    }
}


/****************************************************************/
/*                            Refine                            */
/****************************************************************/

pair<vector<int>, double> insert_station_by_enumeration(vector<int>& route, Case& instance) {
    vector<double> accumulateDistance(route.size(), 0);
    for (int i = 1; i < (int)route.size(); i++) {
        accumulateDistance[i] = accumulateDistance[i - 1] + instance.get_distance(route[i], route[i - 1]);
    }
    if (accumulateDistance.back() <= instance.maxDis) {
        return make_pair(route, accumulateDistance.back());
    }

    int ub = ceil(accumulateDistance.back() / instance.maxDis);
    int lb = floor(accumulateDistance.back() / instance.maxDis);
    int* chosenPos = new int[route.size()];
    int* chosenSta = new int[route.size()];
    vector<int> finalRoute;
    double finalfit = DBL_MAX;
    for (int i = lb; i <= ub; i++) {
        tryACertainN(0, i, chosenSta, chosenPos, finalRoute, finalfit, i, route, accumulateDistance, instance);
    }
    delete[] chosenPos;
    delete[] chosenSta;
    if (finalfit != DBL_MAX) {
        return make_pair(finalRoute, finalfit);
    }
    else {
        return make_pair(route, -1);
    }
}

void tryACertainN(int mlen, int nlen, int* chosenSta, int* chosenPos, vector<int>& finalRoute, double& finalfit, int curub, vector<int>& route, vector<double>& accumulateDis, Case& instance) {
    for (int i = mlen; i <= (int)route.size() - 1 - nlen; i++) {
        if (curub == nlen) {
            if (accumulateDis[i] >= instance.maxDis) {
                break;
            }
        }
        else {
            if (accumulateDis[i] - accumulateDis[chosenPos[curub - nlen - 1] + 1] >= instance.maxDis) {
                break;
            }
        }
        if (nlen == 1) {
            if (accumulateDis.back() - accumulateDis[i + 1] >= instance.maxDis) {
                continue;
            }
        }
        for (int j = 0; j < instance.stationNumber; j++) { //TODO: 这个地方可以琢磨修改 (int)instance->bestStations[route[i]][route[i + 1]].size()
            chosenSta[curub - nlen] = instance.stations[j];
            chosenPos[curub - nlen] = i;
            if (nlen > 1) {
                tryACertainN(i + 1, nlen - 1, chosenSta, chosenPos, finalRoute, finalfit, curub, route, accumulateDis, instance);
            }
            else {
                bool feasible = true;
                double piecedis = accumulateDis[chosenPos[0]] + instance.get_distance(route[chosenPos[0]],chosenSta[0]);
                if (piecedis > instance.maxDis) feasible = false;
                for (int k = 1; feasible && k < curub; k++) {
                    piecedis = accumulateDis[chosenPos[k]] - accumulateDis[chosenPos[k - 1] + 1];
                    piecedis += instance.get_distance(chosenSta[k - 1], route[chosenPos[k - 1] + 1]);
                    piecedis += instance.get_distance(chosenSta[k], route[chosenPos[k]]);
                    if (piecedis > instance.maxDis) feasible = false;
                }
                piecedis = accumulateDis.back() - accumulateDis[chosenPos[curub - 1] + 1];
                piecedis += instance.get_distance(route[chosenPos[curub - 1] + 1],chosenSta[curub - 1]);
                if (piecedis > instance.maxDis) feasible = false;
                if (feasible) {
                    double totaldis = accumulateDis.back();
                    for (int k = 0; k < curub; k++) {
                        int firstnode = route[chosenPos[k]];
                        int secondnode = route[chosenPos[k] + 1];
                        totaldis -= instance.get_distance(firstnode,secondnode);
                        totaldis += instance.get_distance(firstnode,chosenSta[k]);
                        totaldis += instance.get_distance(chosenSta[k],secondnode);
                    }
                    if (totaldis < finalfit) {
                        finalfit = totaldis;
                        finalRoute = route;
                        for (int k = curub - 1; k >= 0; k--) {
                            finalRoute.insert(finalRoute.begin() + chosenPos[k] + 1, chosenSta[k]);
                        }
                    }
                }
            }
        }
    }
}


/****************************************************************/
/*                 Genetic Algorithm Operators                  */
/****************************************************************/

vector<vector<int>> selRandom(const vector<vector<int>>& chromosomes, int k, std::default_random_engine& rng) {
    vector<vector<int>> selectedSeqs;

    std::uniform_int_distribution<std::size_t> distribution(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t randomIndex = distribution(rng);
        selectedSeqs.push_back(chromosomes[randomIndex]);
    }

    return selectedSeqs;
}

vector<shared_ptr<Individual>> selRandom(const vector<shared_ptr<Individual>>& individuals, int k, std::default_random_engine& rng) {
    vector<shared_ptr<Individual>> selectedIndividuals;

    std::uniform_int_distribution<std::size_t> distribution(0, individuals.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t randomIndex = distribution(rng);
        selectedIndividuals.push_back(individuals[randomIndex]);
    }

    return selectedIndividuals;
}

vector<shared_ptr<Individual>> selTournament(const vector<shared_ptr<Individual>>& individuals, int k, int tournamentSize, std::default_random_engine& rng) {
    vector<shared_ptr<Individual>> chosen;

    for (int i = 0; i < k; ++i) {
        vector<shared_ptr<Individual>> aspirants = selRandom(individuals, tournamentSize, rng);

        // Assuming you have a fitness attribute in your Individual class
        auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
            return ind1->fit < ind2->fit;
        };

        auto minElement = min_element(aspirants.begin(), aspirants.end(), comparator);
        chosen.push_back(*minElement);

    }

    return chosen;
}

void cxPartiallyMatched(vector<int>& parent1, vector<int>& parent2, std::default_random_engine& rng) {
    int size = parent1.size();

    uniform_int_distribution<> distribution(0, size - 1);

    int point1 = distribution(rng);
    int point2 = distribution(rng);

    if (point1 > point2) {
        swap(point1, point2);
    }

    // Copy the middle segment from parents to children
    vector<int> child1(parent1.begin() + point1, parent1.begin() + point2);
    vector<int> child2(parent2.begin() + point1, parent2.begin() + point2);

    // Create a mapping of genes between parents
    unordered_map<int, int> mapping1;
    unordered_map<int, int> mapping2;

    // Initialize mapping with the middle segment
    for (int i = 0; i < point2 - point1; ++i) {
        mapping1[child2[i]] = child1[i];
        mapping2[child1[i]] = child2[i];
    }

    // Copy the rest of the genes, filling in the mapping
    for (int i = 0; i < size; ++i) {
        if (i < point1 || i >= point2) {
            int gene1 = parent1[i];
            int gene2 = parent2[i];

            while (mapping1.find(gene1) != mapping1.end()) {
                gene1 = mapping1[gene1];
            }

            while (mapping2.find(gene2) != mapping2.end()) {
                gene2 = mapping2[gene2];
            }

            child1.push_back(gene2);
            child2.push_back(gene1);
        }
    }

    // Modify the input arguments directly
    parent1 = child1;
    parent2 = child2;
}

void mutShuffleIndexes(vector<int>& chromosome, double indpb, std::default_random_engine& rng) {
    int size = chromosome.size();

    uniform_real_distribution<double> dis(0.0, 1.0);
    uniform_int_distribution<int> swapDist(0, size - 2);

    for (int i = 0; i < size; ++i) {
        if (dis(rng) < indpb) {
            int swapIndex = swapDist(rng);
            if (swapIndex >= i) {
                swapIndex += 1;
            }
            swap(chromosome[i], chromosome[swapIndex]);
        }
    }
}


/****************************************************************/
/*                             Tools                            */
/****************************************************************/

shared_ptr<Individual> select_best_individual(const vector<shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    // Assuming you have a fitness attribute in your Individual class
    auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
        return ind1->get_fit() < ind2->get_fit();
    };

    auto bestIndividual = std::min_element(population.begin(), population.end(), comparator);

    return *bestIndividual;
}

shared_ptr<Individual> select_worst_individual(const vector<shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    // Assuming you have a fitness attribute in your Individual class
    auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
        return ind1->get_fit() < ind2->get_fit();
    };

    auto worstIndividual = std::max_element(population.begin(), population.end(), comparator);

    return *worstIndividual;
}
