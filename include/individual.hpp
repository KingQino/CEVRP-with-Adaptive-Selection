//
// Created by Yinghao Qin on 16/11/2023.
//

#ifndef CEVRP_YINGHAO_INDIVIDUAL_HPP
#define CEVRP_YINGHAO_INDIVIDUAL_HPP

#include <iostream>
#include <vector>
#include <cstring>


using namespace std;

class Individual {
public:
    static const int TOUR_SIZE;

    int route_cap; // route capacity - 2 by MIN_VEHICLES
    int node_cap; // node capacity - NUM_OF_CUSTOMERS + num_of_depot
    int** routes;
    int route_num; // the actual number of routes for the solution
    int* node_num; // the node number of each route
    int* demand_sum; // the demand sum of all customers of each route
    double fit;
    int* tour; // The specified format of the solution, e.g., 0 - 5 - 6 - 8 - 0 - 1 - 2 - 3 - 4 - 0 - 7 - 0
    int steps;

    Individual(const Individual  &ind);
    Individual(int route_cap, int node_cap);
    Individual(int route_cap, int node_cap, const vector<vector<int>>& routes, double fit, const vector<int>& demand_sum);
    ~Individual();

    void reset();
    [[nodiscard]] vector<vector<int>> get_routes() const;
    [[nodiscard]] vector<int> get_chromosome() const;
    [[nodiscard]] double get_fit() const;
    void set_fit(double _fit);
    void set_routes(const vector<vector<int>>& _routes) const;
    pair<int*, int> get_tour();
    void set_tour(const vector<vector<int>>& repaired_routes);



    friend ostream& operator<<(ostream& os, const Individual& individual);
};

#endif //CEVRP_YINGHAO_INDIVIDUAL_HPP
