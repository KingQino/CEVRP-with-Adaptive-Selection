//
// Created by Yinghao Qin on 16/11/2023.
//

#ifndef CEVRP_YINGHAO_INDIVIDUAL_HPP
#define CEVRP_YINGHAO_INDIVIDUAL_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <limits>

class Individual {
public:
    static const int TOUR_SIZE;

    int route_cap; // route capacity - 2 by MIN_VEHICLES
    int node_cap; // node capacity - NUM_OF_CUSTOMERS + num_of_depot
    int** routes;
    int route_num; // the actual number of routes for the solution
    int* node_num; // the node number of each route
    int* demand_sum; // the demand sum of all customers of each route
    double upper_cost; // routing cost before charging decisions
    double lower_cost; // complete cost after charging decisions
    bool upper_locally_optimal;
    int* tour; // The specified format of the solution, e.g., 0 - 5 - 6 - 8 - 0 - 1 - 2 - 3 - 4 - 0 - 7 - 0
    int tour_capacity;
    int steps;

    Individual(const Individual  &ind);
    Individual(int route_cap, int node_cap);
    Individual(int route_cap, int node_cap, const std::vector<std::vector<int>>& routes, double upper_cost, const std::vector<int>& demand_sum);
    ~Individual();

    void reset();
    [[nodiscard]] std::vector<std::vector<int>> get_routes() const;
    [[nodiscard]] std::vector<int> get_chromosome() const;
    [[nodiscard]] double get_upper_cost() const;
    [[nodiscard]] double get_lower_cost() const;
    [[nodiscard]] bool is_upper_locally_optimal() const;
    void set_upper_cost(double cost);
    void set_lower_cost(double cost);
    void set_upper_locally_optimal(bool locally_optimal);
    void invalidate_lower_cost();
    void load_upper_solution(
        const std::vector<std::vector<int>>& routes,
        double cost,
        const std::vector<int>& route_demand_sum);
    void copy_from(const Individual& other);
    void set_routes(const std::vector<std::vector<int>>& _routes);
    std::pair<int*, int> get_tour();
    void set_tour(const std::vector<std::vector<int>>& repaired_routes);



    friend std::ostream& operator<<(std::ostream& os, const Individual& individual);

private:
    void ensure_tour_capacity(int required_capacity);
};

#endif //CEVRP_YINGHAO_INDIVIDUAL_HPP
