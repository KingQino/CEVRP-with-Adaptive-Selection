#ifndef LEADER_HPP
#define LEADER_HPP

#include <random>

class Case;
class Individual;

class Leader {
public:
    static void improve_with_three_neighborhood_vnd(Individual& individual, Case& instance);
    static void improve_with_three_neighborhood_rvnd(
        Individual& individual,
        Case& instance,
        std::default_random_engine& randomEngine);
    static void improve_with_five_neighborhood_rvnd(
        Individual& individual,
        Case& instance,
        std::default_random_engine& randomEngine);
};

#endif
