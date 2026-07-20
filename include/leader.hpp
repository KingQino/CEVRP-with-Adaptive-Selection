#ifndef LEADER_HPP
#define LEADER_HPP

class Case;
class Individual;

class Leader {
public:
    static void improve_with_three_neighborhood_vnd(Individual& individual, Case& instance);
};

#endif
