#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

class Case;
class Individual;

class Follower {
public:
    static void optimize_charging(Individual& individual, Case& instance);
    static void refine_charging_by_enumeration(Individual& individual, Case& instance);
};

#endif
