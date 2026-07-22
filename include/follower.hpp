#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

#include <vector>

class Case;
class Individual;

struct FollowerWorkspace {
    std::vector<double> cumulativeDistance;
    std::vector<int> chosenPositions;
};

class Follower {
public:
    static void optimize_charging(Individual& individual, Case& instance);
    static void optimize_charging(
        Individual& individual,
        Case& instance,
        FollowerWorkspace& workspace);
    static void refine_charging_by_enumeration(Individual& individual, Case& instance);
};

#endif
