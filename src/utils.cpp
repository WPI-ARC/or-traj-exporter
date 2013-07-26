#include "utils.hpp"

bool ortconv::fct_sort( std::pair<int,RobotAndDof> a, std::pair<int,RobotAndDof> b)
{
    return a.first < b.first;
}
