#ifndef __OR_ROBOTSIM_JOINT_MAP_HPP
#define __OR_ROBOTSIM_JOINT_MAP_HPP

#include <string>
#include <map>

class JointMaps
{
public:
    void setMaps();
    std::map<std::string,int> or_map;
    std::map<std::string,int> rs_map;
    std::map<std::string,int> dt_map;
};

void JointMaps::setMaps()
{
    or_map["HPY"]= 0;
    or_map["RHY"]= 1;
    or_map["LHY"]= 2;
    or_map["RHR"]= 3;
    or_map["LHR"]= 4;
    or_map["RHP"]= 5;
    or_map["LHP"]= 6;
    or_map["RKP"]= 7;
    or_map["LKP"]= 8;
    or_map["RAP"]= 9;
    or_map["LAP"]= 10;
    or_map["RAR"]= 11;
    or_map["LAR"]= 12;
    or_map["RSP"]= 13;
    or_map["LSP"]= 14;
    or_map["RSR"]= 15;
    or_map["LSR"]= 16;
    or_map["RSY"]= 17;
    or_map["LSY"]= 18;
    or_map["REP"]= 19;
    or_map["LEP"]= 20;
    or_map["RWY"]= 21;
    or_map["LWY"]= 22;
    or_map["RWP"]= 23;
    or_map["LWP"]= 24;
    or_map["HNR"]= 25;
    or_map["HNP"]= 26;

    rs_map["HNR"]= 1-1;
    rs_map["HNP"]= 2-1;
    rs_map["LSP"]= 3-1;
    rs_map["LSR"]= 4-1;
    rs_map["LSY"]= 5-1;
    rs_map["LEP"]= 6-1;
    rs_map["LWY"]= 7-1;
    rs_map["LWP"]= 8-1;
    rs_map["RSP"]= 24-1;
    rs_map["RSR"]= 25-1;
    rs_map["RSY"]= 26-1;
    rs_map["REP"]= 27-1;
    rs_map["RWY"]= 28-1;
    rs_map["RWP"]= 29-1;
    rs_map["HPY"]= 45-1;
    rs_map["LHY"]= 46-1;
    rs_map["LHR"]= 47-1;
    rs_map["LHP"]= 48-1;
    rs_map["LKP"]= 49-1;
    rs_map["LAP"]= 50-1;
    rs_map["LAR"]= 51-1;
    rs_map["RHY"]= 52-1;
    rs_map["RHR"]= 53-1;
    rs_map["RHP"]= 54-1;
    rs_map["RKP"]= 55-1;
    rs_map["RAP"]= 56-1;
    rs_map["RAR"]= 57-1;
};

#endif // JOINT_MAP_HPP
