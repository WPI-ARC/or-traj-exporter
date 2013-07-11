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
    std::map<std::string,int> or_drc;
    std::map<std::string,int> rs_drc;
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





    // Version 2
    or_drc["LSP"]=       0;           //0         Body_Torso  Body_LSP
    or_drc["LSR"]=       1;           //1         Body_LSP    Body_LSR
    or_drc["LSY"]=       2;           //2         Body_LSR    Body_LSY
    or_drc["LEP"]=       3;           //3         Body_LSY    Body_LEP
    or_drc["LWY"]=       4;           //4         Body_LEP    Body_LWY
    or_drc["LWP"]=       5;           //5         Body_LWY    Body_LWP
    or_drc["LWR"]=       6;           //6         Body_LWP    Body_LWR
    or_drc["TSY"]=       10;          //10        Body_Torso  Body_TSY
    or_drc["LHY"]=       11;          //11        Body_TSY    Body_LHY
    or_drc["LHR"]=       12;          //12        Body_LHY    Body_LHR
    or_drc["LHP"]=       13;          //13        Body_LHR    Body_LHP
    or_drc["LKP"]=       14;          //14        Body_LHP    Body_LKP
    or_drc["LAP"]=       15;          //15        Body_LKP    Body_LAP
    or_drc["LAR"]=       16;          //16        Body_LAP    Body_LAR
    or_drc["NKY"]=       17;          //17        Body_Torso  Body_NKY
    or_drc["NK1"]=       18;          //18        Body_NKY    Body_NK1
    or_drc["NK2"]=       -1;          //19        Body_NK1    Body_NK2
    or_drc["RSP"]=       26;          //26        Body_Torso  Body_RSP
    or_drc["RSR"]=       27;          //27        Body_RSP    Body_RSR
    or_drc["RSY"]=       28;          //28        Body_RSR    Body_RSY
    or_drc["REP"]=       29;          //29        Body_RSY    Body_REP
    or_drc["RWY"]=       30;          //30        Body_REP    Body_RWY
    or_drc["RWP"]=       31;          //31        Body_RWY    Body_RWP
    or_drc["RWR"]=       32;          //32        Body_RWP    Body_RWR
    or_drc["RHY"]=       36;          //36        Body_TSY    Body_RHY
    or_drc["RHR"]=       37;          //37        Body_RHY    Body_RHR
    or_drc["RHP"]=       38;          //38        Body_RHR    Body_RHP
    or_drc["RKP"]=       39;          //39        Body_RHP    Body_RKP
    or_drc["RAP"]=       40;          //40        Body_RKP    Body_RAP
    or_drc["RAR"]=       41;          //41        Body_RAP    Body_RAR

//    or_drc["LF11"]=      7;           //7         Body_LWR    Body_LF11
//    or_drc["LF12"]=      8;           //8         Body_LF11   Body_LF12
//    or_drc["LF13"]=      9;           //9         Body_LF12   Body_LF13
//    or_drc["LF21"]=      20;          //20        Body_LWR    Body_LF21
//    or_drc["LF22"]=      21;          //21        Body_LF21   Body_LF22
//    or_drc["LF23"]=      22;          //22        Body_LF22   Body_LF23
//    or_drc["LF31"]=      23;          //23        Body_LWR    Body_LF31
//    or_drc["LF32"]=      24;          //24        Body_LF31   Body_LF32
//    or_drc["LF33"]=      25;          //25        Body_LF32   Body_LF33
//    or_drc["RF11"]=      33;          //33        Body_RWR    Body_RF11
//    or_drc["RF12"]=      34;          //34        Body_RF11   Body_RF12
//    or_drc["RF13"]=      35;          //35        Body_RF12   Body_RF13
//    or_drc["RF21"]=      42;          //42        Body_RWR    Body_RF21
//    or_drc["RF22"]=      43;          //43        Body_RF21   Body_RF22
//    or_drc["RF23"]=      44;          //44        Body_RF22   Body_RF23
//    or_drc["RF31"]=      45;          //45        Body_RWR    Body_RF31
//    or_drc["RF32"]=      46;          //46        Body_RF31   Body_RF32
//    or_drc["RF33"]=      47;          //47        Body_RF32   Body_RF33
//    or_drc["RF41"]=      48;          //48        Body_RWR    Body_RF41
//    or_drc["RF42"]=      49;          //49        Body_RF41   Body_RF42
//    or_drc["RF43"]=      50;          //50        Body_RF42   Body_RF43







    // Version 1
    rs_drc["LHY"]= 6-6;
    rs_drc["LHR"]= 7-6;
    rs_drc["LHP"]= 8-6;
    rs_drc["LKP"]= 9-6;
    rs_drc["LAP"]= 10-6;
    rs_drc["LAR"]= 11-6;
    rs_drc["RHY"]= 12-6;
    rs_drc["RHR"]= 13-6;
    rs_drc["RHP"]= 14-6;
    rs_drc["RKP"]= 15-6;
    rs_drc["RAP"]= 16-6;
    rs_drc["RAR"]= 17-6;
    rs_drc["TSY"]= 18-6;
    rs_drc["LSP"]= 19-6;
    rs_drc["LSR"]= 20-6;
    rs_drc["LSY"]= 21-6;
    rs_drc["LEP"]= 22-6;
    rs_drc["LWY"]= 23-6;
    rs_drc["LWP"]= 24-6;
    rs_drc["LWR"]= 25-6;
    rs_drc["NKY"]= 29-6;
    rs_drc["NK1"]= 30-6;
    rs_drc["RSP"]= 31-6;
    rs_drc["RSR"]= 32-6;
    rs_drc["RSY"]= 33-6;
    rs_drc["REP"]= 34-6;
    rs_drc["RWY"]= 35-6;
    rs_drc["RWP"]= 36-6;
    rs_drc["RWR"]= 37-6;
    rs_drc["LF1"]= 26-6;
    rs_drc["LF2"]= 27-6;
    rs_drc["LF3"]= 28-6;
    rs_drc["RF1"]= 38-6;
    rs_drc["RF2"]= 39-6;
    rs_drc["RF3"]= 40-6;


//    // Version 2
//    //rs_drc["Hip"]= 5;
//    //rs_drc["leftFoot"]= 12;
//    //rs_drc["leftPalm"]= 31;
//    //rs_drc["rightFoot"]= 19;
//    rs_drc["LHY"]= 6;
//    rs_drc["LHR"]= 7;
//    rs_drc["LHP"]= 8;
//    rs_drc["LKP"]= 9;
//    rs_drc["LAP"]= 10;
//    rs_drc["LAR"]= 11;
//    rs_drc["RHY"]= 13;
//    rs_drc["RHR"]= 14;
//    rs_drc["RHP"]= 15;
//    rs_drc["RKP"]= 16;
//    rs_drc["RAP"]= 17;
//    rs_drc["RAR"]= 18;
//    rs_drc["TSY"]= 20; // Torso
//    rs_drc["LSP"]= 21;
//    rs_drc["LSR"]= 22;
//    rs_drc["LSY"]= 23;
//    rs_drc["LEP"]= 24;
//    rs_drc["LWY"]= 25;
//    rs_drc["LWP"]= 26;
//    rs_drc["LWR"]= 27;
//    rs_drc["NKY"]= 32;
//    rs_drc["NK1"]= 33;
//    rs_drc["RSP"]= 34;
//    rs_drc["RSR"]= 35;
//    rs_drc["RSY"]= 36;
//    rs_drc["REP"]= 37;
//    rs_drc["RWY"]= 38;
//    rs_drc["RWP"]= 39;
//    rs_drc["RWR"]= 40;

//    rs_drc["LF1"]= 28;
//    rs_drc["LF2"]= 29;
//    rs_drc["LF3"]= 30;
//    rs_drc["RF1"]= 41;
//    rs_drc["RF2"]= 42;
//    rs_drc["RF3"]= 43;
//    //rs_drc["rightPalm"]= 44;

};

#endif // JOINT_MAP_HPP
