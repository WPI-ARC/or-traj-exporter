/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Worcester Polytechnic Institute
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jim Mainprice */

#ifndef __OR_ROBOTSIM_JOINT_MAP_HPP
#define __OR_ROBOTSIM_JOINT_MAP_HPP

#include <string>
#include <map>

namespace ortconv
{
class JointMaps
{
public:
    void setHuboPlusMaps();
    void setDrcHuboV1Maps();
    void setDrcHuboV2Maps();
    std::map<std::string,int> or_map;
    std::map<std::string,int> rs_map;
    std::map<std::string,int> ach_map;
    std::map<std::string,int> urdf_map;
};

void JointMaps::setHuboPlusMaps()
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
}


void JointMaps::setDrcHuboV1Maps()
{
    // Version 2
    or_map["LSP"]=       0;           //0         Body_Torso  Body_LSP
    or_map["LSR"]=       1;           //1         Body_LSP    Body_LSR
    or_map["LSY"]=       2;           //2         Body_LSR    Body_LSY
    or_map["LEP"]=       3;           //3         Body_LSY    Body_LEP
    or_map["LWY"]=       4;           //4         Body_LEP    Body_LWY
    or_map["LWP"]=       5;           //5         Body_LWY    Body_LWP
    or_map["LWR"]=       6;           //6         Body_LWP    Body_LWR
    or_map["TSY"]=       10;          //10        Body_Torso  Body_TSY
    or_map["LHY"]=       11;          //11        Body_TSY    Body_LHY
    or_map["LHR"]=       12;          //12        Body_LHY    Body_LHR
    or_map["LHP"]=       13;          //13        Body_LHR    Body_LHP
    or_map["LKP"]=       14;          //14        Body_LHP    Body_LKP
    or_map["LAP"]=       15;          //15        Body_LKP    Body_LAP
    or_map["LAR"]=       16;          //16        Body_LAP    Body_LAR
    or_map["NKY"]=       17;          //17        Body_Torso  Body_NKY
    or_map["NK1"]=       18;          //18        Body_NKY    Body_NK1
    or_map["NK2"]=       -1;          //19        Body_NK1    Body_NK2
    or_map["RSP"]=       26;          //26        Body_Torso  Body_RSP
    or_map["RSR"]=       27;          //27        Body_RSP    Body_RSR
    or_map["RSY"]=       28;          //28        Body_RSR    Body_RSY
    or_map["REP"]=       29;          //29        Body_RSY    Body_REP
    or_map["RWY"]=       30;          //30        Body_REP    Body_RWY
    or_map["RWP"]=       31;          //31        Body_RWY    Body_RWP
    or_map["RWR"]=       32;          //32        Body_RWP    Body_RWR
    or_map["RHY"]=       36;          //36        Body_TSY    Body_RHY
    or_map["RHR"]=       37;          //37        Body_RHY    Body_RHR
    or_map["RHP"]=       38;          //38        Body_RHR    Body_RHP
    or_map["RKP"]=       39;          //39        Body_RHP    Body_RKP
    or_map["RAP"]=       40;          //40        Body_RKP    Body_RAP
    or_map["RAR"]=       41;          //41        Body_RAP    Body_RAR

//    or_map["LF11"]=      7;           //7         Body_LWR    Body_LF11
//    or_map["LF12"]=      8;           //8         Body_LF11   Body_LF12
//    or_map["LF13"]=      9;           //9         Body_LF12   Body_LF13
//    or_map["LF21"]=      20;          //20        Body_LWR    Body_LF21
//    or_map["LF22"]=      21;          //21        Body_LF21   Body_LF22
//    or_map["LF23"]=      22;          //22        Body_LF22   Body_LF23
//    or_map["LF31"]=      23;          //23        Body_LWR    Body_LF31
//    or_map["LF32"]=      24;          //24        Body_LF31   Body_LF32
//    or_map["LF33"]=      25;          //25        Body_LF32   Body_LF33
//    or_map["RF11"]=      33;          //33        Body_RWR    Body_RF11
//    or_map["RF12"]=      34;          //34        Body_RF11   Body_RF12
//    or_map["RF13"]=      35;          //35        Body_RF12   Body_RF13
//    or_map["RF21"]=      42;          //42        Body_RWR    Body_RF21
//    or_map["RF22"]=      43;          //43        Body_RF21   Body_RF22
//    or_map["RF23"]=      44;          //44        Body_RF22   Body_RF23
//    or_map["RF31"]=      45;          //45        Body_RWR    Body_RF31
//    or_map["RF32"]=      46;          //46        Body_RF31   Body_RF32
//    or_map["RF33"]=      47;          //47        Body_RF32   Body_RF33
//    or_map["RF41"]=      48;          //48        Body_RWR    Body_RF41
//    or_map["RF42"]=      49;          //49        Body_RF41   Body_RF42
//    or_map["RF43"]=      50;          //50        Body_RF42   Body_RF43

    // Version 1
    rs_map["LHY"]= 6;
    rs_map["LHR"]= 7;
    rs_map["LHP"]= 8;
    rs_map["LKP"]= 9;
    rs_map["LAP"]= 10;
    rs_map["LAR"]= 11;
    rs_map["RHY"]= 12;
    rs_map["RHR"]= 13;
    rs_map["RHP"]= 14;
    rs_map["RKP"]= 15;
    rs_map["RAP"]= 16;
    rs_map["RAR"]= 17;
    rs_map["TSY"]= 18;
    rs_map["LSP"]= 19;
    rs_map["LSR"]= 20;
    rs_map["LSY"]= 21;
    rs_map["LEP"]= 22;
    rs_map["LWY"]= 23;
    rs_map["LWP"]= 24;
    rs_map["LWR"]= 25;
    rs_map["NKY"]= 29;
    rs_map["NK1"]= 30;
    rs_map["RSP"]= 31;
    rs_map["RSR"]= 32;
    rs_map["RSY"]= 33;
    rs_map["REP"]= 34;
    rs_map["RWY"]= 35;
    rs_map["RWP"]= 36;
    rs_map["RWR"]= 37;
    rs_map["LF1"]= 26;
    rs_map["LF2"]= 27;
    rs_map["LF3"]= 28;
    rs_map["RF1"]= 38;
    rs_map["RF2"]= 39;
    rs_map["RF3"]= 40;
}

void JointMaps::setDrcHuboV2Maps()
{
    // Version 2
    or_map["LSP"]=       0;           //0         Body_Torso  Body_LSP
    or_map["LSR"]=       1;           //1         Body_LSP    Body_LSR
    or_map["LSY"]=       2;           //2         Body_LSR    Body_LSY
    or_map["LEP"]=       3;           //3         Body_LSY    Body_LEP
    or_map["LWY"]=       4;           //4         Body_LEP    Body_LWY
    or_map["LWP"]=       5;           //5         Body_LWY    Body_LWP
    or_map["LWR"]=       6;           //6         Body_LWP    Body_LWR
    or_map["TSY"]=       10;          //10        Body_Torso  Body_TSY
    or_map["LHY"]=       11;          //11        Body_TSY    Body_LHY
    or_map["LHR"]=       12;          //12        Body_LHY    Body_LHR
    or_map["LHP"]=       13;          //13        Body_LHR    Body_LHP
    or_map["LKP"]=       14;          //14        Body_LHP    Body_LKP
    or_map["LAP"]=       15;          //15        Body_LKP    Body_LAP
    or_map["LAR"]=       16;          //16        Body_LAP    Body_LAR
    or_map["NKY"]=       17;          //17        Body_Torso  Body_NKY
    or_map["NK1"]=       18;          //18        Body_NKY    Body_NK1
    or_map["NK2"]=       -1;          //19        Body_NK1    Body_NK2
    or_map["RSP"]=       26;          //26        Body_Torso  Body_RSP
    or_map["RSR"]=       27;          //27        Body_RSP    Body_RSR
    or_map["RSY"]=       28;          //28        Body_RSR    Body_RSY
    or_map["REP"]=       29;          //29        Body_RSY    Body_REP
    or_map["RWY"]=       30;          //30        Body_REP    Body_RWY
    or_map["RWP"]=       31;          //31        Body_RWY    Body_RWP
    or_map["RWR"]=       32;          //32        Body_RWP    Body_RWR
    or_map["RHY"]=       36;          //36        Body_TSY    Body_RHY
    or_map["RHR"]=       37;          //37        Body_RHY    Body_RHR
    or_map["RHP"]=       38;          //38        Body_RHR    Body_RHP
    or_map["RKP"]=       39;          //39        Body_RHP    Body_RKP
    or_map["RAP"]=       40;          //40        Body_RKP    Body_RAP
    or_map["RAR"]=       41;          //41        Body_RAP    Body_RAR

    or_map["LF11"]=      7;           //7         Body_LWR    Body_LF11
    or_map["LF12"]=      8;           //8         Body_LF11   Body_LF12
    or_map["LF13"]=      9;           //9         Body_LF12   Body_LF13
    or_map["LF21"]=      20;          //20        Body_LWR    Body_LF21
    or_map["LF22"]=      21;          //21        Body_LF21   Body_LF22
    or_map["LF23"]=      22;          //22        Body_LF22   Body_LF23
    or_map["LF31"]=      23;          //23        Body_LWR    Body_LF31
    or_map["LF32"]=      24;          //24        Body_LF31   Body_LF32
    or_map["LF33"]=      25;          //25        Body_LF32   Body_LF33
    or_map["RF11"]=      33;          //33        Body_RWR    Body_RF11
    or_map["RF12"]=      34;          //34        Body_RF11   Body_RF12
    or_map["RF13"]=      35;          //35        Body_RF12   Body_RF13
    or_map["RF21"]=      42;          //42        Body_RWR    Body_RF21
    or_map["RF22"]=      43;          //43        Body_RF21   Body_RF22
    or_map["RF23"]=      44;          //44        Body_RF22   Body_RF23
    or_map["RF31"]=      45;          //45        Body_RWR    Body_RF31
    or_map["RF32"]=      46;          //46        Body_RF31   Body_RF32
    or_map["RF33"]=      47;          //47        Body_RF32   Body_RF33
    or_map["RF41"]=      48;          //48        Body_RWR    Body_RF41
    or_map["RF42"]=      49;          //49        Body_RF41   Body_RF42
    or_map["RF43"]=      50;          //50        Body_RF42   Body_RF43

//    // Version 2
    //rs_map["Hip"]= 5;
    //rs_map["leftFoot"]= 12;
    //rs_map["leftPalm"]= 31;
    //rs_map["rightFoot"]= 19;
    //rs_map["Torso, 5
    rs_map["LSP"]= 6;
    rs_map["LSR"]= 7;
    rs_map["LSY"]= 8;
    rs_map["LEP"]= 9;
    rs_map["LWY"]= 10;
    rs_map["LWP"]= 11;
    rs_map["LWR"]= 12;
    rs_map["LF11"]= 13;
    rs_map["LF12"]= 14;
    rs_map["LF13"]= 15;
    rs_map["LF21"]= 16;
    rs_map["LF22"]= 17;
    rs_map["LF23"]= 18;
    rs_map["LF31"]= 19;
    rs_map["LF32"]= 20;
    rs_map["LF33"]= 21;
    //leftPalm"]= 22
    rs_map["NKY"]= 23;
    rs_map["NK1"]= 24;
    rs_map["NK2"]= 25;
    rs_map["RSP"]= 26;
    rs_map["RSR"]= 27;
    rs_map["RSY"]= 28;
    rs_map["REP"]= 29;
    rs_map["RWY"]= 30;
    rs_map["RWP"]= 31;
    rs_map["RWR"]= 32;
    rs_map["RF11"]= 33;
    rs_map["RF12"]= 34;
    rs_map["RF13"]= 35;
    rs_map["RF21"]= 36;
    rs_map["RF22"]= 37;
    rs_map["RF23"]= 38;
    rs_map["RF31"]= 39;
    rs_map["RF32"]= 40;
    rs_map["RF33"]= 41;
    rs_map["RF41"]= 42;
    rs_map["RF42"]= 43;
    rs_map["RF43"]= 44;
    //rightPalm"]= 45
    rs_map["TSY"]= 46;
    rs_map["LHY"]= 47;
    rs_map["LHR"]= 48;
    rs_map["LHP"]= 49;
    rs_map["LKP"]= 50;
    rs_map["LAP"]= 51;
    rs_map["LAR"]= 52;
    //leftFoot"]= 53
    rs_map["RHY"]= 54;
    rs_map["RHR"]= 55;
    rs_map["RHP"]= 56;
    rs_map["RKP"]= 57;
    rs_map["RAP"]= 58;
    rs_map["RAR"]= 59;
    //rightFoot, 60

//    rs_map["LHY"]= 6;
//    rs_map["LHR"]= 7;
//    rs_map["LHP"]= 8;
//    rs_map["LKP"]= 9;
//    rs_map["LAP"]= 10;
//    rs_map["LAR"]= 11;
//    rs_map["RHY"]= 13;
//    rs_map["RHR"]= 14;
//    rs_map["RHP"]= 15;
//    rs_map["RKP"]= 16;
//    rs_map["RAP"]= 17;
//    rs_map["RAR"]= 18;
//    rs_map["TSY"]= 20; // Torso
//    rs_map["LSP"]= 21;
//    rs_map["LSR"]= 22;
//    rs_map["LSY"]= 23;
//    rs_map["LEP"]= 24;
//    rs_map["LWY"]= 25;
//    rs_map["LWP"]= 26;
//    rs_map["LWR"]= 27;
//    rs_map["NKY"]= 32;
//    rs_map["NK1"]= 33;
//    rs_map["RSP"]= 34;
//    rs_map["RSR"]= 35;
//    rs_map["RSY"]= 36;
//    rs_map["REP"]= 37;
//    rs_map["RWY"]= 38;
//    rs_map["RWP"]= 39;
//    rs_map["RWR"]= 40;

//    rs_map["LF1"]= 28;
//    rs_map["LF2"]= 29;
//    rs_map["LF3"]= 30;
//    rs_map["RF1"]= 41;
//    rs_map["RF2"]= 42;
//    rs_map["RF3"]= 43;
//    //rs_map["rightPalm"]= 44;

//    ach_map[""] = ;
    ach_map["RHY"] = 0;
    ach_map["RHR"] = 1;
    ach_map["RHP"] = 2;
    ach_map["RKP"] = 3; // RKN in ACH
    ach_map["RAP"] = 4;
    ach_map["RAR"] = 5;
    ach_map["LHY"] = 6;
    ach_map["LHR"] = 7;
    ach_map["LHP"] = 8;
    ach_map["LKP"] = 9; // LKN in ACH
    ach_map["LAP"] = 10;
    ach_map["LAR"] = 11;
    ach_map["RSP"] = 12;
    ach_map["RSR"] = 13;
    ach_map["RSY"] = 14;
    ach_map["REP"] = 15; // REB in ACH
    ach_map["RWY"] = 16;
    ach_map["RWR"] = 17;
    ach_map["RWP"] = 18;
    ach_map["LSP"] = 19;
    ach_map["LSR"] = 20;
    ach_map["LSY"] = 21;
    ach_map["LEP"] = 22; // LEB in ACH
    ach_map["LWY"] = 23;
    ach_map["LWR"] = 24;
    ach_map["LWP"] = 25;
    ach_map["NKY"] = 26;
    ach_map["NK1"] = 27;
    ach_map["NK2"] = 28;
    ach_map["TSY"] = 29; // WST in ACH
    ach_map["RF1"] = 30;
    ach_map["RF2"] = 31;
    ach_map["RF3"] = 32;
    ach_map["RF4"] = 33;
    ach_map["RF5"] = 34;
    ach_map["LF1"] = 35;
    ach_map["LF2"] = 36;
    ach_map["LF3"] = 37;
    ach_map["LF4"] = 38;
    ach_map["LF5"] = 39;

//    urdf_map["base0"] = 0;
//    urdf_map["base1"] = 1;
//    urdf_map["base2"] = 2;
//    urdf_map["base3"] = 3;
//    urdf_map["base4"] = 4;
//    urdf_map["Torso"]= 5;
    urdf_map["LSP"]= 6;
    urdf_map["LSR"]= 7;
    urdf_map["LSY"]= 8;
    urdf_map["LEP"]= 9;
    urdf_map["LWY"]= 10;
    urdf_map["LWP"]= 11;
    urdf_map["LWR"]= 12;
    urdf_map["LF11"]= 13;
    urdf_map["LF12"]= 14;
    urdf_map["LF13"]= 15;
    urdf_map["LF21"]= 16;
    urdf_map["LF22"]= 17;
    urdf_map["LF23"]= 18;
    urdf_map["LF31"]= 19;
    urdf_map["LF32"]= 20;
    urdf_map["LF33"]= 21;
    //urdf_map["leftPalm"]= 22;
    urdf_map["NKY"]= 23;
    urdf_map["NK1"]= 24;
    urdf_map["NK2"]= 25;
    urdf_map["RSP"]= 26;
    urdf_map["RSR"]= 27;
    urdf_map["RSY"]= 28;
    urdf_map["REP"]= 29;
    urdf_map["RWY"]= 30;
    urdf_map["RWP"]= 31;
    urdf_map["RWR"]= 32;
    urdf_map["RF11"]= 33;
    urdf_map["RF12"]= 34;
    urdf_map["RF13"]= 35;
    urdf_map["RF21"]= 36;
    urdf_map["RF22"]= 37;
    urdf_map["RF23"]= 38;
    urdf_map["RF31"]= 39;
    urdf_map["RF32"]= 40;
    urdf_map["RF33"]= 41;
    urdf_map["RF41"]= 42;
    urdf_map["RF42"]= 43;
    urdf_map["RF43"]= 44;
//    urdf_map["rightPalm"]= 45;
    urdf_map["TSY"]= 46;
    urdf_map["LHY"]= 47;
    urdf_map["LHR"]= 48;
    urdf_map["LHP"]= 49;
    urdf_map["LKP"]= 50;
    urdf_map["LAP"]= 51;
    urdf_map["LAR"]= 52;
//    urdf_map["leftFoot"]= 53;
    urdf_map["RHY"]= 54;
    urdf_map["RHR"]= 55;
    urdf_map["RHP"]= 56;
    urdf_map["RKP"]= 57;
    urdf_map["RAP"]= 58;
    urdf_map["RAR"]= 59;
//    urdf_map["rightFoot"]= 60;

};
}

#endif // JOINT_MAP_HPP
