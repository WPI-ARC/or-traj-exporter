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

#ifndef __OR_ROBOTSIM_CONVERTER__
#define __OR_ROBOTSIM_CONVERTER__

#include <string>
#include <vector>
#include <list>
#include <Eigen/Core>
#include "joint_map.hpp"
#include "or_trajectory.hpp"

namespace ortconv
{
class Converter
{
public:
    Converter();

    milestones loadTrajectoryFromFiles();
    milestones concatFiles();

    void saveToRobotSimFormat(const milestones& paths, bool config_file=false);
    void setPath();

    void addClosingHandsConfigs(const Eigen::VectorXd& q, double theta_init, double theta_end );
    void closeHuboHands( Eigen::VectorXd& q );
    void closeDRCHuboHands( Eigen::VectorXd& q, double theta );
    void printDRCHuboHands( const Eigen::VectorXd& q );
    void setHuboConfiguration( Eigen::VectorXd& q, bool is_position );
    void setHuboJointIndicies();
    void checkMaps();
    void readFile( std::string filename, milestones& values  );
    bool isFinger(std::string id);

    int mRSNbDof;
    std::string mORRobotName;
    double mDeltaTime;
    bool mFromAchFile;
    bool mToUrdf;

private:
    std::vector<int> mTransitionIndices;
    std::vector<OpenraveTrajectory> mTrajs;
    std::vector<Eigen::VectorXd> mPath;
    JointMaps mMaps;
};
}

#endif
