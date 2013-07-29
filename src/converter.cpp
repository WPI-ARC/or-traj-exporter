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

#include "converter.hpp"
#include "utils.hpp"

#include <iosfwd>
#include <iostream>
#include <fstream>

using namespace ortconv;
using std::cout;
using std::endl;

static std::string dir_name = "./";
static std::string file_name = "";
static double t_alpha = 1.0;

Converter::Converter()
{
//    mORRobotName = "Hubo";
//    mORRobotName = "rlhuboplus";
    mORRobotName = "drchubo-v2";

//    mRSNbDof = 41; // V1
    mRSNbDof = 61; // V2

//    mMaps.setHuboPlusMaps();
//    mMaps.setDrcHuboV1Maps();
    mMaps.setDrcHuboV2Maps();

    mFromAchFile = false;
    mToUrdf = true;

    mDeltaTime = 0.02;
}

void Converter::addClosingHandsConfigs(const Eigen::VectorXd& q, double theta_init, double theta_end )
{
    int nb_conf = 30;

    Eigen::VectorXd q_inter = q;

    for( int i=0;i<nb_conf;i++)
    {
        double alpha = i/double(nb_conf-1);
        double theta = (1-alpha)*theta_init+alpha*theta_end;
        //cout << i << " : " << theta << endl;
        closeDRCHuboHands( q_inter, theta );
        mPath.push_back( q_inter );
    }
}

void Converter::setPath()
{
    mPath.clear();
    mTransitionIndices.clear();

    // Wheel turning
    std::vector<int> traj_indexes(6);
    traj_indexes[0] = 0; // home  -> init
    traj_indexes[1] = 1; // init  -> start
    traj_indexes[2] = 2; // start -> goal
    traj_indexes[3] = 3; // goal  -> start
    traj_indexes[4] = 4; // start -> init
    traj_indexes[5] = 5; // init  -> home

    for(int i=0;i<int(traj_indexes.size());i++)
    {
        for(int j=0;j<int(mTrajs[traj_indexes[i]].positions.size());j++)
        {
            Eigen::VectorXd q = mTrajs[traj_indexes[i]].positions[j];

            double theta = 0;

            if( traj_indexes[i] == 1 ) // start
                theta = -0.2;
            if( traj_indexes[i] == 2 ) // goal
                theta = 0.2;
            if( traj_indexes[i] >= 3 ) // back
                theta = -0.2;

            closeDRCHuboHands( q, theta ); // reset hands

            mPath.push_back( q );
        }

        if( traj_indexes[i] == 0 ) // init
        {
            addClosingHandsConfigs( mPath.back(), 0, -0.2 );
        }
        if( traj_indexes[i] == 1 ) // start
        {
            addClosingHandsConfigs( mPath.back(), -0.2, 0.2 );
        }
        if( traj_indexes[i] == 2 ) // goal
        {
            addClosingHandsConfigs( mPath.back(), 0.2, -0.2 );
        }

        cout << mPath.size()*mDeltaTime << " sec"<< endl; // Print the times of the transitions
        //printDRCHuboHands( mPath.back() );
    }
}

void Converter::setHuboJointIndicies()
{
    for(int i=0;i<int(mTrajs.size());i++)
    {
        for(int j=0;j<int(mTrajs[i].positions.size());j++)
        {
            setHuboConfiguration( mTrajs[i].positions[j], true );
        }

        for(int j=0;j<int(mTrajs[i].velocities.size());j++)
        {
            setHuboConfiguration( mTrajs[i].velocities[j], false );
        }
    }
}

std::vector<Eigen::VectorXd> Converter::loadTrajectoryFromFiles()
{
    if( mTrajs.empty() )
    {
        mTrajs.clear();
        mTrajs.resize(6);

        for( int i=0;i<(mTrajs.size());i++) // Set robot name for the parser
        {
             mTrajs[i].robot_name = mORRobotName;
        }

        mTrajs[0].loadTrajectoryFromFile( dir_name + "movetraj0.txt" );
        mTrajs[1].loadTrajectoryFromFile( dir_name + "movetraj1.txt" );
        mTrajs[2].loadTrajectoryFromFile( dir_name + "movetraj2.txt" );
        mTrajs[3].loadTrajectoryFromFile( dir_name + "movetraj3.txt" );
        mTrajs[4].loadTrajectoryFromFile( dir_name + "movetraj4.txt" );
        mTrajs[5].loadTrajectoryFromFile( dir_name + "movetraj5.txt" );

        //setHuboJointIndicies();
        setPath();
    }
    else
    {
        cout << "Trajectory is already loaded" << endl;
        setPath();
    }

    return mPath;
}

void Converter::closeHuboHands( Eigen::VectorXd& q )
{
    // THESE ARE DART INDICES

    double angle = -1.3;

    q( 33-6 ) = angle;
    q( 34-6 ) = angle;
    q( 35-6 ) = angle;
    q( 36-6 ) = angle;
    q( 37-6 ) = angle;

    q( 38-6 ) = angle;
    q( 39-6 ) = angle;
    q( 40-6 ) = angle;
    q( 41-6 ) = angle;
    q( 42-6 ) = angle;
}

void Converter::closeDRCHuboHands( Eigen::VectorXd& q, double angle )
{
    // THESE ARE OR INDICES
    //double angle = 0.3;

    q( mMaps.or_map["LF11"] ) = angle;
    q( mMaps.or_map["LF12"] ) = angle;
    q( mMaps.or_map["LF13"] ) = angle;

    q( mMaps.or_map["LF21"] ) = angle;
    q( mMaps.or_map["LF22"] ) = angle;
    q( mMaps.or_map["LF23"] ) = angle;

    q( mMaps.or_map["LF31"] ) = angle;
    q( mMaps.or_map["LF32"] ) = angle;
    q( mMaps.or_map["LF33"] ) = angle;

    q( mMaps.or_map["RF11"] ) = angle;
    q( mMaps.or_map["RF12"] ) = angle;
    q( mMaps.or_map["RF13"] ) = angle;

    q( mMaps.or_map["RF21"] ) = angle;
    q( mMaps.or_map["RF22"] ) = angle;
    q( mMaps.or_map["RF23"] ) = angle;

    q( mMaps.or_map["RF31"] ) = angle;
    q( mMaps.or_map["RF32"] ) = angle;
    q( mMaps.or_map["RF33"] ) = angle;

    q( mMaps.or_map["RF41"] ) = angle;
    q( mMaps.or_map["RF42"] ) = angle;
    q( mMaps.or_map["RF43"] ) = angle;
}

void Converter::printDRCHuboHands( const Eigen::VectorXd& q )
{
    // THESE ARE OR INDICES
    //double angle = 0.3;

    cout << "--------------" << endl;
    cout << "LF11 : " << q( mMaps.or_map["LF11"] ) << endl;
    cout << "LF12 : " << q( mMaps.or_map["LF12"] ) << endl;
    cout << "LF13 : " << q( mMaps.or_map["LF13"] ) << endl;

    cout << "LF21 : " << q( mMaps.or_map["LF21"] ) << endl;
    cout << "LF22 : " << q( mMaps.or_map["LF22"] ) << endl;
    cout << "LF23 : " << q( mMaps.or_map["LF23"] ) << endl;

    cout << "LF31 : " << q( mMaps.or_map["LF31"] ) << endl;
    cout << "LF32 : " << q( mMaps.or_map["LF32"] ) << endl;
    cout << "LF33 : " << q( mMaps.or_map["LF33"] ) << endl;

    cout << "RF11 : " << q( mMaps.or_map["RF11"] ) << endl;
    cout << "RF12 : " << q( mMaps.or_map["RF12"] ) << endl;
    cout << "RF13 : " << q( mMaps.or_map["RF13"] ) << endl;

    cout << "RF21 : " << q( mMaps.or_map["RF21"] ) << endl;
    cout << "RF22 : " << q( mMaps.or_map["RF22"] ) << endl;
    cout << "RF23 : " << q( mMaps.or_map["RF23"] ) << endl;

    cout << "RF31 : " << q( mMaps.or_map["RF31"] ) << endl;
    cout << "RF31 : " << q( mMaps.or_map["RF32"] ) << endl;
    cout << "RF33 : " << q( mMaps.or_map["RF33"] ) << endl;

    cout << "RF41 : " << q( mMaps.or_map["RF41"] ) << endl;
    cout << "RF42 : " << q( mMaps.or_map["RF42"] ) << endl;
    cout << "RF43 : " << q( mMaps.or_map["RF43"] ) << endl;
}

/// Setup hubo configuration
void Converter::setHuboConfiguration( Eigen::VectorXd& q, bool is_position )
{
    Eigen::VectorXd hubo_config(57);

    for (int i = 0; i<57; i++)
    {
        //hubo_config[i] = q[or_indices[i]];
    }

    if( is_position )
    {
        //hubo_config[7] += (21.19 * M_PI / 180.0); // lsr id : 7 = 13 - 6
        //hubo_config[8] -= (21.19 * M_PI / 180.0); // rsr id : 8 = 14 - 6

        hubo_config[37-6] = -hubo_config[37-6]; // Thumbs
        hubo_config[42-6] = -hubo_config[42-6];
    }

    q = hubo_config;
}

void Converter::saveToRobotSimFormat( const milestones_time& traj )
{
    std::ofstream s;
    std::string filename = dir_name + "robot_commands.log";
    s.open( filename.c_str() );
    typedef milestones_time::const_iterator milestone_t_ptr;
    cout << "Opening save file : " << filename << endl;
    double t = 0.0;

    for( milestone_t_ptr it=traj.begin(); it != traj.end(); it++ )
    {
        if( it==traj.begin() )
        {
            t = it->first;
        }
        else if ( it!=traj.begin() && t_alpha != 1.0 )
        {
            milestone_t_ptr it2 = it;
            milestone_t_ptr it1 = (--it);
            t = t + t_alpha*(it2->first-it1->first);
            it = it2;
        }

        s << t << "\t";
        s << it->second.size() << "\t";

        for( int i=0; i<int(it->second.size()); i++ )
        {
            s << it->second(i) << " ";
        }
        s << endl;
    }

    cout << "Trajectory Saved!!!" << endl;
}

void Converter::saveToRobotSimFormat( const std::vector<Eigen::VectorXd>& path, bool config_file )
{
    mPath = path;

    std::map<std::string,int>& m_in = mMaps.or_map;
    std::map<std::string,int>& m_out = mMaps.rs_map;

    if( mToUrdf ){
        m_out = mMaps.urdf_map;
    }
    if( mFromAchFile ) {
        m_in = mMaps.ach_map;
    }

    std::vector<Eigen::VectorXd>::const_iterator it;
    std::ofstream s;
    std::string filename;

    if( config_file )
        filename = dir_name + "robot_commands.config";
    else
        filename = dir_name + "robot_commands.log";

    s.open( filename.c_str() );

    cout << "Opening save file : " << filename << endl;

    double time_on_path=0.0;

    for( it=mPath.begin(); it != mPath.end(); it++ )
    {
        // Initializes joints to zero
        Eigen::VectorXd q(Eigen::VectorXd::Zero(mRSNbDof));

        for( std::map<std::string,int>::iterator it_map=m_in.begin(); it_map!=m_in.end(); it_map++ )
        {
            if( it_map->second == -1 )
                continue;
            if( mFromAchFile && isFinger(it_map->first) ) // No fingers in ach map
                continue;

            q( m_out[it_map->first] ) = (*it)( it_map->second );
        }

        if( !config_file )
        {
            s << time_on_path << "\t";
            time_on_path += mDeltaTime;
        }

        s << q.size() << "\t";

        for( int i=0; i<q.size(); i++ )
        {
            s << q(i) << " ";
        }
        s << endl;
    }

    cout << "Trajectory Saved!!!" << endl;
}

bool Converter::isFinger(std::string id)
{
    std::vector<std::string> fingers(21);

    fingers[0] ="LF11";           //7         Body_LWR    Body_LF11
    fingers[1] ="LF12";           //8         Body_LF11   Body_LF12
    fingers[2] ="LF13";           //9         Body_LF12   Body_LF13
    fingers[3] ="LF21";          //20        Body_LWR    Body_LF21
    fingers[4] ="LF22";          //21        Body_LF21   Body_LF22
    fingers[5] ="LF23";          //22        Body_LF22   Body_LF23
    fingers[6] ="LF31";          //23        Body_LWR    Body_LF31
    fingers[7] ="LF32";          //24        Body_LF31   Body_LF32
    fingers[8] ="LF33";          //25        Body_LF32   Body_LF33
    fingers[9] ="RF11";          //33        Body_RWR    Body_RF11
    fingers[10]="RF12";          //34        Body_RF11   Body_RF12
    fingers[11]="RF13";          //35        Body_RF12   Body_RF13
    fingers[12]="RF21";          //42        Body_RWR    Body_RF21
    fingers[13]="RF22";          //43        Body_RF21   Body_RF22
    fingers[14]="RF23";          //44        Body_RF22   Body_RF23
    fingers[15]="RF31";          //45        Body_RWR    Body_RF31
    fingers[16]="RF32";          //46        Body_RF31   Body_RF32
    fingers[17]="RF33";          //47        Body_RF32   Body_RF33
    fingers[18]="RF41";          //48        Body_RWR    Body_RF41
    fingers[19]="RF42";          //49        Body_RF41   Body_RF42
    fingers[20]="RF43";          //50        Body_RF42   Body_RF43

    for(int i=0;i<int(fingers.size());i++)
    {
        if( id == fingers[i] )
        {
            return true;
        }
    }

    return false;
}

void Converter::checkMaps()
{
    int size_or_i  =  mMaps.or_map.size();
    int size_rs_i  =  mMaps.rs_map.size();
    int size_ach_i =  mMaps.ach_map.size();
    int size_urdf_i =  mMaps.urdf_map.size();

    for( std::map<std::string,int>::iterator it_map = mMaps.or_map.begin();
         it_map!=mMaps.or_map.end(); it_map++ )
    {
        if( it_map->second == -1 )
            continue;

        cout << it_map->first  << " : " <<  mMaps.rs_map[it_map->first] << endl;
    }

    if( size_rs_i == mMaps.rs_map.size() )
    {
        cout << "OR keys are in RS map :-)" << endl;
    }

    for( std::map<std::string,int>::iterator it_map = mMaps.rs_map.begin();
         it_map!=mMaps.rs_map.end(); it_map++ )
    {
        if( it_map->second == -1 )
            continue;

        if( isFinger( it_map->first ) )
            continue;

        cout << it_map->first  << " : " <<  mMaps.ach_map[it_map->first] << endl;
    }

    if( size_ach_i == mMaps.ach_map.size() )
    {
        cout << "OR keys are in ACH map :-)" << endl;
    }

    for( std::map<std::string,int>::iterator it_map = mMaps.or_map.begin();
         it_map!=mMaps.or_map.end(); it_map++ )
    {
        if( it_map->second == -1 )
            continue;

        if( isFinger( it_map->first ) )
            continue;

        cout << it_map->first  << " : " <<  mMaps.urdf_map[it_map->first] << endl;
    }

    if( size_urdf_i == mMaps.urdf_map.size() )
    {
        cout << "OR keys are in URDF map :-)" << endl;
    }
}

void Converter::readFileAch( std::string filename, std::vector<Eigen::VectorXd>& values  )
{
    std::ifstream in( filename, std::ios::in );
    if (!in){
        cout << "file " << filename << " does not exist" << endl;
        return;
    }

    std::string line;

    while( std::getline( in, line ) )
    {
        Eigen::VectorXd vect( 40 ); // Hard coded number of dofs (ACH)
        std::stringstream ss( line );
        std::string chunk;
        int i=0;

        while( std::getline( ss, chunk, ' ' ) )
        {
            double val;
            if( !ortconv::convert_text_to_num<double>( val, chunk, std::dec ) )
            {
                cout << "conversion from text failed" << endl;
            }
            vect(i) = val;
            i++;
            //cout << val << " ";
        }
        //cout << vect.transpose();
        //cout << endl;

        if( vect.size() > 0 )
        {
            values.push_back( vect );
        }
    }
    cout << "nb of configurations : " << values.size() << endl;
}

void Converter::readFileRobotSim( std::string filename, std::vector< std::pair<double,Eigen::VectorXd> >& values  )
{
    std::ifstream in( filename.c_str() /*,std::ios::in*/ );

    if(!in) {
        cout << "Warning, couldn't open file : " << filename << endl;
        return;
    }

    double t;
    while(in) {
        std::pair<double,Eigen::VectorXd> milestone;
        in >> milestone.first;
        convert_text_to_vect<double>( in, milestone.second );
        if(in) {
            values.push_back( milestone );
        }
    }
    if(in.bad()) {
        cout << "Error during read of file : " << filename << endl;
        return;
    }
    in.close();
}

milestones Converter::concatFiles()
{
    std::vector<milestones> values(6);
//    readFile( dir_name + "home2init.traj",  values[0] );
//    readFile( dir_name + "init2start.traj", values[1] );
//    readFile( dir_name + "start2goal.traj", values[2] );
//    readFile( dir_name + "goal2start.traj", values[3] );
//    readFile( dir_name + "start2init.traj", values[4] );
//    readFile( dir_name + "init2home.traj",  values[5] );

    readFileAch( dir_name + "movetraj0.traj", values[0] );
    readFileAch( dir_name + "movetraj1.traj", values[1] );
    readFileAch( dir_name + "movetraj2.traj", values[2] );
    readFileAch( dir_name + "movetraj3.traj", values[3] );
    readFileAch( dir_name + "movetraj4.traj", values[4] );
    readFileAch( dir_name + "movetraj5.traj", values[5] );

    std::string filename = dir_name + "ach_final.traj";

    std::ofstream fout( filename, std::ios::out );

    mPath.clear();

    for(int i=0;i<int(values.size());i++)
    {
        for(int j=0;j<int(values[i].size());j++)
        {
            for(int k=0;k<int(values[i][j].size());k++)
            {
                fout << values[i][j][k];
                fout << " ";
            }
            fout << endl;

            //cout << values[i][j].maxCoeff() << endl;
            mPath.push_back( values[i][j] );
        }
    }

    return mPath;
}

void print_help()
{
    cout << "----------------------------------------------------" << endl;
    cout << "----------------------------------------------------" << endl;
    cout << endl;
    cout << "        Welcome to ORTCONV" << endl;
    cout << endl;
    cout << " This program is used to convert trajectories" << endl;
    cout << endl;
    cout << "----------------------------------------------------" << endl;
    cout << "----------------------------------------------------" << endl;
    cout << endl;
    cout << " Mandatory arguments to long options are mandatory for short options too." << endl;
    cout << endl;
    cout << "  -a,    --achconcat         concatenate ach trajectories" << endl;
    cout << "  -a2rs, --ach2robsim        convert form ach to robotsim" << endl;
    cout << "  -or,   --openrave          convert from operave xml trajectory" << endl;
    cout << "  -c,    --check             check that the joint maps are correct" << endl;
    cout << "  -d,    --directory         set the directory" << endl;
    cout << endl;
    cout << "Exit status:" << endl;
    cout << "0  if OK," << endl;
    cout << "1  if minor problems (e.g., cannot access subdirectory)," << endl;
    cout << "2  if serious trouble (e.g., cannot access command-line argument)." << endl;
}

int main(int argc, char** argv)
{
    bool display_help=false;
    bool check_map=false;
    bool concat_ach_files=false;
    bool ach_2_rs=false;
    bool openrave=false;
    bool time_scaling=false;

    for(int i=1;i<argc;i++)
    {
        if(argv[i][0] == '-')
        {
            std::string option = argv[i];

            if( option == "-h"  || option == "--help" ) {
                display_help = true;
                break;
            }
            if( option == "-a"  || option == "--achconcat" ) {
                concat_ach_files = true;
            }
            else if( option == "-or" || option == "--openrave" ) {
                openrave = true;
            }
            else if( option == "-a2rs"  || option == "--ach2robsim" ) {
                ach_2_rs = true;
            }
            else if( option == "-c" || option == "--check" ) {
                check_map = true;
            }
            else if( option == "-d"  || option == "--directory") {
                if( i+1 >= argc ) {
                    cout << "Error : no directory argument given" << endl;
                    return 1;
                }
                dir_name = std::string(argv[i+1]) + "/";
                i++;
            }
            else if( option == "-t"  || option == "--timescale") {
                if( i+1 >= argc ) {
                    cout << "Error : no timescale argument given" << endl;
                    return 1;
                }
                time_scaling = true;
                std::string alpha( argv[i+1] );
                ortconv::convert_text_to_num<double>( t_alpha, alpha, std::dec );
                i++;
            }
            else {
                printf("Unknown option %s\n",argv[i]);
                return 1;
            }
        }
    }

    Converter conv;

    if( display_help )
    {
        print_help();
        return 0;
    }
    if( time_scaling )
    {
        milestones_time traj;
        conv.readFileRobotSim( dir_name + "valve_turning.traj", traj );
        conv.saveToRobotSimFormat( traj );
        return 0;
    }
    if( ach_2_rs ) // Converts from ach to robotsim format
    {
       std::vector<Eigen::VectorXd> path;
       conv.readFileAch( file_name, path );
       conv.mFromAchFile = true;
       conv.mDeltaTime = 0.002;
       conv.saveToRobotSimFormat( path );
       return 0;
    }
    else if( check_map ) // Check joint mapping
    {
        conv.checkMaps();
        return 0;
    }
    else if( concat_ach_files ) // Concatanate ach files
    {
        milestones path = conv.concatFiles();
        conv.mFromAchFile = true;
        conv.mDeltaTime = 0.002;
        conv.saveToRobotSimFormat( path );
        return 0;
    }
    else if( openrave ) {
        milestones path = conv.loadTrajectoryFromFiles();
        conv.saveToRobotSimFormat(path,true); // config_file
        conv.saveToRobotSimFormat(path,false); // traj
        return 0;
    }

    return 1;
}
