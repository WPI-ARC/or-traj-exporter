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

#include <iosfwd>
#include <iostream>
#include <fstream>
#include <libxml2/libxml/parser.h>

using std::cout;
using std::endl;

std::string dir_name = ".";

bool fct_sort( std::pair<int,RobotAndDof> a, std::pair<int,RobotAndDof> b)
{
    return a.first < b.first;
}

template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

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

void Converter::loadTrajectoryFromFile( std::string filename, OpenraveTrajectory& traj )
{
    cout << "-------------------------------------------" << endl;
    cout << " load file : " << filename << endl;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;
    xmlChar* tmp;

    doc = xmlParseFile(filename.c_str());
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return;
    }

    root = xmlDocGetRootElement(doc);
    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return;
    }

    if (xmlStrcmp(root->name, xmlCharStrdup("trajectory")))
    {
        cout << "Document of the wrong type root node not trajectory" << endl;
        xmlFreeDoc(doc);
        return;
    }

    cur = root->xmlChildrenNode->next;

    if (xmlStrcmp(cur->name, xmlCharStrdup("configuration")))
    {
        cout << "Error : no node named configuration" << endl;
        xmlFreeDoc(doc);
        return;
    }

    std::vector< std::pair<int,RobotAndDof> > offsets;

    xmlNodePtr node =  cur->xmlChildrenNode->next;

    while( node != NULL )
    {
        //cout << xmlGetProp( node, xmlCharStrdup("name") ) << endl;
        RobotAndDof rd;

        offsets.push_back(std::make_pair(0,rd));

        tmp = xmlGetProp( node, xmlCharStrdup("offset") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().first, (char*)tmp, std::dec );
        //cout << offsets.back().first << endl;

        tmp = xmlGetProp( node, xmlCharStrdup("dof") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().second.nb_dofs, (char*)tmp, std::dec );

        std::stringstream ss( (char *)xmlGetProp( node, xmlCharStrdup("name") ) );
        std::string line;

        std::getline( ss, line, ' ' );
        offsets.back().second.type = line;

        std::getline( ss, line, ' ' );
        offsets.back().second.robot_name = line;

        node = node->next->next;
    }

    std::sort( offsets.begin(), offsets.end(), fct_sort );

    // ------------------------------------------------
    cur = cur->next->next;

    if (xmlStrcmp(cur->name, xmlCharStrdup("data")))
    {
        cout << "Error : no node named data" << endl;
        xmlFreeDoc(doc);
        return;
    }

    tmp = xmlGetProp( cur, xmlCharStrdup("count") );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        xmlFreeDoc(doc);
        return;
    }
    int count = 0;
    convert_text_to_num<int>( count, (char*)tmp, std::dec );
    //cout << count << endl;

    tmp = xmlNodeGetContent( cur );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        xmlFreeDoc(doc);
        return;
    }

    std::string configuration( (char*)(tmp) );
    std::stringstream ss( configuration );
    std::vector<double> values;
    std::string line;
    while( std::getline(ss,line,' ') )
    {
        double val;
        convert_text_to_num<double>( val, line, std::dec );
        values.push_back( val );
    }

    cout << "values.size() : " << values.size() << endl;

    xmlFreeDoc(doc);

    traj.positions.resize(count);
    traj.velocities.resize(count);
    traj.deltatime.resize(count);

    cout << "count : " << count << endl;

    int ith_value=0;
    int configuration_offset=0;

    for(int i=0;i<count;i++)
    {
        for(int k=0;k<int(offsets.size());k++)
        {
            if( offsets[k].second.type != "deltatime" &&
                offsets[k].second.robot_name != mORRobotName )
            {
                ith_value += offsets[k].second.nb_dofs;
                continue;
            }

            int start = ith_value + offsets[k].first;
            int end = ith_value + offsets[k].first + offsets[k].second.nb_dofs;

            if( end > values.size() )
            {
                cout << " name : "  <<  offsets[k].second.robot_name << ", ith_value : " << ith_value << endl;
                cout << " type : " << offsets[k].second.type << endl;
                cout << " nb of dof : " << offsets[k].second.nb_dofs << endl;
                cout << " end : "  <<   end << endl;
                cout << "ERROR Reading trajectory" << endl;
                continue;
            }

            configuration_offset += offsets[k].second.nb_dofs;

            if( offsets[k].second.type == "joint_values" )
            {
                traj.positions[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.positions[i][l++] = values[j];
                }
            }

            if( offsets[k].second.type == "joint_velocities" )
            {
                traj.velocities[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.velocities[i][l++] = values[j];
                }
            }

            if( offsets[k].second.type == "deltatime" )
            {
                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.deltatime[i] = values[l++];
                }

                ith_value += configuration_offset;
                configuration_offset = 0;
            }
        }
    }

    //cout << traj.deltatime.transpose() << endl;
    cout << "End trajectory parsing" << endl;
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

void Converter::loadTrajectoryFromFiles()
{
    if( mTrajs.empty() )
    {
        mTrajs.clear();
        mTrajs.resize(6);

        loadTrajectoryFromFile( dir_name + "movetraj0.txt", mTrajs[0] );
        loadTrajectoryFromFile( dir_name + "movetraj1.txt", mTrajs[1] );
        loadTrajectoryFromFile( dir_name + "movetraj2.txt", mTrajs[2] );
        loadTrajectoryFromFile( dir_name + "movetraj3.txt", mTrajs[3] );
        loadTrajectoryFromFile( dir_name + "movetraj4.txt", mTrajs[4] );
        loadTrajectoryFromFile( dir_name + "movetraj5.txt", mTrajs[5] );

        //setHuboJointIndicies();
        setPath();
    }
    else
    {
        cout << "Trajectory is already loaded" << endl;
    }
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

void Converter::saveToRobotSimFormat(bool config_file)
{
    std::map<std::string,int>& m_in = mMaps.or_map;
    std::map<std::string,int>& m_out = mMaps.rs_map;

    if( mToUrdf ){
        m_out = mMaps.urdf_map;
    }
    if( mFromAchFile ) {
        m_in = mMaps.ach_map;
    }

    std::list<Eigen::VectorXd>::const_iterator it;
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

void Converter::readFile( std::string filename, std::vector<Eigen::VectorXd>& values  )
{
    std::ifstream in( filename, std::ios::in );
    if (!in){
        cout << "file " << filename << " does not exist" << endl;
        return;
    }

    std::string line;

    while( std::getline( in, line ) )
    {
        Eigen::VectorXd vect( 40 ); // Hard coded number of dofs
        std::stringstream ss( line );
        std::string chunk;
        int i=0;

        while( std::getline( ss, chunk, ' ' ) )
        {
            double val;
            if( !convert_text_to_num<double>( val, chunk, std::dec ) )
            {
                cout << "conversion from text failed" << endl;
            }
            vect(i) = val;
            i++;
            //cout << val << " ";
        }
//        cout << vect.transpose();
//        cout << endl;

        if( vect.size() > 0 )
        {
            values.push_back( vect );
        }
    }
}

void Converter::concatFiles()
{
    std::vector< std::vector<Eigen::VectorXd> > values(6);
//    readFile( dir_name + "home2init.traj",  values[0] );
//    readFile( dir_name + "init2start.traj", values[1] );
//    readFile( dir_name + "start2goal.traj", values[2] );
//    readFile( dir_name + "goal2start.traj", values[3] );
//    readFile( dir_name + "start2init.traj", values[4] );
//    readFile( dir_name + "init2home.traj",  values[5] );

    readFile( dir_name + "movetraj0.traj", values[0] );
    readFile( dir_name + "movetraj1.traj", values[1] );
    readFile( dir_name + "movetraj2.traj", values[2] );
    readFile( dir_name + "movetraj3.traj", values[3] );
    readFile( dir_name + "movetraj4.traj", values[4] );
    readFile( dir_name + "movetraj5.traj", values[5] );

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
    cout << "Saved values ach traj in " << filename << endl;
    saveToRobotSimFormat();
}

int main(int argc, char** argv)
{
    bool check_map=false;
    bool concat_ach_files=false;

    for(int i=1;i<argc;i++)
    {
        if(argv[i][0] == '-')
        {
            std::string option = argv[i];

            if( option == "-ach" ) {
                concat_ach_files = true;
                dir_name = std::string(argv[i+1]) + "/";
                i++;
            }
            else if( option == "-or" || option == "-openrave" ) {
                dir_name = std::string(argv[i+1]) + "/";
                i++;
            }
            else if( option == "-check" || option == "-c" ) {
                i++;
                check_map = true;
                break;
            }
            else {
                printf("Unknown option %s\n",argv[i]);
                return 1;
            }
        }
    }

    Converter conv;

    if(check_map)
    {
        conv.checkMaps();
        return 0;
    }
    if(concat_ach_files)
    {
        conv.concatFiles();
        return 0;
    }

    conv.loadTrajectoryFromFiles();
    conv.saveToRobotSimFormat(true); // config_file
    conv.saveToRobotSimFormat(false); // traj
    return 0;
}
