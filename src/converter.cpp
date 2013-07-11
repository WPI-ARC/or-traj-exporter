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


void Converter::loadTrajectoryFromFile( std::string filename, OpenraveTrajectory& traj )
{
    cout << "-------------------------------------------" << endl;
    cout << " load file : " << filename << endl;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;
    xmlChar* tmp;

    //std::string robot_name = "Hubo";
    //std::string robot_name = "rlhuboplus";
    std::string robot_name = "drchubo-v2";

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

    //    while (cur != NULL)
    //    {
    //        cout << cur->name << endl;
    //        cur = cur->next->next;
    //    }

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
        //cout << offsets.back().second.nb_dofs << endl;

        std::stringstream ss( (char *)xmlGetProp( node, xmlCharStrdup("name") ) );
        std::string line;

        std::getline( ss, line, ' ' );
        offsets.back().second.type = line;
        //cout << offsets.back().second.type << endl;

        std::getline( ss, line, ' ' );
        offsets.back().second.robot_name = line;
        //cout << "ROBOT NAME IS : " << offsets.back().second.robot_name << endl;
	//robot_name = offsets.back().second.robot_name;

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
    // cout << configuration << endl;
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
                    offsets[k].second.robot_name != robot_name )
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
                //cout << "position : start = " << start << " , end = " << end << endl;
                //cout << traj.positions[i].transpose() << endl;
            }

            if( offsets[k].second.type == "joint_velocities" )
            {
                traj.velocities[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.velocities[i][l++] = values[j];
                }
                //cout <<  traj.velocities[i] << endl;
                //cout << "velocities : start = " << start << " , end = " << end << endl;
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
                //cout << "deltatime : start = " << start << " , end = " << end << endl;
            }
        }
    }

    cout << "End trajectory parsing" << endl;
}

void Converter::setPath()
{
    mPath.clear();

    // Wheel turning
    std::vector<int> traj_indexes(6);
    traj_indexes[0] = 0;
    traj_indexes[1] = 1;
    traj_indexes[2] = 2;
    traj_indexes[3] = 3;
    traj_indexes[4] = 4;
    traj_indexes[5] = 5;

    // Waving
    //    std::vector<int> traj_indexes(2);
    //    traj_indexes[0] = 0;
    //    traj_indexes[1] = 1;

    for(int i=0;i<int(traj_indexes.size());i++)
    {
        for(int j=0;j<int(mTrajs[traj_indexes[i]].positions.size());j++)
        {
            Eigen::VectorXd q = mTrajs[traj_indexes[i]].positions[j];

//            if( traj_indexes[i] == 1 ) {
//                //cout << "Close hands" << endl;
//                closeHuboHands( q );
//            }

            mPath.push_back( q );
        }
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
    Eigen::VectorXd q(57);

    if( mTrajs.empty() )
    {
        //std::string dir = "/home/jmainpri/workspace/drc/wpi_openrave/hubo/matlab/";
        //std::string dir = "../trajs/";

        mTrajs.clear();
        mTrajs.resize(6);

        loadTrajectoryFromFile( dir_name + "movetraj0.txt", mTrajs[0] );
        loadTrajectoryFromFile( dir_name + "movetraj1.txt", mTrajs[1] );

        // comment the following for waving
        loadTrajectoryFromFile( dir_name + "movetraj2.txt", mTrajs[2] );
        loadTrajectoryFromFile( dir_name + "movetraj3.txt", mTrajs[3] );
        loadTrajectoryFromFile( dir_name + "movetraj4.txt", mTrajs[4] );
        loadTrajectoryFromFile( dir_name + "movetraj5.txt", mTrajs[5] );

        //setHuboJointIndicies();
        setPath();

        cout << "Set start confiuration of traj 0 " << endl;
        q = mTrajs[0].positions[0];
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

void Converter::saveToRobotSimFormat()
{
    std::list<Eigen::VectorXd>::const_iterator it;
    std::ofstream s;
    std::string filename( dir_name + "robot_commands.log" );
    s.open( filename.c_str() );

    cout << "Opening save file : " << filename << endl;

    double time_on_path=0.0;

//    int size_conf=0;
//    for( std::map<std::string,int>::iterator it_map = mMaps.or_drc.begin();
//         it_map!=mMaps.or_drc.end(); it_map++ )
//    {
//        if( it_map->second != -1 )
//            size_conf++;
//    }

    //cout << "size_conf : " << size_conf << endl;

    for( it=mPath.begin(); it != mPath.end(); it++ )
    {
        Eigen::VectorXd q((*it).size()+6);

//        cout << q.size() << endl;
//        cout << (*it).size() << endl;

        for( std::map<std::string,int>::iterator it_map = mMaps.or_drc.begin();
             it_map!=mMaps.or_drc.end(); it_map++ )
        {
            if( it_map->second == -1 )
                continue;

            //cout << it_map->first << endl;
            q( mMaps.rs_drc[it_map->first] + 6 ) = (*it)( it_map->second );
        }

        s << time_on_path << "\t";
        s << q.size() << "\t";

        for( int i=0; i<q.size(); i++ )
        {
            s << q(i) << " ";
        }

        s << endl;

        time_on_path += 0.05;
    }

    cout << "Trajectory Saved!!!" << endl;
}

int main(int argc, char** argv)
{
    for(int i=1;i<argc;i++)
    {
        if(argv[i][0] == '-')
        {
            if(0==strcmp(argv[i],"-d")) {
                dir_name = std::string(argv[i+1]) + "/";
                i++;
            }
            else {
                printf("Unknown option %s",argv[i]);
                return 1;
            }
        }
    }

    Converter conv;
    conv.loadTrajectoryFromFiles();
    conv.saveToRobotSimFormat();
    return 0;
}
