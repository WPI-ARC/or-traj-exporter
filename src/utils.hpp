#ifndef ORTC_UTILS_HPP
#define ORTC_UTILS_HPP

#include <string>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

namespace ortconv
{
typedef std::vector<Eigen::VectorXd> milestones;
typedef std::vector< std::pair<double,Eigen::VectorXd> > milestones_time;

struct RobotAndDof
{
    int nb_dofs;
    std::string robot_name;
    std::string type;
};

bool fct_sort( std::pair<int,RobotAndDof> a, std::pair<int,RobotAndDof> b);

template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

template <class T>
bool convert_text_to_vect( std::istream& in, Eigen::VectorXd& v )
{
    int n;
    in >> n;
    if(!in) return in;
    if(n != v.size())
        v.resize(n);
    for(int i=0; i<v.size(); i++)
    {
        in >> v[i];
    }
    return true;
}

}
#endif // UTILS_HPP
