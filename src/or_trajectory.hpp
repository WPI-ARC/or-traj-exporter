#ifndef OR_TRAJECTORY_HPP
#define OR_TRAJECTORY_HPP

#include <Eigen/Core>
#include <string>

namespace ortconv
{
class OpenraveTrajectory
{
public:
    OpenraveTrajectory();
    void loadTrajectoryFromFile( std::string filename );

    std::vector<Eigen::VectorXd> positions;
    std::vector<Eigen::VectorXd> velocities;
    Eigen::VectorXd deltatime;
    std::string robot_name;
};
}

#endif // OR_TRAJECTORY_HPP
