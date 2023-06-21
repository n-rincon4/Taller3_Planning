#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

static const std::string GROUP = "manipulator";
static const std::string OBJECT = "Cube3";
static const std::string END_EFFECTOR = "ee_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table.yml";
static const std::vector<double> START = {{0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0}};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();
    ur5->setGroupState(GROUP, START);  // Stow

    // Create the trajectory object
    auto trajectory = std::make_shared<Trajectory> (ur5, GROUP);

    // Calculate the trajectory length in the CSpace
    trajectory->fromYAMLFile(*ur5->getScratchState(),"ur5_block.yml");
    auto length = trajectory->getLength();
    std::cout << "CSpace Length: "<< length << "rad" << std::endl;

    // Extract the RobotTrajectory object
    auto trayectoria = trajectory->getTrajectory();
    
    // Calculate the distance in XYZ with Forward Kinematics
    auto trajectory_length = 0.0;
    for (std::size_t index = 1; index < trayectoria->getWayPointCount(); ++index)
    {
        auto first = trayectoria->getWayPoint(index - 1);
        auto second = trayectoria->getWayPoint(index);
        auto pos1 = first.getFrameTransform(END_EFFECTOR).translation();
        auto pos2 = second.getFrameTransform(END_EFFECTOR).translation();

        double& x1 = pos1[0];
        double& x2 = pos2[0];
        double& y1 = pos1[1];
        double& y2 = pos2[1];
        double& z1 = pos1[2];
        double& z2 = pos2[2];
        
        trajectory_length += sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
    }
    std::cout << "Workspace Length: "<< trajectory_length << "m" << std::endl;

  return 1;
}
