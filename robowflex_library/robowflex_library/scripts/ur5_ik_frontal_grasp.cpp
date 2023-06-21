#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
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

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(ur5);
    IO::RobotBroadcaster bc(ur5);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(ur5);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *ur5->getScratchState();

    // Create the trajectory object to be read
    auto loaded_trajectory = std::make_shared<Trajectory> (ur5, GROUP);
    // Extract the RobotTrajectory object 
    loaded_trajectory->fromYAMLFile(*ur5->getScratchState(),"ur5_block.yml");
    auto trayectoria = loaded_trajectory->getTrajectory();

    // Create the empty trajectory object
    auto trajectory = std::make_shared<Trajectory> (ur5, GROUP);
    
    // Calculate the distance in XYZ with Forward Kinematics
    for (std::size_t index = 1; index < trayectoria->getWayPointCount(); ++index)
    {
        auto first = trayectoria->getWayPoint(index - 1);
        // Vector that contains the position of the end-effector
        auto pos_mano = first.getFrameTransform(END_EFFECTOR);
        // Calculate Inverse Kinematics to get the configuration of the pose
        // Create IK query.
        const auto query = Robot::IKQuery(GROUP, {pos_mano}, {END_EFFECTOR}, constants::ik_tolerance,
                                            constants::ik_vec_tolerance, scene);
        // Set goal configuration from IK query.
        if (not ur5->setFromIK(query))
        {
            RBX_ERROR("IK solution not found");
            return 0;
        }


        auto state = *ur5->getScratchState();
        std::cout << "State: " << state << std::endl;
        trajectory->addSuffixWaypoint(state,0);
        
    }
    // Visualize the scene in RViz.
    rviz.updateTrajectory(*trajectory);
    // Visualize the scene in RViz.
    rviz.updateScene(scene);
    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    // Clean up RViz.
    rviz.removeMarker("goal");
    rviz.updateMarkers();
    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
