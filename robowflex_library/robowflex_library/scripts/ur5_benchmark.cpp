/* Author: Zachary Kingston */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/trajectory.h>

using namespace robowflex;

/* \file ur5_benchmark.cpp
 * A basic script that demonstrates benchmarking with the UR5 robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 */

static const std::string GROUP = "manipulator";
static const std::string OBJECT = "Cube3";
static const std::string END_EFFECTOR = "ee_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table.yml";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(ur5);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *ur5->getScratchState();

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("unfurl",  // Name of experiment
                          options,   // Options for internal profiler
                          5.0,       // Timeout allowed for ALL queries
                          100);      // Number of trials

    // Create the trajectory object
    auto trajectory = std::make_shared<Trajectory> (ur5, GROUP);

    // Calculate the trajectory length in the CSpace
    trajectory->fromYAMLFile(*ur5->getScratchState(),"ur5_block.yml");
    // Extract the RobotTrajectory object
    auto trayectoria = trajectory->getTrajectory();

    auto first = trayectoria->getWayPoint(0);
    auto last = trayectoria->getWayPoint(trayectoria->getWayPointCount()-1);

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr joint_request(new MotionRequestBuilder(planner, "manipulator"));
    joint_request->setStartConfiguration(first);
    joint_request->setGoalConfiguration(last);

    // Create a motion planning request with a pose goal.
    MotionRequestBuilderPtr pose_request(new MotionRequestBuilder(planner, "manipulator"));
    pose_request->setStartConfiguration(first);

    joint_request->setConfig("RRTConnect");
    experiment.addQuery("rrtconnect", scene, planner, joint_request->getRequest());
    pose_request->setConfig("RRTConnect");
    experiment.addQuery("rrtconnect", scene, planner, pose_request->getRequest());

    joint_request->setConfig("RRT");
    experiment.addQuery("rrt", scene, planner, joint_request->getRequest());
    pose_request->setConfig("RRT");
    experiment.addQuery("rrt", scene, planner, pose_request->getRequest());

    joint_request->setConfig("PRM");
    experiment.addQuery("prm", scene, planner, joint_request->getRequest());
    pose_request->setConfig("PRM");
    experiment.addQuery("prm", scene, planner, pose_request->getRequest());

    joint_request->setConfig("EST");
    experiment.addQuery("est", scene, planner, joint_request->getRequest());
    pose_request->setConfig("EST");
    experiment.addQuery("est", scene, planner, pose_request->getRequest());

    //experiment.addQuery("joint", scene, planner, joint_request);
    //experiment.addQuery("pose", scene, planner, pose_request);

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex_ur5_demo");
    output.dump(*dataset);

    return 0;
}
