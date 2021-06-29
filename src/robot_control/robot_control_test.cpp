#include "robot_control_base.h"
#include "path_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "elevator_control_test");
    using namespace ZROS;

    RobotControlBase* path = Instance<PathControl>("begin", "end"); // get_point_id from 1 to 20 
    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status

    path->RunAction(true); //publish get_point_id
    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status

    path->Cancel(); //cancel action
    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status

    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status
    path->Rollback(true); //rollback
    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status

    while(ros::ok()){
        ros::Duration(2).sleep();
        ROS_INFO("main running!!");
    }
    std::cout << "status: " << path->GetStatus() << std::endl; //get action Status

    return 0;
}