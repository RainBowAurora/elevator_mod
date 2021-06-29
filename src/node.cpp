#include <ros/ros.h>
#include "process_planning.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "elevator_mod");

    ZROS::ProcessPlanning process_;

    process_.Run();

    return 0;
}