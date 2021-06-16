#include "robot_control_base.h"
#include "move_control.h"
#include "path_control.h"


int main(int argc, char *argv[])
{
    using namespace ZROS;
    RobotControlBase* move_1 = Instance<MoveControl>(1, 0);
    RobotControlBase* move_2 = Instance<MoveControl>(-1, 0);

    RobotControlBase* move_3 = Instance<MoveControl>(0, 1); 
    RobotControlBase* move_4 = Instance<MoveControl>(0, -1);

    move_1->Action();
    move_1->Cancel();
    move_1->GetStatus();
    move_1->Rollback();

    RobotControlBase* path = Instance<PathControl>(1, 20); // get_point_id from 1 to 20 
    path->Action(); //publish get_point_id
    path->Cancel(); //cancel action
    path->GetStatus(); //get action Status
    path->Rollback(); //rollback

    return 0;
}