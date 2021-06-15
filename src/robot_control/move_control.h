#ifndef __MOVE_CONTROL_H__
#define __MOVE_CONTROL_H__

#include <iostream>
#include "robot_control_base.h"

#include <geometry_msgs/PoseStamped.h> // [current_pose] 
#include <geometry_msgs/Twist.h> // [cmd_vel]
#include <std_msgs/Int32.h> // [start_flag]

namespace ZROS{

class MoveControl final: public RobotControlBase{
public:
    MoveControl(int32_t distance, float angel); 
    ~MoveControl();
    void Action() override;
    void Rollback() override;

private:
    void Init();

    //ros topic
    ros::Subscriber current_pose_sub_;
    ros::Publisher start_flag_pub_;
    ros::Publisher cmd_vel_pub_;

    geometry_msgs::Point current_pose_; //位置
    double current_yaw_;//角度

    int32_t distance_increment_ = 0;
    double angel_increment_ = 0;

    //param
    double linear_speed_; //线速度
    double angular_speed_; //角速度

    //callback
    void CurrentPoseCallback(const geometry_msgs::PoseStamped& msg);
};

}

#endif /*__MOVE_CONTROL_H__*/