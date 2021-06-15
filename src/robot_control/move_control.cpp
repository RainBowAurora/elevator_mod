#include "move_control.h"
#include <tf/transform_datatypes.h>

namespace ZROS{
    MoveControl::MoveControl(int32_t distance, float angel): \
                            distance_increment_(distance),
                            angel_increment_(angel)
    {
        Init();
        status_ = ActionStatus::WAITING; //等待执行
    }

    MoveControl::~MoveControl()
    {

    }

    void MoveControl::Action()
    {
        geometry_msgs::Twist temp_twist;
        auto begin_pose = current_pose_;
        
        status_ = ActionStatus::RUNNING;

        while(ros::ok() && fabs(begin_pose.x - current_pose_.x) < distance_increment_){
            ros::spinOnce();
            temp_twist.linear.x = distance_increment_ > 0 ? linear_speed_: -linear_speed_;
            cmd_vel_pub_.publish(temp_twist);
            ros::Duration(0.1).sleep();
            if(status_ == ActionStatus::CANCEL) {
                Rollback();
                return;
            }
        }

        status_ = ActionStatus::COMPLETION;
    }

    void MoveControl::Rollback()
    {
        geometry_msgs::Twist temp_twist;
        auto begin_pose = current_pose_;
        
        status_ = ActionStatus::BACKING;

        while(ros::ok() && fabs(begin_pose.x - current_pose_.x) < distance_increment_){
            ros::spinOnce();
            temp_twist.linear.x = distance_increment_ < 0 ? linear_speed_: -linear_speed_;
            cmd_vel_pub_.publish(temp_twist);
            ros::Duration(0.1).sleep();
        }

        status_ = ActionStatus::COMPLETION;

    }

    void MoveControl::Init()
    {
        current_pose_sub_ =  ros_nh_.subscribe("/current_pose", 1, &MoveControl::CurrentPoseCallback, this);
        start_flag_pub_ = ros_nh_.advertise<std_msgs::Int32>("/start_flag", 10, true);
        cmd_vel_pub_ = ros_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);

        ros_param_.param<double>("linear_speed", linear_speed_, 0.35);
        ros_param_.param<double>("angular_speed", angular_speed_, 0.2);
    }

    void MoveControl::CurrentPoseCallback(const geometry_msgs::PoseStamped& msg)
    {
        current_pose_ = msg.pose.position;
        tf::Quaternion temp_quat;
        tf::quaternionMsgToTF(msg.pose.orientation, temp_quat);
        double temp_roll, temp_pitch, temp_yaw;
        tf::Matrix3x3(temp_quat).getRPY(temp_roll, temp_pitch, temp_yaw);//进行转换
        current_yaw_ =  temp_yaw + M_PI;
    }

}