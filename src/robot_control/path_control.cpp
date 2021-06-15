#include "path_control.h"
#include <autoware_msgs/Point_id.h>

/**
 * @brief 我记的啸枭说韩国的get_point_id不用输入起始点ID（自动识别当前id）
 *        只需要输入终点id就可以发布消息，这里我不知道=-=！，这里先按照旧版代码编写
 * 
 */

namespace  ZROS{
PathControl::PathControl(const int16_t& start_point, const int16_t& target_point): \
            start_point_(start_point), target_point_(target_point), delivered_(false)
{
    Init(); 
    status_ = ActionStatus::WAITING;
}

PathControl::~PathControl()
{

}

void PathControl::Action()
{
    autoware_msgs::Point_id temp_point_id;
    temp_point_id.firstpoint = start_point_;
    temp_point_id.secondpoint = target_point_;

    get_point_id_pub_.publish(temp_point_id);
    
    status_ = ActionStatus::RUNNING;
    while(!delivered_){ 
        ros::spinOnce(); 
        if(status_ == ActionStatus::CANCEL){
            Rollback();
            return;
        }
    } //等待到达终点
}

void PathControl::Rollback()
{
    autoware_msgs::Point_id temp_point_id;
    temp_point_id.firstpoint = target_point_;
    temp_point_id.secondpoint = start_point_;

    get_point_id_pub_.publish(temp_point_id);
    status_ = ActionStatus::BACKING;
    while(!delivered_){ ros::spinOnce(); } //等待到达终点
    status_ = ActionStatus::WAITING;
}

void PathControl::Init()
{
    get_point_id_pub_ = ros_nh_.advertise<autoware_msgs::Point_id>("/get_point_id", 1);
    delivered_sub_  = ros_nh_.subscribe("/delivered" , 10, &PathControl::DeliveredCallback, this);
}


void PathControl::DeliveredCallback(const std_msgs::Bool& msg)
{
    delivered_ = msg.data;
}

}