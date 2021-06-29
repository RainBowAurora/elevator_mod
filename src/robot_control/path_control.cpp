#include "path_control.h"
#include <autoware_msgs/Point_id.h>

namespace  ZROS{
PathControl::PathControl(const std::string& start_point, const std::string& target_point): \
            start_point_(start_point), target_point_(target_point), delivered_(false)
{
    Init();
}

PathControl::~PathControl()
{

}

/**
 * @brief 发布从起点到终点的路径
 * 
 * @return true 执行成功[未被打断]
 * @return false 执行失败[被打断]
 */
bool PathControl::Running()
{
    zr_msgs::Point_id temp_point_id;
    temp_point_id.dest = start_point_;
    temp_point_id.origin = target_point_;

    get_point_id_pub_.publish(temp_point_id);

    do{
        ros::spinOnce(); 
        // std::cout << "Running wait deliviered = true " << std::endl;
        if(IsCancle()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }while(!delivered_.load()&&ros::ok());//等待到达终点

    return true;
}

/**
 * @brief 发布从终点到起点的路径
 * 
 * @return true 执行成功[未被打断]
 * @return false 执行失败[被打断]
 */
bool PathControl::Restore()
{
    zr_msgs::Point_id temp_point_id;
    temp_point_id.dest = target_point_;
    temp_point_id.origin = start_point_;

    get_point_id_pub_.publish(temp_point_id);

    do{       
        ros::spinOnce(); 
        // std::cout << " Restore wait deliviered = true " << std::endl;
        if(IsCancle()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }while(!delivered_.load()&&ros::ok()); //等待到达终点

    return true;
}   

/**
 * @brief 初始化一些topic
 * 
 */
void PathControl::Init()
{
    get_point_id_pub_ = ros_nh_.advertise<zr_msgs::Point_id>("/get_point_id", 1);
    delivered_sub_  = ros_nh_.subscribe("/delivered" , 10, &PathControl::DeliveredCallback, this);
}

/**
 * @brief 回调函数用来判断是否到达终点
 * 
 * @param msg true: 到达终点, false: 没有到达终点
 */
void PathControl::DeliveredCallback(const std_msgs::Bool& msg)
{
    delivered_.store(msg.data);
}

}