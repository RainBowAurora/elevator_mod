#ifndef __PATH_CONTROL_H__
#define __PATH_CONTROL_H__

/**
 * @file path_control.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 发布路径
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <std_msgs/Bool.h>
#include "robot_control_base.h"
#include "../common/macro.h"
#include <zr_msgs/Point_id.h>

namespace ZROS{

class PathControl final: public RobotControlBase{
public:
    PathControl(const std::string& start_point, const std::string& target_point);
    ~PathControl();
    bool Running() override;
    bool Restore() override;

private:
    void Init();
    
    ros::Publisher get_point_id_pub_;
    ros::Subscriber delivered_sub_;
    const std::string start_point_; //const 标记的只读数据[线程安全]
    const std::string target_point_; //const 标记的只读数据[线程安全]
    std::atomic<bool> delivered_; //原子操作[线程安全]

    void DeliveredCallback(const std_msgs::Bool& msg);
};

}

#endif /*__PATH_CONTROL_H__*/