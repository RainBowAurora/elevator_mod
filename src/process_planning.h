#ifndef __PROCESS_PLANNING_H__
#define __PROCESS_PLANNING_H__

/**
 * @file work_flow.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 乘梯流程规划
 * @version 0.1
 * @date 2021-06-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <memory>

#include "common/declare_singleton.h"
#include "elevator_control/elevator_control.h"
#include "robot_control/robot_control.h"
#include "common/macro.h"
#include <ros/ros.h>

#include <zr_msgs/Ev_param.h> //啸枭你说的中间商发过来的电梯信息

namespace ZROS{

class ProcessPlanning final{
public:
    ProcessPlanning();
    ~ProcessPlanning();

    void Run();

    void GetOnEve(); //上电梯流程
    void GetOffEve(); //下电梯流程

private:
    void Init();
    void WaitingFor(const std::string& command); //阻塞获取电梯状态
    void SendStatus(const std::string& status); //上报机器人状态
    
    ros::NodeHandle ros_nh_;  
    ros::NodeHandle ros_param_; 

    ros::Subscriber target_info_sub_; //路径点信息消息

    void EvParamCallback(const zr_msgs::Ev_param& msg);//路径点解析[暂定]

    std::string client_id_;
    std::string site_id_;
    std::string target_point_id_;
    std::string mqtt_ip_;

    std::shared_ptr<ZROS::Elevator> elevator_control_;
    std::shared_ptr<ZROS::PathControl> path_control_;
    
    DISABLE_COPY_AND_ASSIGN(ProcessPlanning);
};


}

#endif /*__PROCESS_PLANNING_H__*/