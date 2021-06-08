#ifndef __ELEVATOR_CONTROL_H__
#define __ELEVATOR_CONTROL_H__

/**
 * @file elevator_control.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 电梯控制模块封装,包含通信woowa协议和mqtt
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include "../../lib/mqtt/mqtt.h"
#include "../../lib/topic/topic.h"
#include "../declare_singleton.h"

namespace ZROS{

class Elevator final{
public:
    void Init() { mqtt_.init(); }
    void SetMqttIp(const std::string& ip) { mqtt_.send_host(ip); } 
    void SetMqttPort(const int& port) { mqtt_.send_port(port); }

    int GetCurrentFloor() const { /*待完成*/ }; //获取当前电梯楼层
    void RequestResponse() { /*待完成*/ }; //请求乘梯
    std::string GetStatus() const { /*待完成*/ }; //获取电梯状态
    
private:
    ZROS::Mqtt mqtt_;
    ConfirmResponse::TopicType confirm_response_;
    Confirm::TopicType confirm_;
    ElevatorCommand::TopicType elevator_command_;
    RobotCommand::TopicType robot_command_;

    std::string LocalTime();

    DECLARE_SINGLETON(Elevator);
};

} //namespace ZROS


#endif /*__ELEVATOR_CONTROL_H__*/