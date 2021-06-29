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
#include <thread>
#include <chrono>
#include "../../lib/mqtt/mqtt.h"
#include "../../lib/topic/topic.h"
#include "../common/declare_singleton.h"

namespace ZROS{

class Elevator final{
public:
    void Init();
    void SetMqttIp(const std::string& ip) { mqtt_.send_host(ip); } 
    void SetMqttPort(const int& port) { mqtt_.send_port(port); }
    void SetAuth(const std::string& username, const std::string& pwd) { mqtt_.send_username_pwd(username, pwd); }
    void SetTopicWrapper(const std::string& , const std::string& );
    void SetElevatorParams(const std::string& buildingId, const std::string& zoneId, const std::string& elevatorId, const int& source, const int& destination );
    void MessageTrigger(std::string);

    std::string ConfirmRequest(const bool& only_listen = false);
    std::string CommandRequest(const std::string& , const bool& only_listen = false, const bool& wait=true);
    // bool UpdateReceiveData(); // Update received data from mqtt to private variables
    // int GetCurrentFloor() const { /*待完成*/ }; //获取当前电梯楼层
    // void RequestResponse() { /*待完成*/ }; //请求乘梯
    // std::string GetStatus() const { /*待完成*/ }; //获取电梯状态
    
    int PubConfirm(const Confirm::PayloadType& );
    int PubRobotCommand(const RobotCommand::PayloadType& );
    int SubConfirmResponse();
    int SubElevatorCommand();
    ConfirmResponse::PayloadType GetConfirmResponse();
    ElevatorCommand::PayloadType GetElevatorCommand();

    // int message_received();
    bool confirm_res_topic_updated;
    bool elevator_command_topic_updated;
    
private:
    ZROS::Mqtt mqtt_;
    std::string clientId_;
    std::string siteId_;
    ConfirmResponse::TopicType confirm_response_;
    Confirm::TopicType confirm_;
    ElevatorCommand::TopicType elevator_command_;
    RobotCommand::TopicType robot_command_;

    std::string LocalTime();
    


    DECLARE_SINGLETON(Elevator);
};

} //namespace ZROS


#endif /*__ELEVATOR_CONTROL_H__*/