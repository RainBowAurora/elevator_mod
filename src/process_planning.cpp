#include "process_planning.h"

namespace ZROS{

ProcessPlanning::ProcessPlanning(): ros_nh_(), ros_param_("~")
{
    Init();
}

ProcessPlanning::~ProcessPlanning()
{

}

/**
 * @brief 运行一些周期性任务
 * 
 */
void ProcessPlanning::Run()
{
    while(ros::ok()){
        //do Something ...
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
}

/**
 * @brief 等待对应电梯发送的状态
 * 
 * @param command 
 */
void ProcessPlanning::WaitingFor(const std::string& command){
    do{
        elevator_control_->SubElevatorCommand();
        ros::Duration(0.5).sleep(); //sleep 0.5s
    }while(elevator_control_->GetElevatorCommand().data.status != command);
}

/**
 * @brief 机器人状态切换
 * 
 * @param status 
 */
void ProcessPlanning::SendStatus(const std::string& status)
{
    elevator_control_->CommandRequest(status);
}

/**
 * @brief 初始化一些mqtt 和rostopic
 * 
 */
void ProcessPlanning::Init()
{
    target_info_sub_ = ros_nh_.subscribe("target_point_info", 1, &ProcessPlanning::EvParamCallback, this);
    //ROS get param from launch
    ros_param_.param<std::string>("client_id", client_id_, "zhen-zr1022");
    ros_param_.param<std::string>("site_id", site_id_, "woowa-big-office");
    ros_param_.param<std::string>("mqtt_ip", mqtt_ip_, "elevator-commander-broker.robot.beta.baemin.com");

    ros_param_.getParam("/elevator_mod/client_id", client_id_);
    ros_param_.getParam("site_id", site_id_);
    ros_param_.getParam("mqtt_ip", mqtt_ip_);

    ROS_INFO("client_id = %s", client_id_.c_str());
    ROS_INFO("site_id = %s", site_id_.c_str());
    ROS_INFO("mqtt_ip = %s", mqtt_ip_.c_str());

    /**************************Elevator communication Init****************************/
    elevator_control_ = std::shared_ptr<ZROS::Elevator>(ZROS::Elevator::Instance()) ;
    elevator_control_->SetMqttPort(8883);
    elevator_control_->SetMqttIp(mqtt_ip_);
    elevator_control_->SetAuth("robot", "l*K2qo@7a&SToP");
    elevator_control_->Init(); //必须先设置ip和端口在初始化
    elevator_control_->SetTopicWrapper(client_id_, site_id_);
}

/**
 * @brief 设置电梯通信必要的一些参数
 * 
 * @param msg 
 */
void ProcessPlanning::EvParamCallback(const zr_msgs::Ev_param& msg)
{
    elevator_control_->SetElevatorParams(msg.buildingId, msg.zoneId, msg.elevatorId, msg.sourceFloor, msg.destinationFloor);
}

/**
 * @brief 乘坐电梯流程[未完,不知道乘梯失败该怎么判断....]
 * 
 */
void ProcessPlanning::GetOnEve()
{
    /*************************Confirm EV*******************************/
    if(elevator_control_->ConfirmRequest() == CONFIRM_STATUS::SUCCESS) //预约电梯
    {
        std::cout << "Confirm succeed. CommandId set. \n" ;
    }
    else
    {
        std::cout << elevator_control_->GetConfirmResponse().result.detail << std::endl;
    }

    /*向调度电梯前移动*/
    (path_control_ = std::shared_ptr<ZROS::PathControl>(Instance<ZROS::PathControl>("start_point", "end_point")))->RunAction(true); //非阻塞

    /*************************Ideal EV Flow*******************************/
    auto elevator_command_status = elevator_control_->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_CALL);//呼叫电梯
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_CAR_DISPATCHED)
    {
        // move to assigned elevator
        std::cout << "Move to elevator: " << elevator_control_->GetElevatorCommand().data.elevatorId << std::endl; 
        /*向调度电梯前移动*/
        if(path_control_ != nullptr && path_control_->GetStatus() == ActionStatus::RUNNING) path_control_->Cancel(); //取消上一个没有执行完的路径任务
        (path_control_ = std::shared_ptr<ZROS::PathControl>(Instance<ZROS::PathControl>("start_point", "end_point")))->RunAction(true); //非阻塞
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control_->GetElevatorCommand().data.status << std::endl;
        return;
    }

    while(!path_control_->IsFinish()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); } //移动至调度的电梯前
    SendStatus(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_WAITING);//发送等待乘坐
    
    WaitingFor(ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_ARRIVED);
    WaitingFor(ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_GET_ON);

    SendStatus(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GETTING_ON);//发送乘坐中...
    /*移动到电梯里*/
    (path_control_ = std::shared_ptr<ZROS::PathControl>(Instance<ZROS::PathControl>("start_point", "end_point")))->RunAction(false); //阻塞
    SendStatus(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GOT_ON);//发送乘坐完成

}

/**
 * @brief 下电梯的一些流程[未完,不知道乘梯失败该怎么判断....]
 * 
 */
void ProcessPlanning::GetOffEve()
{
    WaitingFor(ELEVATOR_COMMAND_STATUS::DESTINATION_FLOOR_GET_OFF);
    SendStatus(ROBOT_COMMAND_STATUS::DESTINATION_FLOOR_GETTING_OFF);//发送下电梯中...
    (path_control_ = std::shared_ptr<ZROS::PathControl>(Instance<ZROS::PathControl>("start_point", "end_point")))->RunAction(false); //下电梯[阻塞]
    SendStatus(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GOT_ON);//下电梯完成坐完成

}   

}