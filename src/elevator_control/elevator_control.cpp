#include "elevator_control.h"
#include <time.h>

namespace ZROS{

void Elevator::Init(){
    this->mqtt_.init(); 
    this->mqtt_.set_trigger(std::bind(&Elevator::MessageTrigger, this, std::placeholders::_1)); 
    this->confirm_res_topic_updated = false; 
    this->elevator_command_topic_updated = false;
}

/**
 * @brief 获取格式化时间
 * 
 * @return std::string 
 */
std::string Elevator::LocalTime(){
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::milliseconds ms_t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return std::string(tmp) + "." + std::to_string((int)ms_t.count()%1000);
}

void Elevator::SetTopicWrapper(const std::string& clientId, const std::string& siteId){
    this->clientId_ = clientId;
    this->siteId_ = siteId;

    this->confirm_.topic = "v1/commander/confirm";
    this->confirm_.authorization = "zhen";
    this->confirm_.payload.clientId = clientId;
    this->confirm_.payload.data.siteId = siteId;

    this->confirm_response_.topic = "v1/commander/" + clientId + "/confirm-res";

    this->robot_command_.topic = "v1/commander/" + clientId + "/robot-command";
    this->robot_command_.authorization = "zhen";
    this->robot_command_.payload.clientId = clientId;
    this->robot_command_.payload.data.siteId = siteId;

    this->elevator_command_.topic = "v1/commander/" + clientId + "/elevator-command";
}

void Elevator::SetElevatorParams(const std::string& buildingId, const std::string& zoneId, const std::string& elevatorId, const int& source, const int& destination ){
    this->confirm_.payload.data.source.buildingId = buildingId;
    this->confirm_.payload.data.source.floor = source;
    this->confirm_.payload.data.destination.buildingId = buildingId;
    this->confirm_.payload.data.destination.floor = destination;

    this->robot_command_.payload.data.elevatorId = elevatorId;
    this->robot_command_.payload.data.zoneId = zoneId;
}

void Elevator::MessageTrigger(std::string topic){

    if(topic == this->confirm_response_.topic)
    {
        this->confirm_response_.from_string(this->mqtt_.get_revice_data(this->confirm_response_.topic));
        if(this->confirm_response_.payload.result.status == CONFIRM_STATUS::SUCCESS)
        {
            // std::cout << "Command ID: " << confirm_response_payload.data.commandId << std::endl;
            this->robot_command_.payload.data.commandId = confirm_response_.payload.data.commandId;
            this->robot_command_.payload.data.zoneId = confirm_response_.payload.data.destinationPath[0].zoneId;
            this->robot_command_.payload.data.elevatorId = confirm_response_.payload.data.destinationPath[0].zoneId;
            this->robot_command_.payload.data.buildingId = confirm_response_.payload.data.destinationPath[0].buildingId;
            this->robot_command_.payload.data.destinationFloor = confirm_response_.payload.data.destinationPath[0].floor;
            this->robot_command_.payload.data.sourceFloor = confirm_response_.payload.data.source.floor;
        }

        this->confirm_res_topic_updated = true;
    }

    if(topic == this->elevator_command_.topic)
    {
        this->elevator_command_.from_string(this->mqtt_.get_revice_data(this->elevator_command_.topic));
        this->robot_command_.payload.data.buildingId = elevator_command_.payload.data.buildingId;
        this->robot_command_.payload.data.elevatorId = elevator_command_.payload.data.elevatorId;
        this->robot_command_.payload.data.sourceFloor = elevator_command_.payload.data.sourceFloor;
        this->robot_command_.payload.data.destinationFloor = elevator_command_.payload.data.destinationFloor;

        this->elevator_command_topic_updated = true;
    }

}

/*************************Subscriber*******************************/
int Elevator::SubConfirmResponse(){
    this->mqtt_.receive_message(this->confirm_response_.topic);
}

int Elevator::SubElevatorCommand(){
    this->mqtt_.receive_message(this->elevator_command_.topic);
}

/*************************Publisher*******************************/
int Elevator::PubConfirm(const Confirm::PayloadType& payload){
    this->confirm_.payload = payload;
    this->confirm_.timestamp = this->LocalTime(); 
    return this->mqtt_.send_message(this->confirm_.to_string(), this->confirm_.topic, 1, false);
}

int Elevator::PubRobotCommand(const RobotCommand::PayloadType& payload){
    this->robot_command_.payload = payload;
    this->robot_command_.timestamp = this->LocalTime();
    std::cout << "Pub robot command: " << this->robot_command_.to_string() << std::endl;
    return this->mqtt_.send_message(this->robot_command_.to_string(), this->robot_command_.topic, 1, false);
}

/******************API for waiting and get the response data***********************/
ConfirmResponse::PayloadType Elevator::GetConfirmResponse(){
    // std::cout << "Waiting for confirm response...\n";
    // std::string confirm_res_str;

    // do{
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //     confirm_res_str = this->mqtt_.get_revice_data(this->confirm_response_.topic);
    // }while(confirm_res_str.empty());

    // std::cout << "Get confirm-res: " + confirm_res_str << std::endl;
    // this->confirm_response_.from_string(confirm_res_str);
    // // revice_data[this->confirm_response_.topic].erase();
    this->confirm_res_topic_updated = false;
    return confirm_response_.payload;
}

ElevatorCommand::PayloadType Elevator::GetElevatorCommand(){
    // std::cout << "Waiting for elevator command...\n";
    // std::string elevator_command_str;

    // do{
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //     elevator_command_str = this->mqtt_.get_revice_data(this->elevator_command_.topic);
    // }while(elevator_command_str.empty());

    // std::cout << "Get elevator command: " + elevator_command_str << std::endl;
    // this->elevator_command_.from_string(elevator_command_str);
    // // revice_data[this->elevator_command_.topic] = "";
    this->elevator_command_topic_updated = false;
    return elevator_command_.payload;
}

/******************API for EV Flow usage***********************/

std::string Elevator::ConfirmRequest(const bool& only_listen){
    // this->confirm_res_topic_updated = false;
    
    if(!only_listen) this->PubConfirm(this->confirm_.payload);

    while(! this->confirm_res_topic_updated){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return this->confirm_response_.payload.result.status;
}

std::string Elevator::CommandRequest(const std::string& upload_status, const bool& only_listen, const bool& wait){
    // this->elevator_command_topic_updated = false;
    if(!only_listen)
    {
        this->robot_command_.payload.data.status = upload_status;
        this->PubRobotCommand(this->robot_command_.payload);
    }
    
    while((! this->elevator_command_topic_updated) && wait){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return this->elevator_command_.payload.data.status;
}

} //namespace ZROS