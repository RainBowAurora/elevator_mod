#include "elevator_control.h"


int main(int argc, char *argv[])
{
    Confirm::PayloadType confirm_payload;
    ConfirmResponse::PayloadType confirm_response_payload;
    RobotCommand::PayloadType robot_command_payload;
    ElevatorCommand::PayloadType elevator_command_payload;

    std::string client_id = "zhen-zr1022";
    std::string site_id = "woowa-big-office";
    std::string building_id = "1";
    std::string zone_id = "2";
    std::string elevator_id = "2";
    int source_floor = 1;
    int destination_floor = 5;
    
    // auto elevator_control = ZROS::Elevator::Instance();
    std::shared_ptr<ZROS::Elevator> elevator_control(ZROS::Elevator::Instance());

    elevator_control->SetMqttPort(8883);
    elevator_control->SetMqttIp("elevator-commander-broker.robot.beta.baemin.com");
    elevator_control->SetAuth("robot", "l*K2qo@7a&SToP");
    elevator_control->Init(); //必须先设置ip和端口在初始化

    elevator_control->SetTopicWrapper(client_id, site_id);
    elevator_control->SetElevatorParams(building_id, zone_id, elevator_id, source_floor, destination_floor);
    elevator_control->SubConfirmResponse();
    elevator_control->SubElevatorCommand();
    
/*************************Confirm EV*******************************/

    std::string confirm_res_status = elevator_control->ConfirmRequest();

    if(confirm_res_status == CONFIRM_STATUS::SUCCESS)
    {
        std::cout << "Confirm succeed. CommandId set. \n" ;
    }
    else
    {
        std::cout << elevator_control->GetConfirmResponse().result.detail << std::endl;
    }

/*************************Ideal EV Flow*******************************/
    std::this_thread::sleep_for(std::chrono::seconds(5)); //moving
    std::string elevator_command_status;

    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_CALL);
    // only 2 cases for SOURCE_FLOOR_CALL
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_CAR_DISPATCHED)
    {
        // move to assigned elevator
        std::cout << "Move to elevator: " << elevator_control->GetElevatorCommand().data.elevatorId << std::endl; 
        std::this_thread::sleep_for(std::chrono::seconds(5)); //moving
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control->GetElevatorCommand().data.status << std::endl;
        return 0;
    }

    CAR_DISPATCHED:
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_WAITING);
    // 3 cases for SOURCE_FLOOR_WAITING
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_CAR_DISPATCHED)
    {
        // move to assigned elevator
        std::cout << "Move to elevator: " << elevator_control->GetElevatorCommand().data.elevatorId << std::endl; 
        std::this_thread::sleep_for(std::chrono::seconds(5)); //moving
        goto CAR_DISPATCHED;
    }
    else if(elevator_command_status == ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_ARRIVED)
    {
        // Elevator arrived
        std::cout << "Elevator arrived: " << elevator_control->GetElevatorCommand().data.status << std::endl; 
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control->GetElevatorCommand().data.status << std::endl;
        return 0;
    }

    // How long is the period between SOURCE_FLOOR_ARRIVED and SOURCE_FLOOR_GET_ON ?
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_WAITING, true);
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::SOURCE_FLOOR_GET_ON)
    {
        // Start getting on
        std::cout << "Elevator arrived: " << elevator_control->GetElevatorCommand().data.status << std::endl; 
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control->GetElevatorCommand().data.status << std::endl;
        return 0;
    }

    // Get into elevator
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GETTING_ON, false, false);
    // while(!elevator_control->elevator_command_topic_updated) moving on;
    std::this_thread::sleep_for(std::chrono::seconds(5)); //moving
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GOT_ON);
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::DESTINATION_FLOOR_CALL_CONFIRMED)
    {
        // Destination floor confirmed
        std::cout << "Floor confirmed: " << elevator_control->GetElevatorCommand().data.status << std::endl; 
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control->GetElevatorCommand().data.status << std::endl;
        return 0;
    }

    // Waiting arrive destination floor and get off
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::SOURCE_FLOOR_GOT_ON, true);
    if(elevator_command_status == ELEVATOR_COMMAND_STATUS::DESTINATION_FLOOR_GET_OFF)
    {
        // Destination floor arrived and start getting off
        std::cout << "Floor arrived: " << elevator_control->GetElevatorCommand().data.status << std::endl; 
    }
    else
    {
        // usually CALL_CANCELED
        std::cout << elevator_control->GetElevatorCommand().data.status << std::endl;
    }

    // Get off elevator
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::DESTINATION_FLOOR_GETTING_OFF, false, false);
    // while(!elevator_control->elevator_command_topic_updated) moving on;
    std::this_thread::sleep_for(std::chrono::seconds(5)); //moving
    elevator_command_status = elevator_control->CommandRequest(ROBOT_COMMAND_STATUS::DESTINATION_FLOOR_GOT_OFF, false, false);

    std::cout << "Robot got off the elevator. EV flow finished. \n";

/*************************EV interaction*******************************/

    return 0;
}