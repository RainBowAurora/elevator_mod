#include <iostream>
#include <gtest/gtest.h>
#include "topic.h"

TEST(topic, robot_command){
    //Initial  RobotCommand object
    RobotCommand::TopicType robot_command;
    robot_command.timestamp = "2021-06-06 10:57:37";
    robot_command.topic = "v1/commander/{clientId}/robot-command";
    robot_command.authorization = "token";
    robot_command.payload.clientId = "{机器人制造公司}-{机器人ID}";
    robot_command.payload.data.siteId = "Site ID";
    robot_command.payload.data.buildingId = "building ID";
    robot_command.payload.data.zoneId = "Zone ID";
    robot_command.payload.data.elevatorId = "Elevator ID";
    robot_command.payload.data.sourceFloor = 1;
    robot_command.payload.data.destinationFloor = 2;
    robot_command.payload.data.commandId = "confirm-res";
    robot_command.payload.data.status = "SOURCE_FLOOR_CALL";

    //test to_string()
    std::string robot_command_str = robot_command.to_string();
    EXPECT_EQ(robot_command.to_string(), \
            RobotCommand::to_string(robot_command));

    //test from_string()
    RobotCommand::TopicType robot_command_from_str;
    robot_command_from_str.from_string(robot_command_str);
    EXPECT_EQ(robot_command, robot_command_from_str);

    //test operator!=
    robot_command_from_str.topic += "change";
    EXPECT_NE(robot_command, robot_command_from_str);

    //test copy construct
    RobotCommand::TopicType robot_command_copy(robot_command);
    EXPECT_EQ(robot_command_copy, robot_command); 
}


TEST(topic, eleator_commder){
    //Initial  ElevatorCommand object
    ElevatorCommand::TopicType elevator_command;
    elevator_command.timestamp = "2021-06-06 11:57:37";
    elevator_command.topic = "v1/commander/{clientId}/elevator-command";
    elevator_command.payload.clientId = "Commander Client ID";
    elevator_command.payload.data.siteId = "Site ID";
    elevator_command.payload.data.buildingId = "building ID";
    elevator_command.payload.data.elevatorId = "elevator ID";
    elevator_command.payload.data.sourceFloor = 1;
    elevator_command.payload.data.destinationFloor = 2;
    elevator_command.payload.data.commandId = "confirm-res"; 
    elevator_command.payload.data.status = "SOURCE_FLOOR_ARRIVED";

    //test to_string()
    std::string elevator_command_str = elevator_command.to_string();
    EXPECT_EQ(elevator_command.to_string(), \
            ElevatorCommand::to_string(elevator_command));


    //test from_string()
    ElevatorCommand::TopicType elevator_command_from_str;
    elevator_command_from_str.from_string(elevator_command_str);
    EXPECT_EQ(elevator_command, elevator_command_from_str);

    //test operator!=
    elevator_command_from_str.topic += "change";
    EXPECT_NE(elevator_command, elevator_command_from_str);

    //test copy construct
    ElevatorCommand::TopicType elevator_command_copy(elevator_command);
    EXPECT_EQ(elevator_command_copy, elevator_command); 

}

TEST(topic, confirm){
    //Initial  Confirm object
    Confirm::TopicType confirm;
    confirm.timestamp = "2021-06-08 10:57:37";
    confirm.topic = "v1/commander/confirm";
    confirm.authorization = "token";
    confirm.payload.clientId = "{机器人制造公司}-{机器人ID}";
    confirm.payload.data.siteId = "Site ID";
    confirm.payload.data.source.buildingId = "source building ID";
    confirm.payload.data.source.floor = 10086;
    confirm.payload.data.destination.buildingId = "destination building ID";
    confirm.payload.data.destination.floor = 2;

    //test to_string()
    std::string confirm_str = confirm.to_string();
    EXPECT_EQ(confirm.to_string(), \
            Confirm::to_string(confirm));

    //test from_string()
    Confirm::TopicType confirm_from_str;
    confirm_from_str.from_string(confirm_str);
    EXPECT_EQ(confirm, confirm_from_str);

    //test operator!=
    confirm_from_str.topic += "change";
    EXPECT_NE(confirm, confirm_from_str);

    //test copy construct
    Confirm::TopicType confirm_copy(confirm);
    EXPECT_EQ(confirm_copy, confirm); 
}




TEST(topic, confirm_response){
    //Initial  Confirm object
    ConfirmResponse::TopicType confirm_response;
    confirm_response.timestamp = "2021-07-01 10:57:37";
    confirm_response.topic = "v1/commander/{clientId}/confirm-res";
    confirm_response.payload.clientId = "{机器人制造公司}-{机器人ID}";
    confirm_response.payload.result.status = "SUCCESS";
    confirm_response.payload.result.detail = "REJECT";
    confirm_response.payload.data.siteId = "Site ID";
    confirm_response.payload.data.source.buildingId = "source building ID";
    confirm_response.payload.data.source.floor = 10086;
    ConfirmResponse::DestinationPathType temp_destination_path;
    temp_destination_path.buildingId = "A";
    temp_destination_path.floor = 16;
    temp_destination_path.zoneId = "a";
    confirm_response.payload.data.destinationPath.push_back(temp_destination_path);
    temp_destination_path.buildingId = "B";
    temp_destination_path.floor = 32;
    temp_destination_path.zoneId = "b";
    confirm_response.payload.data.destinationPath.push_back(temp_destination_path);
    confirm_response.payload.data.commandId = "command Id";

    //test to_string()
    std::string confirm_response_str = confirm_response.to_string();
    EXPECT_EQ(confirm_response.to_string(), \
            ConfirmResponse::to_string(confirm_response));

    //test from_string()
    ConfirmResponse::TopicType confirm_from_str;
    confirm_from_str.from_string(confirm_response_str);
    EXPECT_EQ(confirm_response, confirm_from_str);

    //test operator!=
    confirm_from_str.topic += "change";
    EXPECT_NE(confirm_response, confirm_from_str);

    //test copy construct
    ConfirmResponse::TopicType confirm_response_copy(confirm_response);
    EXPECT_EQ(confirm_response_copy, confirm_response); 
}



int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}