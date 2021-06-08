#ifndef __TOPIC_ELEATOR_COMMAND_H__
#define __TOPIC_ELEATOR_COMMAND_H__

/**
 * @file topic_elevator_command.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief Robot的电梯状态向Elevator Commander告知的Topic
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <nlohmann/json.hpp>

namespace ElevatorCommand{

/*****************************DataType**********************************/
struct DataType{
    std::string siteId{};
    std::string buildingId{};
    std::string elevatorId{};
    int sourceFloor{1};
    int destinationFloor{2};
    std::string commandId{};
    std::string status{};

    bool operator==(const DataType& ref) const {
        return (ref.siteId == this->siteId && \
                ref.buildingId == this->buildingId && \
                ref.elevatorId == this->elevatorId && \
                ref.sourceFloor == this->sourceFloor && \
                ref.destinationFloor == this->destinationFloor && \
                ref.commandId == this->commandId && \
                ref.status == this->status);
    }

    bool operator!=(const DataType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DataType& topic){
    os  <<  "siteId: " << topic.siteId << \
            "\nbuildingId: " << topic.buildingId << \
            "\nelevatorId: " << topic.elevatorId << \
            "\nsourceFloor: " << topic.sourceFloor << \
            "\ndestinationFloor: " << topic.destinationFloor << \
            "\ncommandId: " << topic.commandId << \
            "\nstatus: " << topic.status;
    return os;
}

/*****************************PayloadType**********************************/
struct PayloadType{
    std::string clientId{};
    DataType data;

    bool operator==(const PayloadType& ref) const {
        return (ref.clientId == this->clientId && \
                ref.data == this->data);
    }

    bool operator!=(const PayloadType& ref){
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const PayloadType& topic){
    os  <<  "clientId: " << topic.clientId << \
            "\ndata: \n" << topic.data;
    return os;
}

/*****************************TopicType**********************************/
struct TopicType{
    std::string timestamp{};
    std::string topic{};
    PayloadType payload;

    std::string to_string()
    {
        nlohmann::json j;
        j["timestamp"] = timestamp;
        j["topic"] = topic;
        j["payload"]["clientId"] = payload.clientId;
        j["payload"]["data"]["siteId"] = payload.data.siteId;
        j["payload"]["data"]["buildingId"] = payload.data.buildingId;
        j["payload"]["data"]["elevatorId"] = payload.data.elevatorId;
        j["payload"]["data"]["sourceFloor"] = payload.data.sourceFloor;
        j["payload"]["data"]["destinationFloor"] = payload.data.destinationFloor;
        j["payload"]["data"]["commandId"] = payload.data.commandId;
        j["payload"]["data"]["status"] = payload.data.status;
        return j.dump();
    }

    void from_string(const std::string data)
    {
        nlohmann::json j = nlohmann::json::parse(data);
        timestamp = j["timestamp"];
        topic = j["topic"];
        payload.clientId = j["payload"]["clientId"];
        payload.data.siteId = j["payload"]["data"]["siteId"];
        payload.data.buildingId = j["payload"]["data"]["buildingId"];
        payload.data.elevatorId = j["payload"]["data"]["elevatorId"];
        payload.data.sourceFloor = j["payload"]["data"]["sourceFloor"];
        payload.data.destinationFloor = j["payload"]["data"]["destinationFloor"];
        payload.data.commandId = j["payload"]["data"]["commandId"];
        payload.data.status = j["payload"]["data"]["status"];
    }

    bool operator==(const TopicType& ref) const {
        return (ref.payload == this->payload && \
                ref.timestamp == this->timestamp && \
                ref.topic == this->topic);
    }

    bool operator!=(const TopicType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const TopicType& topic){
    os  <<  "timestamp: " << topic.timestamp << \
            "\ntopic: " << topic.topic << \
            "\npayload: \n" << topic.payload;
    return os;
}

std::string to_string(ElevatorCommand::TopicType& topic)
{
    return topic.to_string();
}

ElevatorCommand::TopicType from_string(const std::string data)
{
    ElevatorCommand::TopicType temp_topic;
    temp_topic.from_string(data);
    return temp_topic;
}

}// namespace ElevatorCommand


#endif /*__TOPIC_ELEATOR_COMMAND_H__*/