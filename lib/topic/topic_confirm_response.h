#ifndef __TOPIC_CONFIRM_RESPONSE_H__
#define __TOPIC_CONFIRM_RESPONSE_H__

/**
 * @file topic_confirm_response.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief Elevator Commander为了应答robot的请求批准结果的Topic
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace  ConfirmResponse{

/******************DestinationPathType***********************/
struct DestinationPathType{
    std::string buildingId{};
    std::string zoneId{};
    int floor{1};
    bool operator==(const DestinationPathType& ref) const {
        return (this->buildingId == ref.buildingId &&\
                this->floor == ref.floor&& \
                this->zoneId == ref.zoneId);
    }

    bool operator!=(const DestinationPathType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DestinationPathType& topic){
    os  <<  "buildingId: " << topic.buildingId << \
            "\nzoneId: " << topic.zoneId << \
            "\nfloor: " << topic.floor;
    return os;
}

/*************************SourceType*****************************/
struct SourceType{
    std::string buildingId{};
    int floor{2};

    bool operator==(const SourceType& ref) const {
        return (this->buildingId == ref.buildingId && \
                this->floor == ref.floor);
    }

    bool operator!=(const SourceType& ref) const {
        return !(*this == ref);
    } 
};

std::ostream& operator<<(std::ostream& os, const SourceType& topic){
    os  <<  "buildingId: " << topic.buildingId << \
            "\nfloor: " << topic.floor;
    return os;
}

/************************DataType******************************/
struct DataType{
    std::string siteId{};
    SourceType source;
    std::vector<DestinationPathType> destinationPath;
    std::string commandId{};

    bool operator==(DataType ref) const {
        if(ref.destinationPath.size() != this->destinationPath.size()) return false;
        for(int i = 0; i < ref.destinationPath.size(); i++){
            if(ref.destinationPath[i] != this->destinationPath[i]) return false;
        }

        return (ref.commandId == this->commandId&&\
                ref.source == this->source&&\
                ref.commandId == this->commandId);
    }

    bool operator!=(DataType ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DataType& topic){
    os  <<  "siteId: " << topic.siteId << \
            "\nsource: \n" << topic.source << \
            "\ncommandId: " << topic.commandId;

    for(auto element: topic.destinationPath){
        os << element << "\n";
    }
    return os;
}

/**********************ResultType**************************/
struct ResultType{
    std::string status{};
    std::string detail{};
    
    bool operator==(const ResultType& ref) const {
        return (ref.detail == this->detail && \
                ref.status == this->status);
    }

    bool operator!=(const ResultType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const ResultType& topic){
    os  <<  "status: " << topic.status << \
            "\ndetail: " << topic.detail;
    return os;
}

/************************PayloadType*************************/
struct PayloadType{
    std::string clientId{};
    ResultType result{};
    DataType data;

    bool operator==(PayloadType ref) const {
        return (ref.clientId == this->clientId && \
                ref.data == this->data && \
                ref.result == ref.result);
    }

    bool operator!=(PayloadType ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const PayloadType& topic){
    os  <<  "clientId: " << topic.clientId << \
            "\nresult: \n" << topic.result << \
            "\ndata: \n" << topic.data;
    return os;
}
/**************************TopicType***************************/
struct TopicType{
    std::string timestamp{};
    std::string topic{};
    PayloadType payload;

    std::string to_string(){
        nlohmann::json j;
        j["timestamp"] = timestamp;
        j["topic"] = topic;
        j["payload"]["clientId"] = payload.clientId;
        j["payload"]["result"]["status"] = payload.result.status;
        j["payload"]["result"]["detail"] = payload.result.detail;
        j["payload"]["data"]["siteId"] = payload.data.siteId;
        j["payload"]["data"]["source"]["buildingId"] = payload.data.source.buildingId;
        j["payload"]["data"]["source"]["floor"] = payload.data.source.floor;
        j["payload"]["data"]["commandId"] = payload.data.commandId;

        for(auto element: payload.data.destinationPath){
            nlohmann::json temp;
            temp["buildingId"] = element.buildingId;
            temp["floor"] = element.floor;
            temp["zoneId"] = element.zoneId;
            j["payload"]["data"]["destinationPath"].push_back(temp);
        }
        return j.dump();
    }

    void from_string(const std::string data){
        nlohmann::json j = nlohmann::json::parse(data);
        timestamp = j["timestamp"];
        topic = j["topic"];
        payload.clientId = j["payload"]["clientId"];
        payload.result.status = j["payload"]["result"]["status"];
        payload.result.detail = j["payload"]["result"]["detail"];
        payload.data.siteId = j["payload"]["data"]["siteId"];
        payload.data.source.buildingId = j["payload"]["data"]["source"]["buildingId"];
        payload.data.source.floor = j["payload"]["data"]["source"]["floor"];
        payload.data.commandId = j["payload"]["data"]["commandId"];

        for(auto element: j["payload"]["data"]["destinationPath"]){
            ConfirmResponse::DestinationPathType temp; 
            temp.buildingId = element["buildingId"];
            temp.floor = element["floor"];
            temp.zoneId = element["zoneId"];
            payload.data.destinationPath.push_back(temp);
        }
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
    os  <<  "---"  << \
            "\ntimestamp: " << topic.timestamp << \
            "\ntopic: " << topic.topic << \
            "\npayload: \n" << topic.payload;
    return os;
}

std::string to_string(ConfirmResponse::TopicType& topic)
{
    return topic.to_string();
}

ConfirmResponse::TopicType from_string(const std::string data)
{
    ConfirmResponse::TopicType temp_topic;
    temp_topic.from_string(data);
    return temp_topic;
}

} //namespace ConfirmResponse

#endif /*__TOPIC_CONFIRM_RESPONSE_H__*/