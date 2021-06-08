#ifndef __TOPIC_CONFIRM_H__
#define __TOPIC_CONFIRM_H__

/**
 * @file topic_confirm.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 为了乘坐电梯，向Elevator Commander请求批准的topic
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <string>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace Confirm{


/********************DataDestinationType************************/
struct DataDestinationType{
    std::string buildingId{};
    int floor{1};

    bool operator==(const DataDestinationType& ref) const {
        return (ref.buildingId == this->buildingId &&\
                ref.floor == this->floor);
    }

    bool operator!=(const DataDestinationType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DataDestinationType& topic){
    os  <<  "buildingId: " << topic.buildingId << \
            "\nfloor: " << topic.floor;

    return os;
}


/*************************DataSourceType*******************************/
struct DataSourceType{
    std::string buildingId{};
    int floor{2};

    bool operator==(const DataSourceType& ref) const {
        return (ref.floor == this->floor && \
                ref.buildingId == this->buildingId);
    }

    bool operator!=(const DataSourceType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DataSourceType& topic){
    os  <<  "buildingId: " << topic.buildingId << \
            "\nfloor: " << topic.floor;

    return os;
}

/*************************DataType*******************************/

struct DataType{
    std::string siteId{};
    DataSourceType source;
    DataDestinationType destination;

    bool operator==(const DataType& ref) const {
        return (ref.destination == this->destination && \
                ref.siteId == this->siteId &&\
                ref.source == this->source);
    }

    bool operator!=(const DataType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const DataType& topic){
    os  <<  "siteId: " << topic.siteId << \
            "\nsource: \n" << topic.source << \
            "\ndestination: \n" << topic.destination;

    return os;
}


/*************************PayloadType*******************************/
struct PayloadType{
    std::string clientId{};
    DataType data;
    
    bool operator==(const PayloadType& ref) const {
        return (ref.data == this->data && \
                ref.clientId == this->clientId);
    }

    bool operator!=(const PayloadType& ref) const {
        return !(*this == ref);
    }
};

std::ostream& operator<<(std::ostream& os, const PayloadType& topic){
    os  <<  "clientId: " << topic.clientId << \
            "\ndata: \n" << topic.data;
    return os;
}

/*************************TopicType*******************************/
struct TopicType{
    std::string timestamp{};
    std::string authorization{};
    std::string topic{};
    PayloadType payload;

    std::string to_string()
    {
        nlohmann::json j;
        j["timestamp"] = timestamp;
        j["authorization"] = authorization;
        j["topic"] = topic;
        j["payload"]["clientId"] = payload.clientId;
        j["payload"]["data"]["siteId"] = payload.data.siteId;
        j["payload"]["data"]["source"]["buildingId"] = payload.data.source.buildingId;
        j["payload"]["data"]["source"]["floor"] = payload.data.source.floor;
        j["payload"]["data"]["destination"]["buildingId"] = payload.data.destination.buildingId;
        j["payload"]["data"]["destination"]["floor"] = payload.data.destination.floor;

        return j.dump();
    }

    void from_string(const std::string data)
    {
        nlohmann::json j = nlohmann::json::parse(data);
        timestamp = j["timestamp"];
        authorization = j["authorization"];
        topic = j["topic"];
        payload.clientId = j["payload"]["clientId"];
        payload.data.siteId = j["payload"]["data"]["siteId"];
        payload.data.source.buildingId = j["payload"]["data"]["source"]["buildingId"];
        payload.data.source.floor = j["payload"]["data"]["source"]["floor"];
        payload.data.destination.buildingId = j["payload"]["data"]["destination"]["buildingId"];
        payload.data.destination.floor = j["payload"]["data"]["destination"]["floor"];
    }
    

    bool operator==(const TopicType& ref) const {
        return (ref.topic == this->topic && \
                ref.timestamp == this->timestamp && \
                ref.payload == this->payload &&\
                ref.authorization == this->authorization);
    }

    bool operator!=(const TopicType& ref) const {
        return !(*this == ref);
    }

};

std::ostream& operator<<(std::ostream& os, const TopicType& topic){
    os  <<  "---" << \
            "\ntimestamp: " << topic.timestamp << \
            "\nauthorization: " << topic.authorization << \
            "\ntopic: " << topic.topic << \
            "\npayload: \n"  << topic.payload;

    return os;
}

std::string to_string(Confirm::TopicType& topic)
{
    return topic.to_string();
}

Confirm::TopicType from_string(const std::string data)
{
    Confirm::TopicType temp_topic;
    temp_topic.from_string(data);
    return temp_topic;
}

} // namespace Confirm

#endif /*__TOPIC_CONFIRM_H__*/