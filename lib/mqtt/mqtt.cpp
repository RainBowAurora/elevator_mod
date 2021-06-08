#include "mqtt.h"

namespace ZROS{

Mqtt::Mqtt(const char* id, const char* host, int port):host_(host), id_(id), port_(port)
{ 
    init();
}

void Mqtt::init(){
    switch(mosqpp::lib_init()){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] mosquitto init.\n"; break;
        case MOSQ_ERR_UNKNOWN:
            std::cout << "[ERROR] couldn’t be initialized.\n"; break;
    }

    switch(connect_async(host_.c_str(), port_, keeplive_)){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Connect to an MQTT broker.\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] input parameters were invalid.\n"; break;
        case MOSQ_ERR_ERRNO:
            std::cout << "[ERROR] system call returned an error..\n"; break;
    }

    switch(loop_start()){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Start thread managing connection publish/subscribe\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] input parameters were invalid.\n"; break;
        case MOSQ_ERR_NOT_SUPPORTED:
            std::cout << "[ERROR] thread support is not available.\n";  break;
    }
}

Mqtt::~Mqtt()
{
    loop_stop();            // Kill the thread
    mosqpp::lib_cleanup();    // Mosquitto library cleanup
}

void Mqtt::on_connect(int rc)
{
    if ( rc == 0 ) {
        std::cout << ">> Mqtt - connected with server" << std::endl;
    } else {
        std::cout << ">> Mqtt - Impossible to connect with server(" << rc << ")" << std::endl;
    }
}


void Mqtt::on_disconnect(int rc)
{
    std::cout << ">> Mqtt - disconnection(" << rc << ")" << std::endl;
}


void Mqtt::on_publish(int mid)
{
     std::cout << ">> Mqtt - Message (" << mid << ") succeed to be published " << std::endl;
}

void Mqtt::on_subscribe(int mid, int qos_count, const int * granted_qos)
{
    std::cout << ">> Mqtt - Message (" << mid << ") succeed to be subcribe" << std::endl;
}

int Mqtt::send_message(const std::string& message, const std::string& topic)
{
    this->topic_ = topic.empty()? this->topic_: topic;
    return publish(NULL, topic_.c_str(), strlen(message.c_str())+1, message.c_str());
}

void Mqtt::on_message(const struct mosquitto_message * message){
    std::cout << "message->topic: "<< message->topic << "\tmsg->qos: "  << \
                    message->qos << "\tmsg->payload: " << (char *)message->payload<< std::endl;
    {
        std::lock_guard<std::mutex> lock(revice_mutex);
        revice_data_[message->topic] = (char *)message->payload;
    }
}

std::string Mqtt::receive_message(const std::string& topic)
{
    this->topic_ = topic.empty()? this->topic_: topic;
    subscribe(NULL, this->topic_.c_str() ,2);

    return  revice_data_[topic]; 
}

void Mqtt::on_unsubscribe(int mid)
{
    std::cout << ">> Mqtt - unsubscribe (" << mid << ") succeed to be unsubcribe\n";
}

Mqtt::revice_type Mqtt::get_revice_data() { 
    std::lock_guard<std::mutex> lock(revice_mutex);
    return revice_data_; 
};

}
