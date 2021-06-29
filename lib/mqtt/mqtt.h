#ifndef __MQTT_H__
#define __MQTT_H__

/**
 * @file mqtt.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <string>
#include <cstring>
#include <map>
#include <mutex>
#include <functional>

#include <mosquittopp.h>
#include <mosquitto.h>

namespace ZROS{

class Mqtt final: public mosqpp::mosquittopp{
public:
    using revice_type = std::map<std::string, std::string>;

    explicit Mqtt(const char* id, const char* host, int port, const char* username, const char* pwd);
    Mqtt() = default;
    ~Mqtt();

    void send_topic(const std::string& topic) { this->topic_ = topic;}
    void send_port(const int port) { this->port_ = port; }
    void send_host(const std::string& host) {this->host_ = host;}
    void send_username_pwd(const std::string& username, const std::string& pwd) {this->username_ = username, this->pwd_ = pwd;}
    std::string get_topic() const { return this->topic_; }
    int get_port() const { return this->port_; }
    std::string get_revice_data(const std::string& );

    void set_trigger(std::function<void(std::string)> func) { this->trigger_function = func; }

    int send_message(const std::string& message, const std::string& topic = "", const int& qos = 1, const bool& retain = false);
    std::string receive_message(const std::string& topic = "");

    void on_connect(int /*rc*/) override;
    // void on_connect_with_flags(int rc, int flags) override { std::cout << "on_connect_with_flags..\n";}
    void on_disconnect(int /*rc*/) override;
    void on_publish(int /*mid*/) override;
    void on_message(const struct mosquitto_message * /*message*/) override;
    void on_subscribe(int /*mid*/, int /*qos_count*/, const int * /*granted_qos*/) override;
    void on_unsubscribe(int /*mid*/)override;
    // void on_log(int /*level*/, const char * /*str*/);
    // void on_error();

    void init();
private:
    revice_type revice_data_;
    std::string host_ = "localhost";
    std::string id_ = "zhen-robot";
    std::string topic_ = "temp";
    std::string username_ = "robot";
    std::string pwd_ = "";
    int port_ = 1883;
    const int keeplive_ = 60;
    std::mutex revice_mutex;
    std::function<void(std::string)> trigger_function;

};

}

#endif /*__MQTT_H__*/