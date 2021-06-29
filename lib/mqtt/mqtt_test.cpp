#include "mqtt.h"
#include <thread>
#include <chrono>
#include <gtest/gtest.h>

TEST(mqtt, send_and_revice){

        std::vector<std::string> topic_name = {"v1/commander/zhen-zr1022/confirm-res"};

        ZROS::Mqtt mqtt("tempconv", "elevator-commander-broker.robot.beta.baemin.com", 8883, "robot", "l*K2qo@7a&SToP");


        for(auto element: topic_name){
            mqtt.receive_message(element);    
        }

        // for(auto element: topic_name){  
        //     EXPECT_EQ(mqtt.send_message(element, element), MOSQ_ERR_SUCCESS);   
        // }
        std::string data_send = "{\"topic\": \"v1/commander/confirm\", \"timestamp\": \"2021-06-12 23:45:34.223\", \"payload\": {\"data\": {\"source\": {\"buildingId\": \"1\", \"floor\": 1}, \"destination\": {\"buildingId\": \"1\", \"floor\": 5}, \"siteId\": \"woowa-big-office\"}, \"clientId\": \"zhen-zr1022\"}, \"authorization\": \"one\"}";
        std::string send_topic = "v1/commander/confirm";
        EXPECT_EQ(mqtt.send_message(data_send, send_topic), MOSQ_ERR_SUCCESS);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // auto revice_data = mqtt.get_revice_data("v1/commander/zhen-zr1022/confirm-res");
        for(auto element: topic_name){
            auto revice_data = mqtt.get_revice_data(element);
            std::cout << revice_data << std::endl;
        }

        mqtt.disconnect();
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}