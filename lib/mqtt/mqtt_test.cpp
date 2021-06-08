#include "mqtt.h"
#include <thread>
#include <chrono>
#include <gtest/gtest.h>

TEST(mqtt, send_and_revice){

        std::vector<std::string> topic_name = {"Robot", "Woowa", "Zhen", \
                                                "Hello", "Temp", "Elevator"};

        ZROS::Mqtt mqtt("tempconv", "82.156.45.220", 1883);


        for(auto element: topic_name){
            mqtt.receive_message(element);    
        }

        for(auto element: topic_name){  
            EXPECT_EQ(mqtt.send_message(element, element), MOSQ_ERR_SUCCESS);   
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto revice_data = mqtt.get_revice_data();
        for(auto element: topic_name){
            EXPECT_EQ(element.c_str(), revice_data[element]);
        }

        mqtt.disconnect();
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}