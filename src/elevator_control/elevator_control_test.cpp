#include "elevator_control.h"


int main(int argc, char *argv[])
{

    auto elevator_control = ZROS::Elevator::Instance();

    elevator_control->SetMqttPort(1883);
    elevator_control->SetMqttIp("");
    elevator_control->Init(); //必须先设置ip和端口在初始化

    
    return 0;
}