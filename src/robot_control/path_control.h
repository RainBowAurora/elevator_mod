#ifndef __PATH_CONTROL_H__
#define __PATH_CONTROL_H__

#include <iostream>
#include <std_msgs/Bool.h>
#include "robot_control_base.h"

namespace ZROS{

class PathControl final: public RobotControlBase{
public:
    PathControl(const int16_t& start_point, const int16_t& target_point);
    ~PathControl();
    void Action() override;
    void Rollback() override;

private:
    void Init();
    
    ros::Publisher get_point_id_pub_;
    ros::Subscriber delivered_sub_;
    int16_t start_point_;
    int16_t target_point_;
    bool delivered_;

    void DeliveredCallback(const std_msgs::Bool& msg);
};

}

#endif /*__PATH_CONTROL_H__*/