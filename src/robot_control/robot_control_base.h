#ifndef __ROBOT_CONTROL_BASE_H__
#define __ROBOT_CONTROL_BASE_H__

#include <iostream>

#include <ros/ros.h>

namespace ZROS{

enum ActionStatus{
    WAITING, //等待
    RUNNING, //执行中
    CANCEL,  //撤销
    BACKING, //回退中
    COMPLETION //完成
};

class RobotControlBase{
public:
    RobotControlBase() = default;
    virtual ~RobotControlBase() = default;
    ActionStatus GetStatus() const { return status_; }
    virtual void Action() = 0;
    virtual void Rollback() = 0;
    void Cancel() { status_ = ActionStatus::CANCEL; }

protected:
    ros::NodeHandle ros_nh_;
    ros::NodeHandle ros_param_;    
    ActionStatus status_;
};

template<typename T, typename... Args>
T* Instance(Args&&... args){
    return new T(std::forward<Args>(args)...);
}

} //namespace ZROS

#endif /*__ROBOT_CONTROL_BASE_H__*/