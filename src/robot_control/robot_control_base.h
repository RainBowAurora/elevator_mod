#ifndef __ROBOT_CONTROL_BASE_H__
#define __ROBOT_CONTROL_BASE_H__

/**
 * @file robot_control_base.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 机器人动作基类，内部包含多线程
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include "../common/macro.h"

namespace ZROS{

enum ActionStatus: std::int8_t{
    FAILD = -1,
    WAITING = 0, //等待
    RUNNING, //执行中
    CANCEL,  //撤销
    RESTORE, //回退中
    FINISH //完成
};

class RobotControlBase{
public:
    RobotControlBase(): status_(ActionStatus::WAITING), ros_nh_(), ros_param_("~") { }

    virtual ~RobotControlBase(){
        Cancel(); //状态切换为取消
        if(run_task_.joinable()) run_task_.join(); //等待多线程执行完毕
        if(rol_task_.joinable()) rol_task_.join(); //等待多线程执行完毕
    }   

    //获取动作运行状态
    ActionStatus GetStatus() const { return status_.load(); }
    
    //取消操作
    void Cancel() { status_.store(ActionStatus::CANCEL); }
    
    // 动作的运行状态
    bool IsCancle() const  { return (GetStatus() == ActionStatus::CANCEL);  }
    bool IsRunning() const { return (GetStatus() == ActionStatus::RUNNING); }
    bool IsFinish() const { return (GetStatus() == ActionStatus::FINISH); } 

    /**
     * @brief 动作执行
     * 
     * @param background true: 开启多线程[非阻塞], false: 不启用多线程[阻塞]
     */
    void RunAction(const bool background = false){
        if(background){
            run_task_ = std::thread([&]{ //开启多线程[非阻塞]
                std::lock_guard<std::mutex> lock(mutex_);
                status_.store(ActionStatus::RUNNING); 
                if(Running()){
                    status_.store(ActionStatus::FINISH);
                }else{
                    status_.store(ActionStatus::FAILD);
                }
            });
        }else{
            if(Running()){ //顺序执行非阻塞
                status_.store(ActionStatus::FINISH);
            }else{
                status_.store(ActionStatus::FAILD);
            }
        }
    }

    /**
     * @brief 动作回退
     * 
     * @param background true: 开启多线程[非阻塞], false: 不启用多线程[阻塞]
     */
    void Rollback(const bool background = false){
        if(background){
            rol_task_ = std::thread([&]{ // 开启多线程[非阻塞]
                std::lock_guard<std::mutex> lock(mutex_);
                status_.store(ActionStatus::RESTORE); 
                if(Restore()){
                    status_.store(ActionStatus::FINISH);
                }else{
                    status_.store(ActionStatus::FAILD);
                }
            });
        }else{
            if(Restore()){ //顺序执行非阻塞
                status_.store(ActionStatus::FINISH);
            }else{
                status_.store(ActionStatus::FAILD);
            }
        }
    }

private:
    std::thread run_task_; //每一个任务开一个线程[有点蠢但是目前没想到更优解...]
    std::thread rol_task_; 
    std::mutex mutex_; //用来互斥，防止RunAction()和Rollback()同时进行导致的逻辑错误

protected:
    ros::NodeHandle ros_nh_; 
    ros::NodeHandle ros_param_;    

    std::atomic<ActionStatus> status_; //原子操作，相当于std::motic<int8_t>

    virtual bool Running() = 0; //子类实现时候要严格保证此函数线程安全
    virtual bool Restore() = 0; //子类实现时候要严格保证此函数线程安全

    DISABLE_COPY_AND_ASSIGN(RobotControlBase); //禁止拷贝构造和赋值
};

/**
 * @brief 利用函数模板可以自动推导的原理，封装子类构造函数的实例化
**/
template<typename T, typename... Args>
T* Instance(Args&&... args){
    return new T(std::forward<Args>(args)...);
}

} //namespace ZROS

#endif /*__ROBOT_CONTROL_BASE_H__*/