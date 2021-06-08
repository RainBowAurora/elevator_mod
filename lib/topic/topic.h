/**
 * @file topic.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "topic_robot_command.h"
#include "topic_elevator_command.h"

#include "topic_confirm.h"
#include "topic_confirm_response.h"


#define DEFINE_STATUS(name) const std::string name = #name;

namespace TOPIC_STATUS{
    
DEFINE_STATUS(SUCCESS) //成功
DEFINE_STATUS(FAIL) //失败

DEFINE_STATUS(REJECT) //拒绝
DEFINE_STATUS(WAITING) //待机

DEFINE_STATUS(SOURCE_FLOOR_CALL) //乘坐
DEFINE_STATUS(SOURCE_FLOOR_WAITING) //等待乘坐
DEFINE_STATUS(SOURCE_FLOOR_GETTING_ON) //乘坐中
DEFINE_STATUS(SOURCE_FLOOR_GET_ON_FAILED) //乘坐失败
DEFINE_STATUS(SOURCE_FLOOR_GOT_ON) //乘坐完成
DEFINE_STATUS(DESTINATION_FLOOR_GETTING_OFF) //下电梯中
DEFINE_STATUS(DESTINATION_FLOOR_GET_OFF_FAILED) //下电梯失败
DEFINE_STATUS(DESTINATION_FLOOR_GOT_OFF)//下电梯完成
DEFINE_STATUS(DESTINATION_FLOOR_CAR_CALL)//按电梯
DEFINE_STATUS(CALL_CANCEL)//取消呼叫

}