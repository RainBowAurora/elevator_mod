#include "elevator_control.h"
#include <time.h>

namespace ZROS{

/**
 * @brief 获取格式化时间
 * 
 * @return std::string 
 */
std::string Elevator::LocalTime(){
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return std::string(tmp);
}



} //namespace ZROS