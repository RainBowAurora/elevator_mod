#ifndef __ELEVATOR_CONTROL__
#define __ELEVATOR_CONTROL__

#include "../../lib/mqtt/mqtt.h"
#include "../../lib/topic/topic.h"
#include <string.h> //basename()

/**
 * @brief  模拟shell终端的调试方式
 * 
 */

namespace ZROS{
class ModuleArgument{
public:
    void DisplayUsage();  //显示帮助信息
    void ParseArgument(int argc, char* const argv[]); //参数解析
    void GetOptions(const int argc, char* const argv[]); //获取参数

private:
    std::string binary_name_;
};

} // namespace ZROS

#endif /*__ELEVATOR_CONTROL__*/