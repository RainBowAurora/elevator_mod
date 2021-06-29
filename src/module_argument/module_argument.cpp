#include "module_argument.h"

#include <getopt.h>
#include <libgen.h>
#include <unistd.h>

namespace ZROS{

/**
 * @brief 帮助提示信息
 * 
 */
void ModuleArgument::DisplayUsage(){
    std::cout   << "Usage: \n    " << binary_name_ << " [OPTION]...\n"
                << "Description: \n"
                << "    -h, --help : help information \n"
                << "    -d, --address : mqtt host to connect to. Defaults to localhost \n"
                << "    -p, --port :  -p : network port to connect to. Defaults to 1883 for plain MQTT and 8883 for MQTT over TLS \n"
                << "    -s, --status : return elevator status \n";}

void ModuleArgument::ParseArgument(int argc, char* const argv[]){
    binary_name_ = std::string(basename(argv[0])); //basename() 用于获取文件名字
    GetOptions(argc, argv);  //解析参数
}

/**
 * @brief 解析参数
 * 
 * @param argc 参数个数
 * @param argv 参数本体
 */
void ModuleArgument::GetOptions(const int argc, char* const argv[]) {
    int long_index = 0;
    const std::string short_opts = "hd:p:s";
    static const struct option long_opts[] = {
            {"help", no_argument, nullptr, 'h'},
            {"address", required_argument, nullptr, 'd'},
            {"port", required_argument, nullptr, 'p'},
            {"status", no_argument, nullptr, 's'},
            {NULL, no_argument, nullptr, 0}};

    // log command for info
    std::string cmd("");
    for (int i = 0; i < argc; ++i) {
        cmd += argv[i];
        cmd += " ";
    }
    std::cout << "command: " << cmd << std::endl;

    if (1 == argc) {
        DisplayUsage();
        exit(0);
    }

    do{
        //[GNU C提供的解析函数进行命令的解析](https://blog.csdn.net/qq_33850438/article/details/80172275)
        int opt = getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
        if (opt == -1) break;
        switch (opt) {
            case 'd':
                std::cout << "address: " << std::string(optarg) << std::endl;
                break;
            
            case 'p':
                std::cout << "port: " << std::string(optarg) << std::endl;
                break;

            case 's':
                std::cout << "status: " << "elevator status test\n";
                break; 
    
            case 'h':
                DisplayUsage();
                exit(0);
            default:
                break;
        }

    }while(true);

    if (optind < argc) {
        std::cout << "Found non-option ARGV-element \"" << argv[optind++] << "\"";
        DisplayUsage();
        exit(1);
    }

} 
} //namespace ZROS