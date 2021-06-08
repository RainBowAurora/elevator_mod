#include "module_argument.h"


int main(int argc, char *argv[])
{
    ZROS::ModuleArgument().ParseArgument(argc, argv);
    
    return 0;
}