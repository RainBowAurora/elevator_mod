# 韩国电梯节点
```
      ___           ___           ___           ___     
     /\  \         /\  \         /\  \         /\  \    
     \:\  \       /::\  \       /::\  \       /::\  \   
      \:\  \     /:/\:\  \     /:/\:\  \     /:/\ \  \  
       \:\  \   /::\~\:\  \   /:/  \:\  \   _\:\~\ \  \ 
 _______\:\__\ /:/\:\ \:\__\ /:/__/ \:\__\ /\ \:\ \ \__\
 \::::::::/__/ \/_|::\/:/  / \:\  \ /:/  / \:\ \:\ \/__/
  \:\~~\~~        |:|::/  /   \:\  /:/  /   \:\ \:\__\  
   \:\  \         |:|\/__/     \:\/:/  /     \:\/:/  /  
    \:\__\        |:|  |        \::/  /       \::/  /   
     \/__/         \|__|         \/__/         \/__/  
```
---
## 目录
```
elevator_mod
├── CMakeLists.txt
├── include
│   └── elevator_mod
├── launch
├── lib
│   ├── mqtt  #封装mosquitto
│   │   ├── CMakeLists.txt
│   │   ├── mqtt.cpp
│   │   ├── mqtt.h
│   │   └── mqtt_test.cpp
│   └── topic #协议
│       ├── CMakeLists.txt
│       ├── topic_confirm.h
│       ├── topic_confirm_response.h
│       ├── topic_elevator_command.h
│       ├── topic.h
│       ├── topic_robot_command.h
│       └── topic_test.cpp
├── package.xml
├── REAMDE.md
└── src
    ├── CMakeLists.txt
    ├── declare_singleton.h
    ├── define_type_trait.h
    ├── elevator_control  # 电梯控制
    │   ├── elevator_control.cpp
    │   ├── elevator_control.h
    │   └── elevator_control_test.cpp
    └── module_argument # 模块参数解析（用于快速测试）
        ├── module_argument.cpp
        ├── module_argument.h
        └── module_argument_test.cpp
```
## 依赖
1. [mosquitto(必须)](https://github.com/eclipse/mosquitto)
```
sudo apt-get install xsltproc
sudo apt-get install docbook-xsl

git clone https://github.com.cnpmjs.org/eclipse/mosquitto.git
cd mosquitto 
mkdir build && cd build
cmake .. 
make && sudo make install 

sudo ln -s /usr/local/lib/libmosquittopp.so.1 ./libmosquittopp.so.1
sudo ln -s /usr/local/lib/libmosquitto.so.1 /usr/lib/libmosquitto.so.1
```

2. [json(必须)](https://github.com/nlohmann/json)
```
git clone https://github.com.cnpmjs.org/nlohmann/json.git
cd json
mkdir build && build
cmake ..
make && sudo make install
```

3. [gtest(非必须)](https://github.com/google/googletest/tree/master/googletest.git)
```
git clone https://github.com/google/googletest/tree/master/googletest.git
cd googletest
mkdir build && cd build
cmake ..
make && sudo make install
```

