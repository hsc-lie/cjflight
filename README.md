# 一、介绍
四旋翼飞控，主控MCU使用雅特力的AT32F421，RTOS使用freeRTOS。

# 二、文件夹说明
- CMSIS:ARM 标准接口
- app:应用层
- common：一些通用的模块
- dal：设备抽象层
- dev：设备层
- freeRTOS：存放freeRTOS V9源码
- hal：硬件抽象层
- hardware：硬件层
- at32f4_lib：AT32标准外设库
- linker: 编译链接文件
- protocol_stack：通讯协议栈
- startup: 启动文件


# 三、编译示例
编译生成的所有文件在build/out内
## 3.1 使用cmake构建
Linux环境下

    mkdir build
    cd build
    cmake ..
    cmake --build .

Windows环境下，以MinGW64为例

    mkdir build
    cd build
    cmake -G "MinGW Makefiles" ..
    cmake --build .

## 3.2 使用Makefile构建
在Linux环境下，或在windows使用git的命令行中运行

    make

