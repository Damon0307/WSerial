
# 包含交叉编译工具链文件
#set(CMAKE_TOOLCHAIN_FILE "/opt/luck_toolchain/arm-rockchip830-linux-uclibcgnueabihf/arm-toolchain.cmake")
 
set(CMAKE_TOOLCHAIN_FILE "/opt/arm-rockchip830-linux-uclibcgnueabihf/cross.cmake")

# 定义项目名称和语言
cmake_minimum_required(VERSION 3.10)
project(WSerial C CXX)

# 设置C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

#设置编译bin文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# 设置库文件输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

include_directories(
    ${CMAKE_SOURCE_DIR}/
)

# 添加你的源文件
add_executable(WSerial main.cpp WSerialPort.cpp)

# 如果有需要，可以添加编译选项
target_compile_options(WSerial PRIVATE -Wall -Wextra)

# 如果有需要，可以添加链接选项
target_link_libraries(WSerial PRIVATE -Wl,--as-needed)