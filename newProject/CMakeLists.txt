# 顶层CMakeLists.txt
 
# 定义工程名称
project(TEST)
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
# 定义子目录src，用以递归的调用src中的MakeLists.txt
add_subdirectory(src)
