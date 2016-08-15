# test CMakeLists.txt
  
# 设置编译器
 set(CMAKE_CXX_COMPILER g++)

#添加编译参数，此处设置编译器的版本
ADD_DEFINITIONS(-std=c++11)

# 设置生成的可执行文件的路径
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR/src})
# 源文件列表
        set(SRC_LIST
               main.cpp
               PointCloud.cpp           PointCloud.h
               DataInput.cpp            DataInput.h
               coco.cpp                 coco.h
               border.cpp               border.h
               border_facet_selection.cpp       border_facet_selection.h
               ring_extending_coco.cpp          ring_extending_coco.h
               test_incrementalcoco.cpp         test_incrementalcoco.h
               IncLocal.cpp                     IncLocal.h
               IncTestFunc.cpp                  IncTestFunc.h
                                    
               )
#查找当前目录下的所有源文件，并将名称保存到 SRC_LIST变量
#AUX_SOURCE_DIRECTORY(. SRC_LIST)


find_package(CGAL)

if ( CGAL_FOUND )

  include(${CGAL_USE_FILE})
 
else()
  message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

#该指令添加位置不同，执行结果不同。
ADD_EXECUTABLE(test_coco ${SRC_LIST})

#添加多个特定的头文件搜索路径，路径之间用空格分割
# 所需要的glib库文件的目录
INCLUDE_DIRECTORIES(
	/usr/include/glib-2.0
	/usr/lib/glib-2.0/include
	/usr/lib/x86_64-linux-gnu/glib-2.0/include
	/usr/include/gsl
	)

#为target添加共享库
TARGET_LINK_LIBRARIES(test_coco
        glib-2.0
	gsl
        gslcblas
	m
)
