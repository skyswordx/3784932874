cmake_minimum_required(VERSION 3.8)
project(smartcar2024)

# options
option(ENABLE_AI_MODULE "Enable AI module" ON)
option(ENABLE_SERIAL_MODULE "Enable serial module" ON)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
# set OpenCV directory(required if using Windows)
set(OpenCV_DIR "./libs/opencv/build")

# 如果使用的是 MSVC 编译器，设置源代码字符集为 UTF-8
if(MSVC)
    add_compile_options("/source-charset:utf-8")
endif()

# find OpenCV
find_package(OpenCV REQUIRED)

if(NOT WIN32)
    # find Threads(Pthreads if using Linux)
    find_package(Threads REQUIRED)

    # find PkgConfig
    if(ENABLE_AI_MODULE OR ENABLE_SERIAL_MODULE)
        find_package(PkgConfig)
        pkg_search_module(GLIB REQUIRED glib-2.0)
        include_directories(${GLIB_INCLUDE_DIRS})
    endif()

    if(ENABLE_SERIAL_MODULE)
        add_definitions(-DENABLE_SERIAL_MODULE)
        # find libserial
        pkg_search_module(SERIAL REQUIRED libserial)
        include_directories(${SERIAL_INCLUDE_DIRS})
        link_directories(${SERIAL_LIBRARY_DIRS})
    endif()

    if(ENABLE_AI_MODULE)
        add_definitions(-DENABLE_AI_MODULE)
        # find ppnc
        pkg_search_module(PPNC REQUIRED ppnc)
        include_directories(${PPNC_INCLUDE_DIRS})
        link_directories(${PPNC_LIBRARY_DIRS})

        # find onnx
        pkg_search_module(ONNX REQUIRED onnx)
        include_directories(${ONNX_INCLUDE_DIRS})
        link_directories(${ONNX_LIBRARY_DIRS})
    endif()

endif()

# 设置可执行文件目录
file(GLOB_RECURSE SMARTCAR_SOURCE_FILES "src/smartcar/*.cpp")
add_executable(smartcar src/main.cpp ${SMARTCAR_SOURCE_FILES})

# 配置头文件目录
target_include_directories(smartcar PRIVATE include)
target_include_directories(smartcar PRIVATE libs/json/include)

# 配置要链接的库
target_link_libraries(smartcar ${OpenCV_LIBS})
if(NOT WIN32)
    target_link_libraries(smartcar Threads::Threads)

    if(ENABLE_SERIAL_MODULE)
        target_link_libraries(smartcar ${SERIAL_LIBRARIES})
    endif()

    if(ENABLE_AI_MODULE)
        target_link_libraries(smartcar ${PPNC_LIBRARIES})
        target_link_libraries(smartcar ${ONNX_LIBRARIES})
    endif()
endif()

if(WIN32)
    # 获取所有 DLL 文件
    file(GLOB DLL_FILES "${OpenCV_DIR}/x64/vc16/bin/*.dll")
    # 对每个 DLL 文件，如果它与目标文件夹中的文件不同，则复制它
    foreach(DLL_FILE ${DLL_FILES})
        add_custom_command(
            TARGET smartcar 
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${DLL_FILE}
            $<TARGET_FILE_DIR:smartcar>
        )
    endforeach()
endif()

add_executable(screenshot src/screenshot.cpp)
target_link_libraries(screenshot ${OpenCV_LIBS})