cmake_minimum_required(VERSION 3.8)
project(img_show)

# 如果是 GCC 或 Clang 编译器，添加警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)  # 添加 image_transport 包依赖
find_package(cv_bridge REQUIRED)  # 如果你使用 cv_bridge 转换图像格式


# 包含目录
# 不再需要 OpenCV 相关的包含目录

# 添加可执行文件
add_executable(camera_catch src/camera_catch.cpp)
add_executable(camera_receive src/camera_receive.cpp)
add_executable(get_camera_set src/get_camera_set.cpp)
add_executable(img_subscribers src/img_subscribers.cpp)


# 设置编译选项
target_compile_features(camera_catch PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(camera_receive PUBLIC c_std_99 cxx_std_17)
target_compile_features(get_camera_set PUBLIC c_std_99 cxx_std_17)
target_compile_features(img_subscribers PUBLIC c_std_99 cxx_std_17)

# 安装目标
install(TARGETS camera_catch DESTINATION lib/${PROJECT_NAME})
install(TARGETS camera_receive DESTINATION lib/${PROJECT_NAME})
install(TARGETS get_camera_set DESTINATION lib/${PROJECT_NAME})
install(TARGETS img_subscribers DESTINATION lib/${PROJECT_NAME})

# 不再链接 OpenCV 库
# target_link_libraries(camera_catch ${OpenCV_LIBS})
# target_link_libraries(camera_receive ${OpenCV_LIBS})
# target_link_libraries(get_camera_set ${OpenCV_LIBS})

# 添加 ROS 2 的依赖
ament_target_dependencies(camera_catch
  rclcpp
  sensor_msgs
  image_transport  # 使用 image_transport 进行图像传输
)

ament_target_dependencies(camera_receive
  rclcpp
  sensor_msgs
  image_transport  # 使用 image_transport 进行图像接收
)

ament_target_dependencies(get_camera_set
  rclcpp
  sensor_msgs
  image_transport
)

ament_target_dependencies(img_subscribers
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# 如果有单元测试的话
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 打包此项目
ament_package()
