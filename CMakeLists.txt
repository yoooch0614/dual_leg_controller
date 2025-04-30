cmake_minimum_required(VERSION 3.0.2)
project(dual_leg_controller)

# C++11を使用
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamixel_sdk
)

# メッセージファイルの追加
add_message_files(
  FILES
  LegCommand.msg
  LegPosition.msg
)

# メッセージの生成
generate_messages(
  DEPENDENCIES
  std_msgs
)

# パッケージの設定
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime dynamixel_sdk
)

# インクルードディレクトリの設定
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 実行ファイルの生成
add_executable(${PROJECT_NAME}_node 
  src/dual_leg_controller.cpp
  src/dynamixel_sync_controller.cpp
)

# 依存関係の設定
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# リンクの設定
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# インストール設定
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# manual_tripod_stanceノードのビルド
add_executable(manual_tripod_stance src/manual_tripod_stance.cpp)
target_link_libraries(manual_tripod_stance ${catkin_LIBRARIES})
add_dependencies(manual_tripod_stance ${catkin_EXPORTED_TARGETS})

# インストール設定
install(TARGETS manual_tripod_stance
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
# initial_position_setterノードのビルド
add_executable(initial_position_setter src/initial_position_setter.cpp)
target_link_libraries(initial_position_setter ${catkin_LIBRARIES})
add_dependencies(initial_position_setter ${catkin_EXPORTED_TARGETS})

# インストール設定
install(TARGETS initial_position_setter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)