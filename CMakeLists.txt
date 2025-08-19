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


# ライブラリの作成
add_library(${PROJECT_NAME}
  src/dual_leg_controller.cpp
  src/dynamixel_sync_controller.cpp
)

# ライブラリの依存関係
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ライブラリのリンク
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


# リンクの設定
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
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

# インストール設定に以下を追加
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安全テスト用の実行ファイルを追加 (ここから)
# 実行ファイルの作成
add_executable(safe_single_leg_test_node 
  src/safe_single_leg_test.cpp
)

# 依存関係の設定
add_dependencies(safe_single_leg_test_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# ライブラリのリンク
target_link_libraries(safe_single_leg_test_node
  ${catkin_LIBRARIES}
)

# インストール設定
install(TARGETS safe_single_leg_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Pythonスクリプトのインストール
install(PROGRAMS
  scripts/leg_safety_monitor.py
  scripts/check_initialization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 設定ファイルのインストール
install(FILES
  config/safe_leg_test_params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  launch/safe_single_leg_test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
# 安全テスト用の実行ファイルを追加(ここまで)

# 2脚協調テスト用実行ファイル(ここから)
add_executable(safe_dual_leg_test_node 
  src/safe_dual_leg_test.cpp
)

add_dependencies(safe_dual_leg_test_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(safe_dual_leg_test_node
  ${catkin_LIBRARIES}
)

install(TARGETS safe_dual_leg_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 2脚協調テスト用実行ファイル(ここまで)

# 2脚相対位置歩行テスト用実行ファイル(ここから)
add_executable(two_leg_relative_walk_node 
  src/two_leg_relative_walk.cpp
)

# 依存関係の設定
add_dependencies(two_leg_relative_walk_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# ライブラリのリンク
target_link_libraries(two_leg_relative_walk_node
  ${catkin_LIBRARIES}
)

# インストール設定
install(TARGETS two_leg_relative_walk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launchファイルのインストール
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
# 2脚相対位置歩行テスト用実行ファイル(ここまで)

# 6脚トライポッド歩行システム
add_executable(hexapod_tripod_controller_node 
  src/hexapod_tripod_controller.cpp
)

add_dependencies(hexapod_tripod_controller_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hexapod_tripod_controller_node
  ${catkin_LIBRARIES}
)

# インストール設定
install(TARGETS two_leg_relative_walk_node hexapod_tripod_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 6脚トライポッド歩行システム(ここまで)
