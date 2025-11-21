# --- 1. Preamble ---
cmake_minimum_required(VERSION 3.8)
project(hri_trees) # UPDATED Project Name

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 2. Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)

# --- 3. Build the Plugin Library ---
# This part is correct. We create a library target named 'hri_action1_plugin'.
add_library(hri_action1_plugin SHARED
  plugins/actions/action1.cpp
)
target_include_directories(hri_action1_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# We use ament_target_dependencies here because we are linking against OTHER ROS packages.
ament_target_dependencies(hri_action1_plugin PUBLIC
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_behavior_tree
)

# --- 4. Build the Executable ---
add_executable(bt_executor
  src/bt_executor.cpp
)
# Link the executable against OTHER ROS packages.
ament_target_dependencies(bt_executor
  rclcpp_lifecycle
  behaviortree_cpp_v3
)

# --- THE FIX IS HERE ---
# Use target_link_libraries to link against another TARGET in the SAME project.
target_link_libraries(bt_executor hri_action1_plugin)


# --- 5. Register the Plugin with pluginlib ---
# I've updated the name of your plugins.xml file to match your structure.
pluginlib_export_plugin_description_file(behaviortree_cpp_v3 plugins.xml)


# --- 6. Installation Rules ---
# Install the targets (library and executable). I've updated the plugin name.
install(TARGETS
  hri_action1_plugin
  bt_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Install the header files
install(DIRECTORY
  include/
  DESTINATION include
)
# Install the launch and behavior tree files. I've updated the 'trees' directory name.
install(DIRECTORY
  launch
  trees
  DESTINATION share/${PROJECT_NAME}
)
# Install the plugin registration file. I've updated the name to 'plugins.xml'.
install(FILES
  "plugins.xml"
  DESTINATION "share/${PROJECT_NAME}"
)

# --- 7. Final Ament Step ---
ament_package()