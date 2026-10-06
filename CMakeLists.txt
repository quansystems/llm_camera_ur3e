cmake_minimum_required(VERSION 3.8)
project(ur3e_llm_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gripper_control.py scripts/camera_check.py
  scripts/robot_skill.py scripts/task_validator.py scripts/action.py scripts/llm_control.py scripts/student.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/robot_action.py scripts/planner_state.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config description launch RViz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
