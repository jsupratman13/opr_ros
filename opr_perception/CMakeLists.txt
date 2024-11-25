cmake_minimum_required(VERSION 3.0.2)
project(opr_perception)

find_package(catkin REQUIRED)

add_custom_target(${PROJECT_NAME}_install_python_package ALL COMMAND python -m pip install pipenv pipenv-shebang)

catkin_package()
