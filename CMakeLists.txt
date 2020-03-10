cmake_minimum_required(VERSION 2.8.3)
project(yapykaldi_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  hmi
  grammar_parser
)

catkin_python_setup()

catkin_package()
