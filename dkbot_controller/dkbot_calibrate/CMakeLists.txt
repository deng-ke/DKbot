cmake_minimum_required(VERSION 2.8.3)

project(dkbot_calibrate)

find_package(catkin REQUIRED dynamic_reconfigure)

generate_dynamic_reconfigure_options(
   cfg/CalibrateAngular.cfg
   cfg/CalibrateLinear.cfg
)
