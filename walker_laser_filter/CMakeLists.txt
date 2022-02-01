cmake_minimum_required(VERSION 3.5)
project(walker_laser_filter)

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##############################################################################
# Build
##############################################################################

ament_auto_add_library(walker_laser_scan_filter SHARED src/walker_laser_scan_filter.cpp)

##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters walker_laser_filter.xml)
