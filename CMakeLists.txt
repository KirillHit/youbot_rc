cmake_minimum_required(VERSION 3.9)

project(youbot_rc VERSION 1.0 DESCRIPTION "youbot_rc" LANGUAGES C)

include(GNUInstallDirs)

find_package(X11 REQUIRED)

set(CMAKE_C_STANDARD 99)

add_subdirectory(src)