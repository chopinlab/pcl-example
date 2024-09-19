cmake_minimum_required(VERSION 3.10)
project(pcl_project)

# # C++ 표준 설정
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # PCL 찾기
# find_package(PCL 1.9 REQUIRED)

# # PCL 링크 설정
# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
# include_directories(${PCL_INCLUDE_DIRS})
# add_definitions(${PCL_DEFINITIONS})



add_subdirectory(src/pcl_test)
add_subdirectory(src/pcl_filter)
add_subdirectory(src/pcl_merge)