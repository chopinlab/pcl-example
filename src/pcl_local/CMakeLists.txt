
cmake_minimum_required(VERSION 3.10)
project(local_project)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL 찾기
find_package(PCL 1.12 REQUIRED)
find_package(nlohmann_json REQUIRED)
# PCL 링크 설정
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
add_executable(${PROJECT_NAME} local.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)