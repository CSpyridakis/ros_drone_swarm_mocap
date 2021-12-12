cmake_minimum_required(VERSION 2.8)
project( hough_test )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# =================== Image ===================
set(FilesToInclude 
main.cpp
)
add_executable(${PROJECT_NAME} ${FilesToInclude})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
