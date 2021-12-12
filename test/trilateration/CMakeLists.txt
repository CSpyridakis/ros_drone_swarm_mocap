cmake_minimum_required(VERSION 2.8)
project( example )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# =================== Image ===================
set(FilesToInclude 
src/example.cpp
)
add_executable(${PROJECT_NAME} ${FilesToInclude})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

# =================== Video ===================
set(FilesToInclude_video 
src/example-video.cpp
)
add_executable(${PROJECT_NAME}_video ${FilesToInclude_video})
target_link_libraries( ${PROJECT_NAME}_video ${OpenCV_LIBS} )
