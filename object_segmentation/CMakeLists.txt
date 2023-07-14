cmake_minimum_required(VERSION 2.8.3)
project(object_segmentation)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)

if(APPLE)
  catkin_simple()
  set(LLVM_PATH "/usr/local/opt/llvm")
  find_package(LLVM HINTS ${LLVM_PATH}/lib/cmake/llvm)
  if (LLVM_FOUND)
    message(STATUS "Building with LLVM clang and omp support.")
    set(CMAKE_C_COMPILER ${LLVM_TOOLS_BINARY_DIR}/clang)
    set(CMAKE_CXX_COMPILER ${LLVM_TOOLS_BINARY_DIR}/clang++)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -L${LLVM_PATH}/lib -Wl,-rpath,${LLVM_PATH}/lib")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -L${LLVM_PATH}/lib -Wl,-rpath,${LLVM_PATH}/lib")
  else()
    message(STATUS "LLVM not found, building without omp support.")
  endif()
else()
  catkin_simple(ALL_DEPS_REQUIRED)
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
    set(OpenMP_FLAGS -fopenmp)
    set(OpenMP_LIBS omp)
    add_definitions(-Wno-unknown-pragmas -fopenmp)
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
    set(OpenMP_FLAGS -fopenmp)
    set(OpenMP_LIBS gomp)
    add_definitions(-Wno-unknown-pragmas -fopenmp)
  else()
    message("Unrecognized C++ compiler: ${CMAKE_CXX_COMPILER} ID: ${CMAKE_CXX_COMPILER_ID}. Compiling without openMP support.")
  endif()
endif()

cs_add_library(${PROJECT_NAME}
  src/ObjectDetector.cc
)
target_link_libraries(${PROJECT_NAME} ${OpenMP_LIBS})
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})

# add_definitions(-DWRITE_IMAGES)
cs_add_executable(${PROJECT_NAME}_node
  src/object_seg_node.cc
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_install()
cs_export()



# cmake_minimum_required(VERSION 3.0.2)
# project(object_segmentation)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   cv_bridge
#   pcl_ros
#   pcl_conversions
# )

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

# ## Uncomment this if the package has a setup.py. This macro ensures
# ## modules and global scripts declared therein get installed
# ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# # catkin_python_setup()

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# #  INCLUDE_DIRS include
# #  CATKIN_DEPENDS roscpp rospy std_msgs
# #  DEPENDS system_lib

# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )

# ## Declare a C++ library
# # add_library(${PROJECT_NAME}
# #   src/${PROJECT_NAME}/agv_control.cpp
# # )

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide
# # add_executable(${PROJECT_NAME}_node src/agv_control_node.cpp)
# add_executable(${PROJECT_NAME}_node src/object_seg_node.cpp)

# ## Rename C++ executable without prefix
# ## The above recommended prefix causes long target names, the following renames the
# ## target back to the shorter version for ease of user use
# ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# ## Add cmake target dependencies of the executable
# ## same as for the library above
# # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# # target_link_libraries(${PROJECT_NAME}_node
# #   ${catkin_LIBRARIES}
# # )
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )