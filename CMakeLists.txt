cmake_minimum_required(VERSION 3.0.0)
project(mySLAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED)

find_package(Ceres REQUIRED)

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
# find_package(CSparse REQUIRED)
# include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Sophus::Sophus
    ${Pangolin_LIBRARIES} GL GLU GLEW glut
    g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
    Ceres::ceres
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SBA} ${G2O_CSPARSE_EXTENSION}
)

if(NOT TARGET glog::glog)
    find_package(Glog REQUIRED)
    list(APPEND THIRD_PARTY_LIBS ${GLOG_LIBRARIES})
endif()


enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
# add_subdirectory(test)
add_subdirectory(bin)



# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${EIGEN_INCLUDE_DIR})
# include_directories(${Sophus_INCLUDE_DIRS})
# include_directories(${CERES_INCLUDE_DIRS})

# include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${OCTOMAP_INCLUDE_DIRS})

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# set(source_files
#     src/main.cpp
# )

# add_executable(execName ${source_files})


# Suppress Warnings
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     # Suppress specific warnings for GCC and Clang
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-parameter")
# elseif(MSVC)
#     # Suppress specific warnings for MSVC
#     add_compile_options(/wd4244 /wd4267)
# endif()