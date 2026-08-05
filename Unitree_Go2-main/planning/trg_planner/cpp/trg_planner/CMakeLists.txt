cmake_minimum_required(VERSION 3.16...3.24)
project(trg_planner VERSION 1.0.0)

include(cmake/CompilerOptions.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check build types
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

# set package directory path
set(TRG_DIR ${CMAKE_CURRENT_LIST_DIR})
message(STATUS "TRG_DIR: ${TRG_DIR}")
add_definitions(-DTRG_DIR="${TRG_DIR}")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Planner class library
set(TARGET_NAME trg_planner_core)
add_library(${TARGET_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/core/trg_planner/src/kdtree/kdtree.c
    ${CMAKE_CURRENT_LIST_DIR}/core/trg_planner/src/planner/trg_planner.cpp
    ${CMAKE_CURRENT_LIST_DIR}/core/trg_planner/src/graph/trg.cpp
)

target_link_libraries(${TARGET_NAME}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
)

target_include_directories(${TARGET_NAME}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/core/
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    $<INSTALL_INTERFACE:include>
)

set(CMAKE_INSTALL_LIBDIR lib)
install (TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}-export
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/core/trg_planner DESTINATION include)

export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE ${PROJECT_NAME}-exports.cmake)

install(FILES cmake/${PROJECT_NAME}Config.cmake
DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
