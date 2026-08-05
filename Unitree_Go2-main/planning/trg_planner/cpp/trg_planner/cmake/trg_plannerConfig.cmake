get_filename_component(MYPROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)
find_dependency(OpenCV REQUIRED)
find_dependency(PCL REQUIRED)

# The parameter must adhere to the format: ${PROJECT_NAME}Targets.cmake
include("${MYPROJECT_CMAKE_DIR}/trg_plannerTargets.cmake")
