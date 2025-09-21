cmake_minimum_required(VERSION 3.5)
project(drone_docs)

find_package(ament_cmake REQUIRED)
find_package(Doxygen REQUIRED)

# Doxyfile paths
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/../../../Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Configure the Doxyfile with CMake variables
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Custom target to build docs
add_custom_target(doc ALL
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating API documentation with Doxygen"
  VERBATIM
)

# Install the generated docs into the share/ folder
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
  DESTINATION share/${PROJECT_NAME}
)

ament_package()