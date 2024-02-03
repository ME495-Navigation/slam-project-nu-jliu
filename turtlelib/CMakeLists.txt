# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

# Eigen was not necessary, it should not be included here
# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()



# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(${PROJECT_NAME} src/geometry2d.cpp src/se2d.cpp src/svg.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(frame_main src/frame_main.cpp)

target_include_directories(frame_main
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(frame_main ${PROJECT_NAME})

# install the include files by copying the whole include directory
install(DIRECTORY include DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-config.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME})

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    
    # A test is just an executable that is linked against the unit testing library
    add_executable(test_${PROJECT_NAME} tests/test_geometry2d.cpp tests/test_se2d.cpp tests/test_svg.cpp)
    # add_executable(test_se2d tests/test_se2d2.cpp)

    # THis is not needed for executables. Include directories are provided by target_link libraries that
    # are used by the executable
    target_include_directories(test_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:include/>)

    # Do not leave commented out code
    # target_include_directories(test_se2d
    #     PUBLIC
    #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    #     $<INSTALL_INTERFACE:include/>)
    
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} Catch2::Catch2WithMain)
    # target_link_libraries(test_se2d ${PROJECT_NAME} Catch2::Catch2WithMain)

    # register the test with CTest, telling it what executable to run
    add_test(NAME ${PROJECT_NAME}_test COMMAND test_${PROJECT_NAME})
    # Do not leave commented out code
    # add_test(NAME se2d COMMAND test_se2d)
    # add_test(NAME ${PROJECT_NAME} COMMAND test_se2d)
endif()
