cmake_minimum_required(VERSION 3.8)
project(nuturtle_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
    DIRECTORY launch config meshes urdf
    DESTINATION share/${PROJECT_NAME}
)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

if(NOT CMAKE_CROSSCOMPILING)
    find_package(Doxygen REQUIRED)
    # build just because Doxygen is missing
    if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})

        # Turn the README.md into the homepage of the doxygen docs
        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

        # Tell Doxygen where to find the documentation
        doxygen_add_docs(doxygen include/ src/ README.md ALL)

        # The documentation will be in the build/html directory
        # The main page is build/html/index.html
    endif()

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()
endif()

ament_package()
