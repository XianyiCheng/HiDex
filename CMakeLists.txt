# Author: petter.strandmark@gmail.com (Petter Strandmark)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)
PROJECT(MCTS C CXX)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Default locations to search for on various platforms.
LIST(APPEND SEARCH_LIBS /usr/lib)
LIST(APPEND SEARCH_LIBS /usr/local/lib)
LIST(APPEND SEARCH_LIBS /usr/local/homebrew/lib) # Mac OS X
LIST(APPEND SEARCH_LIBS /opt/local/lib)

LIST(APPEND SEARCH_HEADERS /usr/include)
LIST(APPEND SEARCH_HEADERS /usr/local/include)
LIST(APPEND SEARCH_HEADERS /usr/local/homebrew/include) # Mac OS X
LIST(APPEND SEARCH_HEADERS /opt/local/include)

ENABLE_TESTING()

# Change the default build type from Debug to Release, while still
# supporting overriding the build type.
#
# The CACHE STRING logic here and elsewhere is needed to force CMake
# to pay attention to the value of these variables.
IF(NOT CMAKE_BUILD_TYPE)
  MESSAGE("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(MY_LIBRARY_DEPENDENCIES)

# If you installed dart-6 and don't want to build dartsim inside this package, uncomment this
FIND_PACKAGE(DART REQUIRED COMPONENTS gui collision-bullet utils utils-urdf)
set(dart_libs dart dart-utils dart-utils-urdf dart-gui dart-collision-bullet)

# Multithreading using OpenMP
OPTION(OPENMP
  "Enable multi-threading (requires OpenMP)"
  ON)

IF(${OPENMP})
  FIND_PACKAGE(OpenMP)

  IF(${OPENMP_FOUND})
    MESSAGE("-- Found OpenMP.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    ADD_DEFINITIONS(-DUSE_OPENMP)

    IF(NOT MSVC)
      # OpenMP needs to be included as a library on some platforms.
      LIST(APPEND MY_LIBRARY_DEPENDENCIES gomp)
    ENDIF(NOT MSVC)
  ELSE(${OPENMP_FOUND})
    MESSAGE("-- Can't find OpenMP. Continuing without it.")
  ENDIF(${OPENMP_FOUND})
ENDIF(${OPENMP})

#
# gcc settings.
#
IF(CMAKE_COMPILER_IS_GNUCXX)
  # No warnings for C. The Meschach library contains really old code.
  # -fPIC for building a shared library.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter -fpic")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#
# Clang settings
#
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter -fpic")
ENDIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

#
# MSVC settings
#
IF(MSVC)
  # Disable deprecation warning for standard functions.
  ADD_DEFINITIONS("/wd4996")

  # To get rid of annoying min and max macros if windows.h
  # is included.
  ADD_DEFINITIONS("-DNOMINMAX=1")
  ADD_DEFINITIONS("-D_VARIADIC_MAX=6")
ENDIF(MSVC)

#
# Include directories
#
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(third-party/Catch)

FILE(GLOB MCTS_HEADERS ${CMAKE_SOURCE_DIR}/*.h)

# find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)
# if (BULLET_FOUND)
#     message("find bullet")
# else()
#     message("cannot find bullet")
# endif()


add_definitions(-DSRC_DIR="${PROJECT_SOURCE_DIR}")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(general_planner)
