CMAKE_MINIMUM_REQUIRED( VERSION 3.2 )
GET_FILENAME_COMPONENT( CURR_FILE_NAME ${CMAKE_CURRENT_LIST_FILE} NAME )
MESSAGE( STATUS "---------------------------")
MESSAGE( STATUS "CMAKE_CURRENT_LIST_DIR    : ${CMAKE_CURRENT_LIST_DIR}")
MESSAGE( STATUS "CURRENT FILE              : ${CURR_FILE_NAME}")
MESSAGE( STATUS "CURRENT SOURCE DIR        : ${CMAKE_CURRENT_SOURCE_DIR}")
MESSAGE( STATUS "CURRENT BINARY DIR        : ${CMAKE_CURRENT_BINARY_DIR}")
MESSAGE( STATUS "        SOURCE DIR        : ${CMAKE_SOURCE_DIR}")
MESSAGE( STATUS "        BINARY DIR        : ${CMAKE_BINARY_DIR}")

#===============================================================================
#===============================================================================
#===============================================================================

#
#  CMAKE ONLINE DOCUMENTATION
# 
#  https://cmake.org/documentation/
#

# name of executable or library
SET( PRJ_OUT  "mecl" )

# sets the variable 'PROJECT_NAME' indirectly
PROJECT( mecl
  VERSION     1.0.0.0
  LANGUAGES   CXX C )

# search for sources and header recursively
# and store the results as list in variables
FILE( GLOB_RECURSE sources_cxx
      *.cpp
	)
FILE( GLOB_RECURSE sources_c
      *.c
	)
FILE( GLOB_RECURSE headers
      *.h
	)
 
SET( MECL_SOURCES ${sources_cxx} PARENT_SCOPE )
SET( MECL_HEADERS ${headers}     PARENT_SCOPE )
SET( MECL_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. PARENT_SCOPE )


# additional include directories
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_BINARY_DIR} )

# additional directories with libraries
LINK_DIRECTORIES(
  )

# add special definitions
ADD_DEFINITIONS(
  )

# collect all sources to a executable
ADD_LIBRARY( ${PROJECT_NAME} STATIC
  ${sources_cxx}
  ${headers}
  )
  
# link with a list of libraries
# !! only the name of the library
#    the path is taken from 'LINK_DIRECTORIES'
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
  )

# set some target properties
SET_TARGET_PROPERTIES( ${PROJECT_NAME} 
  PROPERTIES
    OUTPUT_NAME ${PRJ_OUT}
    DEBUG_OUTPUT_NAME ${PRJ_OUT}_D
  )
