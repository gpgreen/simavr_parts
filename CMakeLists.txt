cmake_minimum_required(VERSION 3.1)

project(simavr_parts)

add_library(simavr_parts
  mcp2515_virt.c
  mcp2515_virt.h
  mcp2515_def.h
)

##########################################################################
# find simavr library
##########################################################################
find_path(SIMAVR_INCLUDE_DIR simavr/sim_avr.h
  HINTS
  /usr/include/simavr
  /usr/local/include/simavr
  )
mark_as_advanced(SIMAVR_INCLUDE_DIR)
find_library(SIMAVRLIB simavr
  HINTS
  /usr/lib
  /usr/local/lib
  REQUIRED
)
mark_as_advanced(SIMAVRLIB)
include_directories(SIMAVR_INCLUDE_DIR)

##########################################################################
# set build type
##########################################################################
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

##########################################################################
# compiler options for all build types
##########################################################################
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-Wall")
#add_definitions("-Werror")
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions("-c")
add_definitions("-std=gnu99")



