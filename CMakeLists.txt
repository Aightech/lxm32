cmake_minimum_required(VERSION 3.0)
###### Author: Alexis Devillard
###### Date:   27/01/2020
###### Brief: This CmakeLists.txt file enable to build generic project with the following structure:
######                                +---------+
######                                | project |
######                                +----+----+
######                                     |
######            +----------+------------++-------------+-----------+
######            |          |            |              |           |
######         +--+--+  +----+----+ +-----+------+ +-----+------+ +--+--+
######         | src |  | include | | CMakelists | | CMakefunct | | lib |
######         +--+--+  +----+----+ |    .txt    | |  ions.txt  | +--+--+
######            |          |      +------------+ +------------+    |
######      +-----+------+   |                                       |
######      |            |   +-------+            +--------------+---+
###### +----+-----+      |           |            |              |
###### | main.cpp | +----+----+ +----+----+ +-----+------+ +-----+------+
###### +----------+ | lib.cpp | | lib.hpp | | subproject | | subproject |
######              +---------+ +---------+ |     1      | |     2      |
######                                      +-----+------+ +------------+
######                                            |
######                        +-----------+-------+------+-----------+
######                        |           |              |           |
######                     +--+---+ +-----+------+ +-----+------+ +--+--+
######                     | .git | | CMakelists | | CMakeFunct | | ... |
######                     +------+ |    .txt    | |  ions.txt  | +-----+
######                              +------------+ +------------+
###### 
######

###### PROJECT NAME AND VERSION ######
set(EXEC_NAME lxm32)
set(PROJECT_VERSION 1.0)
project(${EXEC_NAME} VERSION ${PROJECT_VERSION})

###### LIBRARY NAME ######
set(LIB_NAME ${EXEC_NAME}.${PROJECT_VERSION})

###### C++ standard ######
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

###### Special libraries  ######
set(EXTRA_LIBS "")
if(NO)
  set(EXTRA_LIBS "")
endif()



###### Get the sources and headers ######
###### Exec main src file ######
set(MAIN_FILE "src/main.cpp")
###### Exec-only header file ######
set(EXTRA_HEADERS "include/gnuplot-iostream.h")

###### Library src and header files (everything else) ######
file(GLOB LIB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "src/*.c")
list(REMOVE_ITEM LIB_SRCS ${MAIN_FILE})
file(GLOB LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" "include/*.hpp")
list(REMOVE_ITEM LIB_HEADERS ${EXTRA_HEADERS})





###### include helper functions ######
include(CMakeFunctions.txt)


###### Search for Subdirectories/Libraries ######
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib") 
  subdirlist(LIBS "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
  foreach(subdir ${LIBS})
    #git submodules init
    execute_process(COMMAND git submodule update --remote --merge --init -- "lib/${subdir}"
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    add_subdirectory("lib/${subdir}")
    #get the version
    subproject_version("${subdir}" subdir_version)
    message("-- ${EXEC_NAME}: Added library ${subdir} version: ${subdir_version}")
    list(APPEND INTERN_LIBS "${subdir}.${subdir_version}")
  endforeach()
endif()




###### Create the library ######
add_library(${LIB_NAME} ${LIB_SRCS})
target_link_libraries (${LIB_NAME}
  PUBLIC ${INTERN_LIBS}
  PUBLIC ${EXTRA_LIBS})
target_include_directories(${LIB_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")



###### Create the executable ######
message("${MAIN_FILE} ${EXEC_NAME} ${LIB_NAME} ${EXTRA_LIBS}")
add_executable(${EXEC_NAME} ${MAIN_FILE})
target_link_libraries (${EXEC_NAME}
  PUBLIC ${LIB_NAME}
  PUBLIC ${EXTRA_LIBS})
target_include_directories(${EXEC_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")



 
###### Instalation rule ######
install(TARGETS ${EXEC_NAME} DESTINATION bin)
install(TARGETS ${LIB_NAME} DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include)


# enable_testing()

# # define a function to simplify adding tests
# function(do_test target arg result)
#   add_test(NAME Comp${arg} COMMAND ${target} ${arg})
#   set_tests_properties(Comp${arg}
#     PROPERTIES PASS_REGULAR_EXPRESSION ${result}
#     )
# endfunction(do_test)

# # do a bunch of result based tests
# do_test(canopen 1 1000 0 "")
# do_test(canopen 2 607A 0 4 d 1000 "")
