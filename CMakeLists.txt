cmake_minimum_required(VERSION 3.0)
set(EXEC_NAME lxm32)
project(${EXEC_NAME})

set(CANOPEN_SUBGIT_DIR "lib/canopen")

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  # Update submodules as needed
  option(GIT_SUBMODULE "Check submodules during build" ON)
  if(GIT_SUBMODULE)
    message(STATUS "Check submodules during build")
    message(STATUS "Submodule update")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
    message(STATUS "Submodule update ${PROJECT_SOURCE_DIR}/${CANOPEN_SUBGIT_DIR}/CMakeLists.txt")
  endif()
endif()
  
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${CANOPEN_SUBGIT_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()




  

#get the sources and headers
file(GLOB SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "src/*.c")
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" "include/*.hpp")


#add the include libraries directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${CANOPEN_SUBGIT_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(${CANOPEN_SUBGIT_DIR})

#create our exec file
add_executable(${EXEC_NAME} ${SRCS} ${HEADERS})

#link LSL libraries
if(UNIX)
  target_link_libraries (${EXEC_NAME} -lutil -lboost_iostreams -lboost_system -lboost_filesystem -lpthread )
  ADD_CUSTOM_COMMAND (	TARGET ${EXEC_NAME} POST_BUILD
						#copy the dll files 
						COMMENT "Saving exec in bin/ directory..." VERBATIM
						COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/bin/UNIX/${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_VERSION}
						COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/${EXEC_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/bin/UNIX/${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_VERSION}
						)
endif(UNIX)

