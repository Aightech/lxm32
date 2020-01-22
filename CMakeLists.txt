cmake_minimum_required(VERSION 3.0)

###### PROJECT NAME AND VERSION ######
set(EXEC_NAME lxm32)
set(PROJECT_VERSION 1.0)

###### LIBRARY NAME ######
set(LIB_NAME ${EXEC_NAME}.${PROJECT_VERSION})

###### C++ standard ######
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)




#Set prooject name
project(${EXEC_NAME} VERSION ${PROJECT_VERSION})
#get the sources and headers
set(MAIN_FILE "src/main.cpp")
file(GLOB SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "src/*.c")
list(REMOVE_ITEM SRCS ${MAIN_FILE})
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" "include/*.hpp")



#### add subdirectory libraries ####
function(subproject_version subproject_name VERSION_VAR)
  # Read CMakeLists.txt for subproject and extract project() call(s) from it.
  file(STRINGS "${${subproject_name}_SOURCE_DIR}/CMakeLists.txt" project_calls REGEX "[ \t]*set\\(")
  # For every project() call try to extract its VERSION option
  foreach(project_call ${project_calls})
    string(REGEX MATCH "VERSION[ ]+([^ )]+)" version_param "${project_call}")
    if(version_param)
      set(version_value "${CMAKE_MATCH_1}")
    endif()
  endforeach()
  if(version_value)
    set(${VERSION_VAR} "${version_value}" PARENT_SCOPE)
  else()
    message("WARNING: Cannot extract version for subproject '${subproject_name}'")
  endif()
endfunction(subproject_version)

MACRO(subdirlist result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
subdirlist(LIBS "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
foreach(subdir ${LIBS})
  execute_process(COMMAND git submodule update --init -- "lib/${subdir}"
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  
  #add subdirectory libraries
  add_subdirectory("lib/${subdir}")
  
  subproject_version("${subdir}" subdir_version)
  message("-- Added lib/${subdir} version: ${subdir_version}")
  list(APPEND EXTRA_LIBS "${subdir}.${subdir_version}")
endforeach()



#create executable
add_executable(${EXEC_NAME} ${SRCS})
target_link_libraries (${EXEC_NAME}
  PUBLIC "${EXTRA_LIBS}")
target_include_directories(${EXEC_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")


#create library
add_library(${LIB_NAME} ${SRCS})
target_include_directories( ${LIB_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
target_link_libraries (${LIB_NAME}
  PUBLIC "${EXTRA_LIBS}")

 

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
