
if(NOT "/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitinfo.txt" IS_NEWER_THAN "/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/thor/Desktop/HAL/build/_deps/moteus-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/thor/Desktop/HAL/build/_deps/moteus-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/mjbots/moteus.git" "moteus-src"
    WORKING_DIRECTORY "/home/thor/Desktop/HAL/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/mjbots/moteus.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout ee4f6ac9c6391523876836fcf6f276411b7f3670 --
  WORKING_DIRECTORY "/home/thor/Desktop/HAL/build/_deps/moteus-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'ee4f6ac9c6391523876836fcf6f276411b7f3670'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/thor/Desktop/HAL/build/_deps/moteus-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/thor/Desktop/HAL/build/_deps/moteus-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitinfo.txt"
    "/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/thor/Desktop/HAL/build/_deps/moteus-subbuild/moteus-populate-prefix/src/moteus-populate-stamp/moteus-populate-gitclone-lastrun.txt'")
endif()

