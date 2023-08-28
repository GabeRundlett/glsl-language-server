
if (EXISTS "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
else()
    if (NOT EXISTS "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
        find_package(Git REQUIRED)
        execute_process(COMMAND ${GIT_EXECUTABLE} clone https://github.com/Microsoft/vcpkg
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMAND_ERROR_IS_FATAL ANY)
    endif()
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()
