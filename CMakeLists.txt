cmake_minimum_required(VERSION 3.21.0)
project(glslls VERSION 1.1.0)

include(cmake/warnings.cmake)

# enable_static_analysis()

file(GLOB SOURCES src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
set_project_warnings(${PROJECT_NAME})

find_package(Threads REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(glslang CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(unofficial-mongoose CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(lspcpp CONFIG REQUIRED)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    GLSLLS_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    GLSLLS_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    GLSLLS_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
#     if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
#         target_link_options(${PROJECT_NAME} PRIVATE "-Wl,/ENTRY:mainCRTStartup,/SUBSYSTEM:WINDOWS")
#     else()
#         target_link_options(${PROJECT_NAME} PRIVATE /ENTRY:mainCRTStartup /SUBSYSTEM:WINDOWS)
#     endif()
# endif()

target_link_libraries(${PROJECT_NAME}
    Threads::Threads
    CLI11::CLI11
    fmt::fmt-header-only
    glslang::OSDependent glslang::glslang glslang::MachineIndependent glslang::GenericCodeGen glslang::glslang-default-resource-limits glslang::OGLCompiler glslang::SPVRemapper glslang::SPIRV glslang::HLSL
    unofficial::mongoose::mongoose
    nlohmann_json::nlohmann_json
    lspcpp::lspcpp
    # lspcpp::network-uri
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
