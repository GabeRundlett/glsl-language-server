cmake_minimum_required(VERSION 3.21.0)
project(glsl-language-server VERSION 1.0.0)

include(cmake/warnings.cmake)

file(GLOB SOURCES src/*.cpp)
add_executable(glslls ${SOURCES})
target_compile_features(glslls PRIVATE cxx_std_20)

find_package(Threads REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(glslang CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(unofficial-mongoose CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

target_link_libraries(glslls
    Threads::Threads
    CLI11::CLI11
    fmt::fmt
    glslang::OSDependent glslang::glslang glslang::MachineIndependent glslang::GenericCodeGen glslang::glslang-default-resource-limits glslang::OGLCompiler glslang::SPVRemapper glslang::SPIRV glslang::HLSL
    unofficial::mongoose::mongoose
    nlohmann_json::nlohmann_json
)

install(TARGETS glslls RUNTIME DESTINATION bin)
