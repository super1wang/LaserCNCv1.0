cmake_minimum_required(VERSION 3.20)

project(LcncModelEnvelopeTools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(LCNC_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(LCNC_QT6_ROOT "E:/Qt6.9.1/6.9.1/msvc2022_64"
    CACHE PATH "Qt used by the isolated model-envelope tools")
set(LCNC_OCCT_ROOT
    "J:/Develop/OCC8.0.1/occt-combined-with-debug-pch/opencascade-8.0.1-vc14-64"
    CACHE PATH "OpenCASCADE used only for STEP tessellation")
set(LCNC_OCCT_3RDPARTY_DIR
    "J:/Develop/OCC8.0.1/occt-combined-with-debug-pch/3rdparty-vc14-64"
    CACHE PATH "OpenCASCADE third-party runtime root")
set(LCNC_ENVELOPE_DEPS_ROOT
    "G:/environment/vcpkg-master/vcpkg/installed/x64-windows"
    CACHE PATH "vcpkg prefix containing OpenVDB and CGAL")

set(Qt6_DIR "${LCNC_QT6_ROOT}/lib/cmake/Qt6" CACHE PATH "Qt6 package directory")
find_package(Qt6 6.9.1 EXACT REQUIRED COMPONENTS Core)
set(OpenCASCADE_DIR "${LCNC_OCCT_ROOT}/cmake"
    CACHE PATH "OpenCASCADE package directory")
find_package(OpenCASCADE 8.0.1 EXACT REQUIRED)

# This is a separate CMake project so its vcpkg Boost/Eigen packages cannot
# affect LaserCNC or Coal dependency resolution in the main build tree.
list(PREPEND CMAKE_PREFIX_PATH "${LCNC_ENVELOPE_DEPS_ROOT}")
list(PREPEND CMAKE_MODULE_PATH "${LCNC_ENVELOPE_DEPS_ROOT}/share/openvdb")
find_package(OpenVDB MODULE REQUIRED)
find_package(CGAL CONFIG REQUIRED
    PATHS "${LCNC_ENVELOPE_DEPS_ROOT}/share/cgal"
    NO_DEFAULT_PATH)

set(LCNC_ENVELOPE_OCC_LIBS
    TKernel TKMath TKLCAF TKBRep TKTopAlgo TKBool TKMesh TKXCAF TKDESTEP)

add_library(lcnc_model_envelope_common STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/model_envelope_common.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/model_envelope_common.cpp")
target_include_directories(lcnc_model_envelope_common PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${OpenCASCADE_INCLUDE_DIR}")
target_link_directories(lcnc_model_envelope_common PRIVATE
    "$<IF:$<CONFIG:Debug>,${LCNC_OCCT_ROOT}/win64/vc14/libd,${LCNC_OCCT_ROOT}/win64/vc14/lib>")
target_link_libraries(lcnc_model_envelope_common PRIVATE
    Qt6::Core ${LCNC_ENVELOPE_OCC_LIBS})
if(WIN32)
    target_link_libraries(lcnc_model_envelope_common PRIVATE Psapi)
    target_compile_definitions(lcnc_model_envelope_common PUBLIC
        NOMINMAX _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
endif()
if(MSVC)
    target_compile_options(lcnc_model_envelope_common PUBLIC
        /FS /FIwindows.h /Zc:__cplusplus /permissive- /utf-8)
endif()

set(LCNC_ENVELOPE_OUTPUT_ROOT
    "${LCNC_REPOSITORY_ROOT}/x64/envelope-tools")
set(LCNC_RUNTIME_SNAPSHOT
    "$<IF:$<CONFIG:Debug>,${LCNC_REPOSITORY_ROOT}/3rd/runtime/bin_debug,${LCNC_REPOSITORY_ROOT}/3rd/runtime/bin_release>")

function(lcnc_deploy_base_runtime target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${LCNC_RUNTIME_SNAPSHOT}"
            "$<TARGET_FILE_DIR:${target}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:${target}>
            "$<TARGET_FILE_DIR:${target}>"
        COMMAND_EXPAND_LISTS
        COMMENT "Copying the isolated runtime for ${target}")
endfunction()

add_executable(lcnc_model_envelope_openvdb
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lcnc_model_envelope_openvdb.cpp")
target_link_libraries(lcnc_model_envelope_openvdb PRIVATE
    lcnc_model_envelope_common OpenVDB::openvdb Qt6::Core)
target_compile_definitions(lcnc_model_envelope_openvdb PRIVATE
    QT_NO_FOREACH QT_NO_KEYWORDS)
if(MSVC)
    target_compile_options(lcnc_model_envelope_openvdb PRIVATE /bigobj)
endif()
set_target_properties(lcnc_model_envelope_openvdb PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
        "${LCNC_ENVELOPE_OUTPUT_ROOT}/openvdb/$<CONFIG>")
lcnc_deploy_base_runtime(lcnc_model_envelope_openvdb)
add_custom_command(TARGET lcnc_model_envelope_openvdb POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "$<IF:$<CONFIG:Debug>,${LCNC_ENVELOPE_DEPS_ROOT}/debug/bin,${LCNC_ENVELOPE_DEPS_ROOT}/bin>"
        "$<TARGET_FILE_DIR:lcnc_model_envelope_openvdb>"
    COMMENT "Copying the OpenVDB runtime closure")

# CGAL has been commercially licensed for this product. The executable remains
# an independent process so its dependency/runtime closure cannot leak into the
# LaserCNC main project.
add_executable(lcnc_model_envelope_cgal
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lcnc_model_envelope_cgal.cpp")
target_link_libraries(lcnc_model_envelope_cgal PRIVATE
    lcnc_model_envelope_common CGAL::CGAL Qt6::Core)
set_target_properties(lcnc_model_envelope_cgal PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
        "${LCNC_ENVELOPE_OUTPUT_ROOT}/cgal/$<CONFIG>")
lcnc_deploy_base_runtime(lcnc_model_envelope_cgal)
add_custom_command(TARGET lcnc_model_envelope_cgal POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<IF:$<CONFIG:Debug>,${LCNC_ENVELOPE_DEPS_ROOT}/debug/bin/gmp-10.dll,${LCNC_ENVELOPE_DEPS_ROOT}/bin/gmp-10.dll>"
        "$<IF:$<CONFIG:Debug>,${LCNC_ENVELOPE_DEPS_ROOT}/debug/bin/mpfr-6.dll,${LCNC_ENVELOPE_DEPS_ROOT}/bin/mpfr-6.dll>"
        "$<TARGET_FILE_DIR:lcnc_model_envelope_cgal>"
    COMMENT "Copying the CGAL GMP/MPFR runtime")

add_executable(lcnc_model_envelope_backend_benchmark_test
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/model_envelope_backend_benchmark_test.cpp")
target_link_libraries(lcnc_model_envelope_backend_benchmark_test PRIVATE
    lcnc_model_envelope_common Qt6::Core)
lcnc_deploy_base_runtime(lcnc_model_envelope_backend_benchmark_test)
add_dependencies(lcnc_model_envelope_backend_benchmark_test
    lcnc_model_envelope_openvdb lcnc_model_envelope_cgal)
target_compile_definitions(lcnc_model_envelope_backend_benchmark_test PRIVATE
    LCNC_OPENVDB_ENVELOPE_TOOL_PATH="$<TARGET_FILE:lcnc_model_envelope_openvdb>"
    LCNC_CGAL_ENVELOPE_TOOL_PATH="$<TARGET_FILE:lcnc_model_envelope_cgal>"
    LCNC_AC_TABLE_MODEL_PATH="${LCNC_REPOSITORY_ROOT}/model/精简ac转台.stp")

enable_testing()
add_test(NAME lcnc_model_envelope_backend_benchmark_test
    COMMAND lcnc_model_envelope_backend_benchmark_test)
set_tests_properties(lcnc_model_envelope_backend_benchmark_test PROPERTIES
    TIMEOUT 3600
    RUN_SERIAL TRUE
    LABELS "benchmark;geometry;real-model")
