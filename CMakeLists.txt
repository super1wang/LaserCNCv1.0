cmake_minimum_required(VERSION 3.20)

file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build" _lcnc_legacy_build_dir)
file(REAL_PATH "${CMAKE_BINARY_DIR}" _lcnc_binary_dir)
if(_lcnc_binary_dir STREQUAL _lcnc_legacy_build_dir)
    message(FATAL_ERROR
        "The legacy build/ tree is no longer supported because Visual Studio "
        "and Ninja previously shared it. Use preset acs-gtn (build-cmake/) "
        "or vs-acs-gtn (build-vs/).")
endif()

project(LaserCNC VERSION 1.2.1 LANGUAGES CXX)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake Tools can invoke Ninja without the INCLUDE/LIB environment prepared by
# VsDevCmd.  Keep the compiler and Windows SDK paths in a dedicated interface
# target created before any third-party source tree is added, so every target
# can opt in without relying on lcnc_core as an accidental carrier.
if(WIN32 AND MSVC)
    if(NOT CMAKE_CXX_COMPILER OR NOT CMAKE_RC_COMPILER)
        message(FATAL_ERROR
            "Unable to derive the MSVC/Windows SDK environment: CMAKE_CXX_COMPILER or CMAKE_RC_COMPILER is not configured")
    endif()

    # The Visual Studio generator deliberately caches the resource compiler as
    # the command name "rc".  Resolve that command first: its SDK bin path is
    # the most precise source for both the installed SDK root and version.
    set(_lcnc_rc_compiler_path "${CMAKE_RC_COMPILER}")
    if(NOT IS_ABSOLUTE "${_lcnc_rc_compiler_path}")
        unset(_lcnc_rc_compiler_path CACHE)
        unset(_lcnc_rc_compiler_path)
        find_program(_lcnc_rc_compiler_path NAMES "${CMAKE_RC_COMPILER}")
    endif()

    if(IS_ABSOLUTE "${_lcnc_rc_compiler_path}")
        get_filename_component(_lcnc_sdk_arch_dir "${_lcnc_rc_compiler_path}" DIRECTORY)
        get_filename_component(_lcnc_sdk_version_dir "${_lcnc_sdk_arch_dir}" DIRECTORY)
        get_filename_component(_lcnc_sdk_bin_dir "${_lcnc_sdk_version_dir}" DIRECTORY)
        get_filename_component(_lcnc_sdk_root "${_lcnc_sdk_bin_dir}" DIRECTORY)
        get_filename_component(_lcnc_sdk_version "${_lcnc_sdk_version_dir}" NAME)
    elseif(DEFINED ENV{WindowsSdkDir} AND NOT "$ENV{WindowsSdkDir}" STREQUAL ""
           AND DEFINED ENV{WindowsSDKVersion} AND NOT "$ENV{WindowsSDKVersion}" STREQUAL "")
        # A developer-command environment remains a valid fallback when rc is
        # not on PATH.  WindowsSDKVersion normally carries a trailing slash.
        # 中文翻译：当 PATH 无法解析 rc 时，使用开发者命令行提供的 Windows SDK 根目录和版本。
        file(REAL_PATH "$ENV{WindowsSdkDir}" _lcnc_sdk_root)
        string(REGEX REPLACE "[/\\\\]+$" "" _lcnc_sdk_version "$ENV{WindowsSDKVersion}")
    else()
        # IDE-triggered CMake regeneration may inherit neither WindowsSdkDir
        # nor the SDK bin PATH.  Read the Windows Kits registration and select
        # the newest complete kit so generated .sln builds remain self-contained.
        # 中文翻译：IDE 自动重配置可能没有 SDK 环境变量或 PATH；从注册表定位完整 Windows SDK。
        get_filename_component(_lcnc_sdk_root
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows Kits\\Installed Roots;KitsRoot10]"
            ABSOLUTE)
        if(_lcnc_sdk_root AND EXISTS "${_lcnc_sdk_root}")
            file(REAL_PATH "${_lcnc_sdk_root}" _lcnc_sdk_root)
            file(GLOB _lcnc_sdk_version_dirs LIST_DIRECTORIES true
                "${_lcnc_sdk_root}/Lib/*")
            list(SORT _lcnc_sdk_version_dirs COMPARE NATURAL ORDER DESCENDING)
            foreach(_lcnc_sdk_version_dir IN LISTS _lcnc_sdk_version_dirs)
                get_filename_component(_lcnc_candidate_sdk_version
                    "${_lcnc_sdk_version_dir}" NAME)
                if(EXISTS "${_lcnc_sdk_root}/Include/${_lcnc_candidate_sdk_version}/ucrt"
                   AND EXISTS "${_lcnc_sdk_root}/Lib/${_lcnc_candidate_sdk_version}/um/x64/kernel32.lib"
                   AND EXISTS "${_lcnc_sdk_root}/Lib/${_lcnc_candidate_sdk_version}/ucrt/x64/ucrtd.lib")
                    set(_lcnc_sdk_version "${_lcnc_candidate_sdk_version}")
                    break()
                endif()
            endforeach()
        endif()
        if(NOT _lcnc_sdk_version)
            message(FATAL_ERROR
                "Unable to derive the Windows SDK root: rc is not resolvable and no complete Windows Kits 10 installation was found")
        endif()
    endif()

    get_filename_component(_lcnc_msvc_arch_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    get_filename_component(_lcnc_msvc_host_dir "${_lcnc_msvc_arch_dir}" DIRECTORY)
    get_filename_component(_lcnc_msvc_bin_dir "${_lcnc_msvc_host_dir}" DIRECTORY)
    get_filename_component(_lcnc_msvc_root "${_lcnc_msvc_bin_dir}" DIRECTORY)

    set(_lcnc_msvc_include_dir "${_lcnc_msvc_root}/include")
    set(_lcnc_msvc_x64_lib_dir "${_lcnc_msvc_root}/lib/x64")
    set(_lcnc_sdk_ucrt_include_dir "${_lcnc_sdk_root}/Include/${_lcnc_sdk_version}/ucrt")
    set(_lcnc_sdk_shared_include_dir "${_lcnc_sdk_root}/Include/${_lcnc_sdk_version}/shared")
    set(_lcnc_sdk_um_include_dir "${_lcnc_sdk_root}/Include/${_lcnc_sdk_version}/um")
    set(_lcnc_sdk_winrt_include_dir "${_lcnc_sdk_root}/Include/${_lcnc_sdk_version}/winrt")
    set(_lcnc_sdk_um_x64_lib_dir "${_lcnc_sdk_root}/Lib/${_lcnc_sdk_version}/um/x64")
    set(_lcnc_sdk_ucrt_x64_lib_dir "${_lcnc_sdk_root}/Lib/${_lcnc_sdk_version}/ucrt/x64")

    foreach(_lcnc_required_path IN ITEMS
            "${_lcnc_msvc_include_dir}/map"
            "${_lcnc_msvc_x64_lib_dir}/msvcprtd.lib"
            "${_lcnc_sdk_ucrt_include_dir}"
            "${_lcnc_sdk_shared_include_dir}"
            "${_lcnc_sdk_um_include_dir}"
            "${_lcnc_sdk_winrt_include_dir}"
            "${_lcnc_sdk_um_x64_lib_dir}/kernel32.lib"
            "${_lcnc_sdk_ucrt_x64_lib_dir}/ucrtd.lib")
        if(NOT EXISTS "${_lcnc_required_path}")
            message(FATAL_ERROR
                "Unable to derive a complete MSVC/Windows SDK environment from the configured toolchain; missing: ${_lcnc_required_path}")
        endif()
    endforeach()

    add_library(lcnc_msvc_environment INTERFACE)
    target_include_directories(lcnc_msvc_environment SYSTEM INTERFACE
        "${_lcnc_msvc_include_dir}"
        "${_lcnc_sdk_ucrt_include_dir}"
        "${_lcnc_sdk_shared_include_dir}"
        "${_lcnc_sdk_um_include_dir}"
        "${_lcnc_sdk_winrt_include_dir}"
    )
    target_link_directories(lcnc_msvc_environment INTERFACE
        "${_lcnc_msvc_x64_lib_dir}"
        "${_lcnc_sdk_um_x64_lib_dir}"
        "${_lcnc_sdk_ucrt_x64_lib_dir}"
    )
endif()

# Memory-safety gate.  This stays opt-in because vendor SDK DLLs are not
# instrumented, but every project-owned translation unit can be built with the
# MSVC AddressSanitizer runtime through the `asan` configure preset.
option(LCNC_ENABLE_ASAN "Enable MSVC AddressSanitizer for project code" OFF)
option(LCNC_WARNINGS_AS_ERRORS "Treat warnings in LaserCNC-owned targets as errors" OFF)
if(LCNC_ENABLE_ASAN AND NOT MSVC)
    message(FATAL_ERROR "LCNC_ENABLE_ASAN currently requires MSVC")
endif()
if(LCNC_ENABLE_ASAN)
    # QuaZip is built from source via add_subdirectory below.  ASan's STL
    # annotations must match across every C++ static library, so this cannot
    # be limited to LaserCNC's own targets.
    add_compile_options(
        "$<$<COMPILE_LANGUAGE:CXX>:/fsanitize=address>"
        "$<$<COMPILE_LANGUAGE:CXX>:/Zi>"
    )
    # /fsanitize=address is a compiler switch (not a link.exe switch).  The
    # compiler injects the runtime library; incremental linking is unsupported.
    add_link_options(/INCREMENTAL:NO)
    get_filename_component(_lcnc_msvc_bin_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set(LCNC_ASAN_RUNTIME_DLL "${_lcnc_msvc_bin_dir}/clang_rt.asan_dynamic-x86_64.dll"
        CACHE FILEPATH "MSVC AddressSanitizer runtime DLL")
    if(NOT EXISTS "${LCNC_ASAN_RUNTIME_DLL}")
        message(FATAL_ERROR "LCNC_ENABLE_ASAN requires clang_rt.asan_dynamic-x86_64.dll: ${LCNC_ASAN_RUNTIME_DLL}")
    endif()
    function(lcnc_deploy_asan_runtime target)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${LCNC_ASAN_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:${target}>"
            COMMENT "Copying MSVC AddressSanitizer runtime for ${target}")
    endfunction()
endif()

# AUTOMOC/AUTOUIC/AUTORCC are configured per-target below (not globally) so
# that each static library only scans its own sources. AUTOUIC in particular
# is enabled only on the process library, which is the sole owner of .ui files.

# ─── Qt ───────────────────────────────────────────────────────────────────────
set(LCNC_QT6_ROOT "E:/Qt6.9.1/6.9.1/msvc2022_64" CACHE PATH "Qt6 installation root")
set(Qt6_DIR "${LCNC_QT6_ROOT}/lib/cmake/Qt6" CACHE PATH "Qt6 CMake package directory")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent Network SerialPort UiTools LinguistTools)

# ─── OpenCASCADE ──────────────────────────────────────────────────────────────
set(LCNC_OCCT_ROOT "G:/environment/occt7.9-vc144-64-combined/occt-vc144-64" CACHE PATH "OpenCASCADE installation root")
set(OpenCASCADE_DIR "${LCNC_OCCT_ROOT}/cmake" CACHE PATH "OpenCASCADE CMake package directory")
find_package(OpenCASCADE REQUIRED)

# ─── SARibbon ─────────────────────────────────────────────────────────────────
set(LCNC_SARIBBON_ROOT "F:/wangchao/SARibbon/bin_qt6.9.1_MSVC_x64" CACHE PATH "SARibbon installation root")
set(SARibbonBar_DIR "${LCNC_SARIBBON_ROOT}/lib/cmake/SARibbonBar" CACHE PATH "SARibbonBar CMake package directory")
find_package(SARibbonBar REQUIRED)

# ─── QuaZip (.lcnc 打包后端，替换原 PowerShell Compress-Archive/Expand-Archive) ─
set(LCNC_QUAZIP_ROOT "E:/GitHub/quazip" CACHE PATH "QuaZip source tree")
set(LCNC_ZLIB_ROOT "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/zlib-1.2.8-vc14-64"
    CACHE PATH "zlib root used by QuaZip (defaults to the OCCT 3rdparty zlib)")
# 让 QuaZip 的 find_package(ZLIB) 命中 OCCT 自带的静态 zlib（无需额外 DLL / 不联网下载）。
set(ZLIB_INCLUDE_DIR "${LCNC_ZLIB_ROOT}/include" CACHE PATH "" FORCE)
set(ZLIB_LIBRARY "${LCNC_ZLIB_ROOT}/lib/zlibstatic.lib" CACHE FILEPATH "" FORCE)
# QuaZip 构建选项：静态库；关闭 bzip2/QTextCodec/测试/安装（规避网络拉取与 Core5Compat 依赖）。
set(QUAZIP_QT_MAJOR_VERSION 6 CACHE STRING "" FORCE)
set(QUAZIP_USE_QT_ZLIB OFF CACHE BOOL "" FORCE)
set(QUAZIP_BZIP2 OFF CACHE BOOL "" FORCE)
set(QUAZIP_ENABLE_QTEXTCODEC OFF CACHE BOOL "" FORCE)
set(QUAZIP_INSTALL OFF CACHE BOOL "" FORCE)
set(QUAZIP_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(QUAZIP_FETCH_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF) # 构建静态 QuaZip（CMP0077 使其 option() 尊重此普通变量）
add_subdirectory("${LCNC_QUAZIP_ROOT}" "${CMAKE_BINARY_DIR}/quazip-build")
if(TARGET QuaZip AND TARGET lcnc_msvc_environment)
    # QuaZip is compiled from source, so it must receive the same IDE-safe
    # standard-library and SDK search paths as LaserCNC-owned targets.
    # QuaZip itself uses CMake's legacy plain signature for this target.
    target_link_libraries(QuaZip lcnc_msvc_environment)
endif()

# ─── Optional process device SDKs ─────────────────────────────────────────────
option(LCNC_WITH_ACS "Enable ACS motion controller adapter" ON)
option(LCNC_WITH_GTN "Enable GTN motion controller adapter" ON)
option(LCNC_WITH_BDAQ "Enable BDAQ IO adapter" OFF)
option(LCNC_WITH_REAL_LASER "Enable real laser adapters" OFF)

set(LCNC_PROCESS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rd" CACHE PATH "Process device SDK root")
set(LCNC_PROCESS_SDK_INCLUDE_DIR "${LCNC_PROCESS_SDK_ROOT}/include_3rd" CACHE PATH "Process device SDK include directory")
set(LCNC_PROCESS_SDK_LIB_DEBUG_DIR "${LCNC_PROCESS_SDK_ROOT}/lib_3rd/debug" CACHE PATH "Process device SDK debug library directory")
set(LCNC_PROCESS_SDK_LIB_RELEASE_DIR "${LCNC_PROCESS_SDK_ROOT}/lib_3rd/release" CACHE PATH "Process device SDK release library directory")

set(LCNC_ACS_INCLUDE_DIR "${LCNC_PROCESS_SDK_INCLUDE_DIR}" CACHE PATH "ACS SDK include directory")
set(LCNC_ACS_LIB "${LCNC_PROCESS_SDK_LIB_DEBUG_DIR}/ACSCL_x64.LIB" CACHE FILEPATH "ACS controller import library")
set(LCNC_ACS_SIMULATOR_PROGRAM "${CMAKE_CURRENT_SOURCE_DIR}/resources/process/Simulator.prg"
    CACHE FILEPATH "ACS Simulator buffer program deployed next to LaserCNC.exe")
set(LCNC_GTN_INCLUDE_DIR "${LCNC_PROCESS_SDK_ROOT}/gts" CACHE PATH "GTN SDK include directory")
set(LCNC_GTN_LIB "${LCNC_PROCESS_SDK_ROOT}/gts/gts.lib" CACHE FILEPATH "GTN controller import library")
set(LCNC_GTN_RUNTIME_DLL "${LCNC_PROCESS_SDK_ROOT}/gts/gts.dll" CACHE FILEPATH "GTN controller runtime DLL")
set(LCNC_BDAQ_INCLUDE_DIR "${LCNC_PROCESS_SDK_INCLUDE_DIR}" CACHE PATH "BDAQ SDK include directory")
set(LCNC_BDAQ_LIB "" CACHE FILEPATH "BDAQ import library")
set(LCNC_REAL_LASER_SDK_ROOT "${LCNC_PROCESS_SDK_ROOT}" CACHE PATH "Real laser SDK root or protocol support root")

# ─── Boost (consumed by process module: ToolFactory, SerialPort, ACSMotionControl) ──
# Header includes: <boost/lexical_cast.hpp>, <boost/thread.hpp>, <boost/chrono.hpp>
# Compiled libs needed by serial_port.cpp (boost::this_thread, boost::chrono): boost_thread, boost_chrono, boost_system
set(LCNC_BOOST_ROOT "G:/environment/vcpkg-master/vcpkg/installed/x64-windows" CACHE PATH "Boost installation root (vcpkg layout: include/ + lib/ + debug/lib/)")
set(LCNC_BOOST_INCLUDE_DIR "${LCNC_BOOST_ROOT}/include" CACHE PATH "Boost include directory")
set(LCNC_BOOST_LIB_DIR_RELEASE "${LCNC_BOOST_ROOT}/lib" CACHE PATH "Boost release library directory")
set(LCNC_BOOST_LIB_DIR_DEBUG "${LCNC_BOOST_ROOT}/debug/lib" CACHE PATH "Boost debug library directory")

function(lcnc_require_existing path label)
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "${label} does not exist: ${path}")
    endif()
endfunction()

if(LCNC_WITH_ACS)
    lcnc_require_existing("${LCNC_ACS_INCLUDE_DIR}/ACSC.h" "LCNC_WITH_ACS requires ACSC.h")
    lcnc_require_existing("${LCNC_ACS_LIB}" "LCNC_WITH_ACS requires LCNC_ACS_LIB")
    lcnc_require_existing("${LCNC_ACS_SIMULATOR_PROGRAM}" "LCNC_WITH_ACS requires Simulator.prg")
endif()
if(LCNC_WITH_GTN)
    lcnc_require_existing("${LCNC_GTN_INCLUDE_DIR}/gts.h" "LCNC_WITH_GTN requires gts.h")
    lcnc_require_existing("${LCNC_GTN_LIB}" "LCNC_WITH_GTN requires LCNC_GTN_LIB")
    lcnc_require_existing("${LCNC_GTN_RUNTIME_DLL}" "LCNC_WITH_GTN requires LCNC_GTN_RUNTIME_DLL")
endif()
if(LCNC_WITH_BDAQ)
    lcnc_require_existing("${LCNC_BDAQ_INCLUDE_DIR}" "LCNC_WITH_BDAQ requires LCNC_BDAQ_INCLUDE_DIR")
    lcnc_require_existing("${LCNC_BDAQ_LIB}" "LCNC_WITH_BDAQ requires LCNC_BDAQ_LIB")
endif()
if(LCNC_WITH_REAL_LASER)
    lcnc_require_existing("${LCNC_REAL_LASER_SDK_ROOT}" "LCNC_WITH_REAL_LASER requires LCNC_REAL_LASER_SDK_ROOT")
endif()

# ─── Shared INTERFACE targets ─────────────────────────────────────────────────
# lcnc_common: include paths, defines, and compile options shared by every
# library and the executable. Linked PUBLIC by lcnc_core so that everything
# downstream inherits it transitively.
add_library(lcnc_common INTERFACE)
add_library(lcnc_project_warnings INTERFACE)
if(MSVC)
    target_compile_options(lcnc_project_warnings INTERFACE
        /W4
        /external:anglebrackets
        /external:W0
    )
    if(LCNC_WARNINGS_AS_ERRORS)
        target_compile_options(lcnc_project_warnings INTERFACE /WX)
    endif()
endif()
target_link_libraries(lcnc_common INTERFACE lcnc_project_warnings)
if(TARGET lcnc_msvc_environment)
    target_link_libraries(lcnc_common INTERFACE lcnc_msvc_environment)
endif()
target_include_directories(lcnc_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/include_3rd
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/magic_enum
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/toml11
    ${OpenCASCADE_INCLUDE_DIR}
)
target_compile_definitions(lcnc_common INTERFACE
    LCNC_PROCESS_HAS_ACS=$<BOOL:${LCNC_WITH_ACS}>
    LCNC_PROCESS_HAS_GTN=$<BOOL:${LCNC_WITH_GTN}>
    LCNC_PROCESS_HAS_BDAQ=$<BOOL:${LCNC_WITH_BDAQ}>
    LCNC_PROCESS_HAS_REAL_LASER=$<BOOL:${LCNC_WITH_REAL_LASER}>
)
if(WIN32)
    target_compile_definitions(lcnc_common INTERFACE
        NOMINMAX
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
    )
endif()
if(MSVC)
    # /FS – serialise PDB writes (parallel cl.exe within the same target)
    # /FIwindows.h – force-include windows.h before any other header to prevent
    # WIN32_LEAN_AND_MEAN (defined inside OpenCASCADE's OpenGl_glext.h) from
    # excluding OLE/NLS headers that Qt6 and the Windows SDK need in the same
    # translation unit.
    target_compile_options(lcnc_common INTERFACE /FS /FIwindows.h)
endif()

# lcnc_occ: OCC link libraries + 3rd-party search paths required by OCCT.
add_library(lcnc_occ INTERFACE)

set(OCC_LIBS
    # Foundation
    TKernel TKMath
    # Modeling Data
    TKG2d TKG3d TKGeomBase TKBRep
    # Modeling Algorithms
    TKGeomAlgo TKTopAlgo TKPrim TKBO TKShHealing TKBool TKOffset TKHLR TKMesh
    # Visualization
    TKService TKV3d TKOpenGl TKMeshVS
    # Application Framework (XDE/XCAF)
    TKCDF TKLCAF TKCAF TKBinL TKXmlL TKBin TKXml TKStdL TKStd TKVCAF
    # Data Exchange
    TKDE TKXSBase TKDESTEP TKXCAF TKDEIGES TKDESTL TKRWMesh TKDECascade TKBinXCAF TKXmlXCAF TKDEGLTF
)

set(LCNC_OCCT_3RDPARTY_DIR "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64" CACHE PATH "OpenCASCADE third-party library root")

target_link_libraries(lcnc_occ INTERFACE ${OCC_LIBS})
target_link_directories(lcnc_occ INTERFACE
    "${LCNC_OCCT_3RDPARTY_DIR}/openvr-1.14.15-64/lib/win64"
    "${LCNC_OCCT_3RDPARTY_DIR}/freeimage-3.18.0-x64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/freetype-2.13.3-x64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/gl2ps-1.3.8-vc14-64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/tbb-2021.13.0-x64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/draco-1.4.1-vc14-64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/tcltk-8.6.15-x64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/zlib-1.2.8-vc14-64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/lzma-5.2.2-vc14-64/lib"
    "${LCNC_OCCT_3RDPARTY_DIR}/ffmpeg-3.3.4-64/lib"
)

# ─── Sources ──────────────────────────────────────────────────────────────────
set(SRCS_CORE
    src/core/logging/logger.cpp
    src/core/settings/toml_config.cpp
    src/core/settings/app_settings.cpp
    src/core/project/lcnc_project_manifest.cpp
    src/core/project/lcnc_project_package.cpp
    src/core/project/lcnc_project_session.cpp
    src/core/project/project_workspace.cpp
    src/core/project/lcnc_project_manager.cpp
    src/core/project/cam/cam_data_manager.cpp
    src/core/project/cam/cam_toolpath_io.cpp
    src/core/project/cam/layer_container.cpp
    src/core/project/cam/layer_manager.cpp
    src/core/project/cam/layer_manager.h
    src/core/machine/machine_workspace.cpp
    src/core/machine/machine_workspace.h
    src/core/document/lcnc_document.cpp
    src/core/document/xcaf_utils.cpp
    src/core/kinematics/machine_kinematics.cpp
    src/core/kinematics/machine_topology.cpp
    src/core/kinematics/toolpath_kinematics_solver.cpp
    src/core/kinematics/machine_configuration_service.cpp
    src/core/kinematics/machine_pose.cpp
    src/core/kinematics/ik_solver.cpp
    src/core/algorithms/cad/primitives.cpp
    src/core/algorithms/cad/boolean_ops.cpp
    src/core/algorithms/cad/measure.cpp
    src/core/algorithms/cad/sketch.cpp
    src/core/algorithms/cad/features.cpp
    src/core/algorithms/cad/transform_ops.cpp
    src/core/algorithms/cam/face_classifier.cpp
    src/core/algorithms/cam/laser_toolpath.cpp
    src/core/algorithms/cam/contour_order_planner.cpp
    src/core/task/task_progress.cpp
    src/core/task/task_manager.cpp
    src/core/task/module_task_scope.cpp
    src/core/command/commands_api.cpp
    src/core/kernel/service_registry.cpp
    src/core/kernel/event_bus.cpp
    src/core/kernel/module_registry.cpp
    src/core/kernel/kernel.cpp
    src/core/services/selection_service.cpp
)

set(SRCS_VIEW
    src/view/graphics_scene.cpp
    src/view/rendering_manager.cpp
    src/view/shape_object_driver.cpp
    src/view/gui_application.cpp
    src/view/gui_document.cpp
    src/view/widget_occ_view.cpp
    src/view/toolpath_renderer.cpp
    src/view/travel_path_renderer.cpp
    src/view/contour_order_label_renderer.cpp
    src/view/machine_guide_renderer.cpp
    src/view/world_axes_renderer.cpp
    src/view/sketch_overlay_renderer.cpp
    src/view/transform_gizmo_renderer.cpp
)

set(SRCS_MODULE_CAD
    src/modules/cad/commands/commands_cad.h
    src/modules/cad/cad_module.cpp
    src/modules/cad/document/cad_document_state.cpp
    src/modules/cad/document/cad_document_registry.cpp
    src/modules/cad/services/cad_modeling_session.cpp
    src/modules/cad/services/cad_document_io_service.h
    src/modules/cad/services/cad_document_io_service.cpp
    src/modules/cad/services/sketch_manager.cpp
    src/modules/cad/services/shape_service.cpp
    src/modules/cad/selection/cad_selection_resolver.cpp
    src/modules/cad/task/cad_command_dispatcher.cpp
    src/modules/cad/task/cad_tool_registry.cpp
    src/modules/cad/task/cad_tool_filter.cpp
    src/modules/cad/commands/commands_file.cpp
    src/modules/cad/commands/commands_edit.cpp
    src/modules/cad/commands/command_helpers.cpp
    src/modules/cad/commands/commands_cad_sketch.cpp
    src/modules/cad/commands/commands_cad_view.cpp
    src/modules/cad/commands/commands_cad_primitives.cpp
    src/modules/cad/commands/commands_cad_transform.cpp
    src/modules/cad/commands/commands_cad_boolean.cpp
    src/modules/cad/commands/commands_cad_measure.cpp
    src/modules/cad/commands/commands_cad_delete.cpp
    src/modules/cad/ui/widget_cad_task_panel.cpp
    src/modules/cad/ui/ribbon_cad_tab.cpp
)

set(SRCS_MODULE_CAM
    src/modules/cam/cam_module.cpp
    src/modules/cam/contracts/toolpath_export_dto.cpp
    src/modules/cam/settings/cam_config.cpp
    src/modules/cam/services/machine_axis_detector.cpp
    src/modules/cam/services/cam_display_projection_service.cpp
    src/modules/cam/services/machining_face_pipeline_service.cpp
    src/modules/cam/services/toolpath_generation_service.cpp
    src/modules/cam/services/machine_io.cpp
    src/modules/cam/services/reference_pick.cpp
    src/modules/cam/commands/commands_machine.cpp
    src/modules/cam/commands/commands_cam.cpp
    src/modules/cam/ui/widget_machine_panel.cpp
    src/modules/cam/ui/widget_machine_tree.cpp
    src/modules/cam/ui/widget_machine_tree.h
    src/modules/cam/ui/widget_toolpath_panel.cpp
    src/modules/cam/ui/dialog_mark_axes.cpp
    src/modules/cam/ui/dialog_axis_calibration_wizard.cpp
    src/modules/cam/ui/ribbon_cam_tab.cpp
)

set(SRCS_MODULE_PROCESS
    src/modules/process/process_module.cpp
    src/modules/process/runtime/process_axis_utilities.cpp
    src/modules/process/runtime/device_command_queue.cpp
    src/modules/process/runtime/device_command_queue.h
    src/modules/process/runtime/process_run_coordinator.cpp
    src/modules/process/runtime/process_run_coordinator.h
    src/modules/process/settings/process_settings_service.cpp
    src/modules/process/settings/process_parameter_registry.cpp
    src/modules/process/settings/process_property_model.cpp
    src/modules/process/settings/process_io_table_model.cpp
    src/modules/process/settings/process_settings_dialog.cpp
    src/modules/process/runtime/process_device_runtime.cpp
    src/modules/process/runtime/process_device_session.cpp
    src/modules/process/runtime/process_device_configuration.cpp
    src/modules/process/runtime/process_cutting_safety.h
    src/modules/process/runtime/process_cutting_safety.cpp
    src/modules/process/runtime/process_preflight_types.h
    src/modules/process/runtime/process_preflight_service.h
    src/modules/process/runtime/process_preflight_service.cpp
    src/modules/process/runtime/process_connection_service.h
    src/modules/process/runtime/process_connection_service.cpp
    src/modules/process/runtime/process_manual_motion_service.h
    src/modules/process/runtime/process_manual_motion_service.cpp
    src/modules/process/runtime/process_interactive_io_service.h
    src/modules/process/runtime/process_interactive_io_service.cpp
    src/modules/process/runtime/process_status_service.h
    src/modules/process/runtime/process_status_service.cpp
    src/modules/process/setting/builtin_io_defs.cpp
    src/modules/process/device/motion_control/motion_control.cpp
    src/modules/process/device/motion_control/simulate_cmhp_motion_control.cpp
    src/modules/process/device/connect/serial_port.cpp
    src/modules/process/device/laser/ld_factory.cpp
    src/modules/process/device/laser/analog_laser_device.cpp
    src/modules/process/device/laser/laser_device.cpp
    src/modules/process/device/laser/simulator_laser_device.cpp
    src/modules/process/tool/tool.cpp
    src/modules/process/tool/tool_factory.cpp
    src/modules/process/monitor/process_monitor_types.h
    src/modules/process/monitor/process_monitor_service.h
    src/modules/process/monitor/process_monitor_service.cpp
    src/modules/process/runtime/process_cancellation_token.h
    src/modules/process/runtime/process_interrupt_context.h
    src/modules/process/runtime/process_interrupt_context.cpp
    src/modules/process/runtime/machine_pose5.h
    src/modules/process/runtime/motion_params.h
    src/modules/process/runtime/axis_map.h
    src/modules/process/runtime/axis_map.cpp
    src/modules/process/runtime/i_motion_command_sink.h
    src/modules/process/runtime/gtn_buffered_command_sink.h
    src/modules/process/runtime/pure_simulation_sink.h
    src/modules/process/runtime/pure_simulation_sink.cpp
    src/modules/process/runtime/motion_sink_factory.cpp
    src/modules/process/toolpath/process_toolpath_service.cpp
    src/modules/process/cutting/normal_cutting_manager.h
    src/modules/process/cutting/normal_cutting_manager.cpp
    src/modules/process/cutting/pure_simulation_toolpath_ticker.h
    src/modules/process/cutting/pure_simulation_toolpath_ticker.cpp
    src/modules/process/cutting/process_cutting_plan_service.h
    src/modules/process/cutting/process_cutting_plan_service.cpp
    src/modules/process/workflow/process_node.cpp
    src/modules/process/workflow/process_node_registry.cpp
    src/modules/process/workflow/process_flow_document.cpp
    src/modules/process/workflow/process_flow_store.cpp
    src/modules/process/workflow/process_workflow_service.h
    src/modules/process/workflow/process_workflow_service.cpp
    src/modules/process/steps/process_step_registry.cpp
    src/modules/process/steps/process_step_builtin_registration.cpp
    src/modules/process/steps/services/process_workflow_services.cpp
    src/modules/process/steps/start/start_step.cpp
    src/modules/process/steps/stop/stop_step.cpp
    src/modules/process/steps/single_axis_move/single_axis_move_step.cpp
    src/modules/process/steps/multi_axis_move/multi_axis_move_step.cpp
    src/modules/process/steps/set_axis_position/set_axis_position_step.cpp
    src/modules/process/steps/output_signal/output_signal_step.cpp
    src/modules/process/steps/input_signal_wait/input_signal_wait_step.cpp
    src/modules/process/steps/normal_cutting/normal_cutting_step.cpp
    src/modules/process/steps/loop/loop_step.cpp
    src/modules/process/steps/if/if_step.cpp
    src/modules/process/ui/process_flow_model.cpp
    src/modules/process/ui/process_flow_tree_view.cpp
    src/modules/process/ui/process_node_edit_dialog.cpp
    src/modules/process/ui/process_flow_widget.cpp
    src/modules/process/commands/commands_process.cpp
    src/modules/process/ui/widget_laser_control.cpp
    src/modules/process/ui/ribbon_process_tab.cpp
    src/modules/process/execution/process_workflow_executor.cpp
    src/modules/process/execution/process_node_executor_registry.cpp
)

if(LCNC_WITH_ACS)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/device/motion_control/acs_motion_control.cpp
        src/modules/process/runtime/acs_text_command_sink.h
        src/modules/process/runtime/acs_text_command_sink.cpp
    )
endif()
if(LCNC_WITH_GTN)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/device/motion_control/gtn_motion_control.cpp
        src/modules/process/runtime/gtn_buffered_command_sink.cpp
    )
    # The GTN adapter implements the complete historical vendor surface.  A
    # small number of SDK-required/no-op parameters and retained diagnostic
    # locals are intentionally unused; scope these exceptions to this source
    # while keeping /W4 and /WX on every other Process translation unit.
    set_source_files_properties(
        src/modules/process/device/motion_control/gtn_motion_control.cpp
        PROPERTIES COMPILE_OPTIONS "/wd4100;/wd4101;/wd4189"
    )
endif()
if(LCNC_WITH_REAL_LASER)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/device/connect/http_client.cpp
        src/modules/process/device/laser/ipg_laser_device.cpp
        src/modules/process/device/laser/pharos_laser_device.cpp
        src/modules/process/device/laser/raycus_laser_device.cpp
        src/modules/process/device/laser/raycus_air_cool_laser_device.cpp
        src/modules/process/device/laser/raycus_qcw_laser_device.cpp
        src/modules/process/device/laser/ultron_laser_device.cpp
    )
endif()

set(SRCS_APP
    src/app/controllers/cad_task_panel_controller.cpp
    src/app/controllers/project_explorer_controller.cpp
    src/app/controllers/view_state_controller.cpp
    src/app/controllers/workspace_presenter.cpp
    src/app/app_context.cpp
    src/app/command_registry.cpp
    src/app/commands/commands_display.cpp
    src/app/dialog/dialog_options.cpp
    src/app/dialog/dialog_task_manager.cpp
    src/app/project_explorer_model.cpp
    src/app/project_explorer_tree_utils.cpp
    src/app/start_guide_widget.cpp
    src/app/main_window.cpp
)

# ─── Static libraries (one per layer) ─────────────────────────────────────────
# core ────────────────────────────────────────────────────────────────────────
add_library(lcnc_core STATIC ${SRCS_CORE})
set_target_properties(lcnc_core PROPERTIES AUTOMOC ON)
target_link_libraries(lcnc_core PUBLIC
    lcnc_common
    lcnc_occ
    Qt6::Core
    Qt6::Gui
    Qt6::Concurrent
)

# QuaZip 仅在 lcnc_project_package.cpp 实现内部使用（不出现在公开头文件）→ PRIVATE。
target_link_libraries(lcnc_core PRIVATE QuaZip::QuaZip)

# view ────────────────────────────────────────────────────────────────────────
add_library(lcnc_view STATIC ${SRCS_VIEW})
set_target_properties(lcnc_view PROPERTIES AUTOMOC ON)
target_link_libraries(lcnc_view PUBLIC
    lcnc_core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)

# cad module ──────────────────────────────────────────────────────────────────
add_library(lcnc_module_cad STATIC ${SRCS_MODULE_CAD})
set_target_properties(lcnc_module_cad PROPERTIES AUTOMOC ON)
target_link_libraries(lcnc_module_cad PUBLIC
    lcnc_view
    SARibbonBar::SARibbonBar
    Qt6::UiTools
)

# cam module (depends on cad — see cam_module.cpp pulling cad/services) ───────
add_library(lcnc_module_cam STATIC ${SRCS_MODULE_CAM})
set_target_properties(lcnc_module_cam PROPERTIES AUTOMOC ON)
target_link_libraries(lcnc_module_cam PUBLIC
    lcnc_view
    lcnc_module_cad
    SARibbonBar::SARibbonBar
)

add_library(lcnc_module_process STATIC ${SRCS_MODULE_PROCESS})
set_target_properties(lcnc_module_process PROPERTIES
    AUTOMOC ON
)
target_include_directories(lcnc_module_process PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/system
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/setting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/tool
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/motion_control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/laser
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/connect
)
# Boost: tool_factory.cpp, serial_port.cpp, acs_motion_control.cpp pull in
# <boost/lexical_cast.hpp>, <boost/thread.hpp>, <boost/chrono.hpp>. The first
# is header-only; the latter two need compiled libs. We link them by absolute
# path (PRIVATE — only this library's TUs reference these symbols, the
# auto-link pragmas in our .obj files would also work but are unreliable
# across boost versions).
target_include_directories(lcnc_module_process PRIVATE "${LCNC_BOOST_INCLUDE_DIR}")
set(_lcnc_boost_lib_dir "$<IF:$<CONFIG:Debug>,${LCNC_BOOST_LIB_DIR_DEBUG},${LCNC_BOOST_LIB_DIR_RELEASE}>")
set(_lcnc_boost_lib_suffix "$<IF:$<CONFIG:Debug>,vc143-mt-gd-x64-1_88,vc143-mt-x64-1_88>")
target_link_libraries(lcnc_module_process PRIVATE
    "${_lcnc_boost_lib_dir}/boost_thread-${_lcnc_boost_lib_suffix}.lib"
    "${_lcnc_boost_lib_dir}/boost_chrono-${_lcnc_boost_lib_suffix}.lib"
    "${_lcnc_boost_lib_dir}/boost_system-${_lcnc_boost_lib_suffix}.lib"
)
target_link_libraries(lcnc_module_process PUBLIC
    lcnc_view
    lcnc_module_cam
    SARibbonBar::SARibbonBar
    Qt6::Network
    Qt6::UiTools
)
if(LCNC_WITH_ACS)
    target_include_directories(lcnc_module_process PRIVATE "${LCNC_ACS_INCLUDE_DIR}")
    target_link_libraries(lcnc_module_process PRIVATE "${LCNC_ACS_LIB}")
endif()
if(LCNC_WITH_GTN)
    target_include_directories(lcnc_module_process PRIVATE "${LCNC_GTN_INCLUDE_DIR}")
    target_link_libraries(lcnc_module_process PRIVATE "${LCNC_GTN_LIB}")
endif()
if(LCNC_WITH_BDAQ)
    target_include_directories(lcnc_module_process PRIVATE "${LCNC_BDAQ_INCLUDE_DIR}")
    target_link_libraries(lcnc_module_process PRIVATE "${LCNC_BDAQ_LIB}")
endif()
target_link_libraries(lcnc_module_process PUBLIC Qt6::SerialPort)

# app — bundles the three modules so the exe only needs to link lcnc_app ──────
add_library(lcnc_app STATIC ${SRCS_APP})
set_target_properties(lcnc_app PROPERTIES AUTOMOC ON)
target_link_libraries(lcnc_app PUBLIC
    lcnc_module_cad
    lcnc_module_cam
    lcnc_module_process
    SARibbonBar::SARibbonBar
)

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(LaserCNC WIN32
    src/main.cpp
    resources/resources.qrc
)
qt_add_lrelease(LaserCNC
    TS_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/translations/lasercnc_zh_CN.ts"
    QM_FILES_OUTPUT_VARIABLE LCNC_TRANSLATION_QM_FILES
)
# qt_add_lrelease writes .qm files into the build tree, so its output paths are
# absolute.  Qt 6 requires an explicit resource alias for such generated files.
set_source_files_properties(${LCNC_TRANSLATION_QM_FILES}
    PROPERTIES QT_RESOURCE_ALIAS "lasercnc_zh_CN.qm"
)
qt_add_resources(LaserCNC "lcnc_translations"
    PREFIX "/i18n"
    FILES ${LCNC_TRANSLATION_QM_FILES}
)
# Generator trees and feature variants remain independent through deployment.
# A runtime folder contains the executable together with its matching Qt/OCC/
# SDK DLLs, TOML and Simulator.prg, so mixing variants can run the wrong binary.
# 中文翻译：运行目录按生成器和功能变体隔离，避免可执行文件、DLL、配置和模拟器程序混用。
unset(LCNC_RUNTIME_OUTPUT_ROOT CACHE)
set(LCNC_RUNTIME_OUTPUT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/x64")
set(LCNC_RUNTIME_VARIANT "" CACHE STRING
    "Runtime deployment variant, for example ninja, ninja-asan, ninja-all-off or vs")
if(NOT LCNC_RUNTIME_VARIANT MATCHES "^(vs|ninja(-[a-z0-9]+)*)$")
    message(FATAL_ERROR
        "LCNC_RUNTIME_VARIANT must be a safe named VS or Ninja variant; got '${LCNC_RUNTIME_VARIANT}'")
endif()
if(CMAKE_GENERATOR MATCHES "Visual Studio" AND NOT LCNC_RUNTIME_VARIANT STREQUAL "vs")
    message(FATAL_ERROR
        "Visual Studio builds must deploy with LCNC_RUNTIME_VARIANT=vs; got '${LCNC_RUNTIME_VARIANT}'")
elseif(CMAKE_GENERATOR MATCHES "Ninja" AND NOT LCNC_RUNTIME_VARIANT MATCHES "^ninja")
    message(FATAL_ERROR
        "Ninja builds must deploy with an LCNC_RUNTIME_VARIANT beginning with ninja; got '${LCNC_RUNTIME_VARIANT}'")
elseif(NOT CMAKE_GENERATOR MATCHES "Visual Studio|Ninja")
    message(FATAL_ERROR "Unsupported generator for LaserCNC runtime deployment: ${CMAKE_GENERATOR}")
endif()
set(LCNC_RUNTIME_OUTPUT_DIRECTORY
    "${LCNC_RUNTIME_OUTPUT_ROOT}/${LCNC_RUNTIME_VARIANT}/$<CONFIG>")
set_target_properties(LaserCNC PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    RUNTIME_OUTPUT_DIRECTORY "${LCNC_RUNTIME_OUTPUT_DIRECTORY}"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/symbols/$<CONFIG>"
)
target_link_libraries(LaserCNC PRIVATE lcnc_app)

if(MSVC)
    # Runtime folders contain deployable files only. Keep incremental-link
    # state and symbols under build/ so they cannot be mistaken for releases.
    target_link_options(LaserCNC PRIVATE "$<$<CONFIG:Debug>:/INCREMENTAL:NO>")
endif()

if(LCNC_WITH_ACS)
    # Relink (and therefore rerun POST_BUILD) when the simulator program
    # changes; otherwise an incremental build could leave a stale deployed
    # copy beside the executable.
    set_property(TARGET LaserCNC APPEND PROPERTY LINK_DEPENDS
        "${LCNC_ACS_SIMULATOR_PROGRAM}")
    add_custom_command(TARGET LaserCNC POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LCNC_ACS_SIMULATOR_PROGRAM}"
            "$<TARGET_FILE_DIR:LaserCNC>/Simulator.prg"
        COMMENT "Deploying ACS Simulator program")
endif()

if(WIN32)
    set_target_properties(LaserCNC PROPERTIES
        WIN32_EXECUTABLE TRUE
        LINK_FLAGS "/ENTRY:mainCRTStartup"
    )
endif()

set(LCNC_RUNTIME_CONFIG_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/config.toml"
    "${CMAKE_CURRENT_SOURCE_DIR}/Peripheral.toml"
)
add_custom_command(TARGET LaserCNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:LaserCNC>/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:LaserCNC>/logs"
    COMMAND ${CMAKE_COMMAND} -E rm -f
        "$<TARGET_FILE_DIR:LaserCNC>/LaserCNC.pdb"
        "$<TARGET_FILE_DIR:LaserCNC>/LaserCNC.ilk"
    COMMENT "Preparing deployable LaserCNC runtime directory")
foreach(_lcnc_runtime_config IN LISTS LCNC_RUNTIME_CONFIG_FILES)
    if(EXISTS "${_lcnc_runtime_config}")
        add_custom_command(TARGET LaserCNC POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_lcnc_runtime_config}"
                "$<TARGET_FILE_DIR:LaserCNC>"
            COMMENT "Deploying runtime configuration")
    endif()
endforeach()

# ─── Deploy runtime DLLs ──────────────────────────────────────────────────────
# All runtime DLLs (OCC, Qt6 + plugins, SARibbon, boost, ACS, etc.) live as a
# self-contained snapshot in 3rd/runtime/bin_<debug|release>/. POST_BUILD copies
# the matching variant into the exe directory, so running LaserCNC.exe needs no
# external Qt / OCC install. To refresh this snapshot after updating an SDK,
# re-run scripts/refresh_runtime_dlls.* (or windeployqt manually) and commit
# the changed DLLs.
set(LCNC_RUNTIME_BIN_DIR_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}/3rd/runtime/bin_debug"   CACHE PATH "Runtime DLL directory (Debug)")
set(LCNC_RUNTIME_BIN_DIR_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/3rd/runtime/bin_release" CACHE PATH "Runtime DLL directory (Release)")
set(LCNC_ZLIB_RUNTIME_DLL "${CMAKE_CURRENT_SOURCE_DIR}/3rd/runtime/common/zlib1.dll"
    CACHE FILEPATH "Compatibility zlib runtime DLL for LaserCNC deployments")
set(LCNC_TBB_BIN_DIR "C:/work/occt/3rdparty-vc14-64/tbb-2021.13.0-x64/bin"
    CACHE PATH "OCCT TBB runtime DLL directory")

if(NOT EXISTS "${LCNC_ZLIB_RUNTIME_DLL}")
    message(FATAL_ERROR
        "LaserCNC zlib compatibility runtime is missing: ${LCNC_ZLIB_RUNTIME_DLL}")
endif()

function(lcnc_deploy_tbb_runtime target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${LCNC_TBB_BIN_DIR}/tbb12_debug.dll,${LCNC_TBB_BIN_DIR}/tbb12.dll>"
            "$<TARGET_FILE_DIR:${target}>"
        COMMENT "Copying OCCT TBB runtime for ${target}")
endfunction()

function(lcnc_deploy_qt_offscreen_platform target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target}>/platforms"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${LCNC_QT6_ROOT}/plugins/platforms/qoffscreend.dll,${LCNC_QT6_ROOT}/plugins/platforms/qoffscreen.dll>"
            "$<TARGET_FILE_DIR:${target}>/platforms"
        COMMENT "Copying Qt offscreen platform plugin for ${target}")
endfunction()

function(lcnc_deploy_acs_simulator_program target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LCNC_ACS_SIMULATOR_PROGRAM}"
            "$<TARGET_FILE_DIR:${target}>/Simulator.prg"
        COMMENT "Deploying ACS Simulator program for ${target}")
endfunction()

function(lcnc_deploy_gtn_runtime target)
    if(LCNC_WITH_GTN)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${LCNC_GTN_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:${target}>"
            COMMENT "Copying GTN runtime DLL for ${target}")
    endif()
endfunction()

set(_lcnc_runtime_bin "$<IF:$<CONFIG:Debug>,${LCNC_RUNTIME_BIN_DIR_DEBUG},${LCNC_RUNTIME_BIN_DIR_RELEASE}>")
if(EXISTS "${LCNC_RUNTIME_BIN_DIR_DEBUG}" AND EXISTS "${LCNC_RUNTIME_BIN_DIR_RELEASE}")
    # All test executables share ${CMAKE_BINARY_DIR}/<config>. A POST_BUILD
    # copy on each target races under a parallel build, so deploy the snapshot
    # once per output directory and make consumers depend on that deployment.
    add_custom_target(lcnc_runtime_deploy_build
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_lcnc_runtime_bin}"
            "${CMAKE_BINARY_DIR}/$<CONFIG>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LCNC_ZLIB_RUNTIME_DLL}"
            "${CMAKE_BINARY_DIR}/$<CONFIG>"
        COMMENT "Copying runtime DLLs to build output directory"
    )
    add_custom_target(lcnc_runtime_deploy_app
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_lcnc_runtime_bin}"
            "$<TARGET_FILE_DIR:LaserCNC>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LCNC_ZLIB_RUNTIME_DLL}"
            "$<TARGET_FILE_DIR:LaserCNC>"
        COMMENT "Copying runtime DLLs to LaserCNC output directory"
    )
    add_dependencies(LaserCNC lcnc_runtime_deploy_app)
endif()
# The canonical application output is shared by the Ninja and VS generator
# trees.  Keep zlib attached to the executable's own POST_BUILD action so a
# generator's custom-target scheduling cannot leave the smoke-test binary
# without this direct dependency.
add_custom_command(TARGET LaserCNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${LCNC_ZLIB_RUNTIME_DLL}"
        "$<TARGET_FILE_DIR:LaserCNC>"
    COMMENT "Copying zlib runtime DLL to LaserCNC output directory")
lcnc_deploy_tbb_runtime(LaserCNC)
if(LCNC_ENABLE_ASAN)
    lcnc_deploy_asan_runtime(LaserCNC)
endif()
if(LCNC_WITH_GTN)
    add_custom_command(TARGET LaserCNC POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LCNC_GTN_RUNTIME_DLL}"
            "$<TARGET_FILE_DIR:LaserCNC>"
        COMMENT "Copying GTN runtime DLL to output directory"
    )
endif()

add_executable(lcnc_project_upgrade
    src/tools/lcnc_project_upgrade.cpp
)
target_link_libraries(lcnc_project_upgrade PRIVATE lcnc_core Qt6::Core QuaZip::QuaZip)
set_target_properties(lcnc_project_upgrade PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${LCNC_RUNTIME_OUTPUT_DIRECTORY}"
)

if(BUILD_TESTING)
    find_program(LCNC_POWERSHELL_EXECUTABLE NAMES powershell pwsh REQUIRED)
    add_executable(lcnc_project_package_test
        tests/lcnc_project_package_test.cpp
    )
    target_link_libraries(lcnc_project_package_test PRIVATE lcnc_core Qt6::Core QuaZip::QuaZip)
    add_dependencies(lcnc_project_package_test lcnc_project_upgrade)
    target_compile_definitions(lcnc_project_package_test PRIVATE
        LCNC_PROJECT_UPGRADE_PATH="$<TARGET_FILE:lcnc_project_upgrade>"
    )
    add_executable(lcnc_task_manager_test
        tests/task_manager_test.cpp
    )
    target_link_libraries(lcnc_task_manager_test PRIVATE lcnc_core Qt6::Core Qt6::Concurrent)
    add_executable(lcnc_cam_lead_in_test
        tests/cam_lead_in_test.cpp
    )
    target_link_libraries(lcnc_cam_lead_in_test PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_cam_lead_in_diag
        tests/cam_lead_in_diag.cpp
    )
    target_link_libraries(lcnc_cam_lead_in_diag PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_contour_extraction_test
        tests/contour_extraction_test.cpp
    )
    target_link_libraries(lcnc_contour_extraction_test PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_cone_construction_diag
        tests/cone_construction_diag.cpp
    )
    target_link_libraries(lcnc_cone_construction_diag PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_process_runtime_configuration_test
        tests/process_runtime_configuration_test.cpp
    )
    target_link_libraries(lcnc_process_runtime_configuration_test PRIVATE Qt6::Core)
    if(TARGET lcnc_msvc_environment)
        target_link_libraries(lcnc_process_runtime_configuration_test PRIVATE lcnc_msvc_environment)
    endif()
    target_include_directories(lcnc_process_runtime_configuration_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/3rd/magic_enum"
    )
    add_executable(lcnc_device_command_queue_test
        tests/device_command_queue_test.cpp
    )
    target_link_libraries(lcnc_device_command_queue_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_preflight_service_test
        tests/process_preflight_service_test.cpp
    )
    target_link_libraries(lcnc_process_preflight_service_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_cutting_safety_test
        tests/process_cutting_safety_test.cpp
    )
    target_link_libraries(lcnc_process_cutting_safety_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_status_service_test
        tests/process_status_service_test.cpp
    )
    target_link_libraries(lcnc_process_status_service_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_connection_service_test
        tests/process_connection_service_test.cpp
    )
    target_link_libraries(lcnc_process_connection_service_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_manual_motion_service_test
        tests/process_manual_motion_service_test.cpp
    )
    target_link_libraries(lcnc_process_manual_motion_service_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_workflow_service_test
        tests/process_workflow_service_test.cpp
    )
    target_link_libraries(lcnc_process_workflow_service_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_loop_if_executor_test
        tests/process_loop_if_executor_test.cpp
    )
    target_link_libraries(lcnc_process_loop_if_executor_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_current_schema_test
        tests/process_current_schema_test.cpp
    )
    target_link_libraries(lcnc_process_current_schema_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_contour_order_planner_test
        tests/contour_order_planner_test.cpp
    )
    target_link_libraries(lcnc_contour_order_planner_test PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_cam_data_manager_test
        tests/cam_data_manager_test.cpp
    )
    target_link_libraries(lcnc_cam_data_manager_test PRIVATE lcnc_core Qt6::Core)
    add_executable(lcnc_machine_topology_solver_test
        tests/machine_topology_solver_test.cpp
    )
    target_link_libraries(lcnc_machine_topology_solver_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_toolpath_snapshot_concurrency_test
        tests/toolpath_snapshot_concurrency_test.cpp
    )
    target_link_libraries(lcnc_toolpath_snapshot_concurrency_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_process_run_coordinator_test
        tests/process_run_coordinator_test.cpp
    )
    target_link_libraries(lcnc_process_run_coordinator_test PRIVATE lcnc_module_process Qt6::Core)
    add_executable(lcnc_cad_algorithm_contract_test
        tests/cad_algorithm_contract_test.cpp
    )
    target_link_libraries(lcnc_cad_algorithm_contract_test PRIVATE lcnc_module_cad Qt6::Core)
    add_executable(lcnc_project_explorer_controller_test
        tests/project_explorer_controller_test.cpp
    )
    target_link_libraries(lcnc_project_explorer_controller_test PRIVATE lcnc_app Qt6::Widgets)
    add_executable(lcnc_toolpath_generation_service_test
        tests/toolpath_generation_service_test.cpp
    )
    target_link_libraries(lcnc_toolpath_generation_service_test PRIVATE lcnc_module_cam Qt6::Core)
    if(LCNC_WITH_ACS)
        add_executable(lcnc_simulator_cmhp_sdk_integration_test
            tests/simulator_cmhp_sdk_integration_test.cpp
        )
        target_link_libraries(lcnc_simulator_cmhp_sdk_integration_test PRIVATE
            lcnc_module_process Qt6::Core)
    endif()
    if(EXISTS "${LCNC_RUNTIME_BIN_DIR_DEBUG}" AND EXISTS "${LCNC_RUNTIME_BIN_DIR_RELEASE}")
        add_dependencies(lcnc_project_package_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_task_manager_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_cam_lead_in_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_cam_lead_in_diag lcnc_runtime_deploy_build)
        add_dependencies(lcnc_contour_extraction_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_cone_construction_diag lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_runtime_configuration_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_device_command_queue_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_preflight_service_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_cutting_safety_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_status_service_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_connection_service_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_manual_motion_service_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_workflow_service_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_loop_if_executor_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_current_schema_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_contour_order_planner_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_cam_data_manager_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_machine_topology_solver_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_toolpath_snapshot_concurrency_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_process_run_coordinator_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_cad_algorithm_contract_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_project_explorer_controller_test lcnc_runtime_deploy_build)
        add_dependencies(lcnc_toolpath_generation_service_test lcnc_runtime_deploy_build)
        if(LCNC_WITH_ACS)
            add_dependencies(lcnc_simulator_cmhp_sdk_integration_test lcnc_runtime_deploy_build)
        endif()
    endif()
    lcnc_deploy_tbb_runtime(lcnc_project_package_test)
    lcnc_deploy_tbb_runtime(lcnc_task_manager_test)
    lcnc_deploy_tbb_runtime(lcnc_cam_lead_in_test)
    lcnc_deploy_tbb_runtime(lcnc_cam_lead_in_diag)
    lcnc_deploy_tbb_runtime(lcnc_contour_extraction_test)
    lcnc_deploy_tbb_runtime(lcnc_cone_construction_diag)
    lcnc_deploy_tbb_runtime(lcnc_process_runtime_configuration_test)
    lcnc_deploy_tbb_runtime(lcnc_device_command_queue_test)
    lcnc_deploy_tbb_runtime(lcnc_process_preflight_service_test)
    lcnc_deploy_tbb_runtime(lcnc_process_cutting_safety_test)
    lcnc_deploy_tbb_runtime(lcnc_process_status_service_test)
    lcnc_deploy_tbb_runtime(lcnc_process_connection_service_test)
    lcnc_deploy_tbb_runtime(lcnc_process_manual_motion_service_test)
    lcnc_deploy_tbb_runtime(lcnc_process_workflow_service_test)
    lcnc_deploy_tbb_runtime(lcnc_process_loop_if_executor_test)
    lcnc_deploy_tbb_runtime(lcnc_process_current_schema_test)
    lcnc_deploy_tbb_runtime(lcnc_contour_order_planner_test)
    lcnc_deploy_tbb_runtime(lcnc_cam_data_manager_test)
    lcnc_deploy_tbb_runtime(lcnc_machine_topology_solver_test)
    lcnc_deploy_tbb_runtime(lcnc_toolpath_snapshot_concurrency_test)
    lcnc_deploy_tbb_runtime(lcnc_process_run_coordinator_test)
    lcnc_deploy_tbb_runtime(lcnc_cad_algorithm_contract_test)
    lcnc_deploy_tbb_runtime(lcnc_project_explorer_controller_test)
    lcnc_deploy_tbb_runtime(lcnc_toolpath_generation_service_test)
    if(LCNC_WITH_ACS)
        lcnc_deploy_tbb_runtime(lcnc_simulator_cmhp_sdk_integration_test)
        lcnc_deploy_acs_simulator_program(lcnc_simulator_cmhp_sdk_integration_test)
        lcnc_deploy_gtn_runtime(lcnc_simulator_cmhp_sdk_integration_test)
    endif()
    lcnc_deploy_qt_offscreen_platform(lcnc_project_explorer_controller_test)
    if(LCNC_ENABLE_ASAN)
        lcnc_deploy_asan_runtime(lcnc_project_package_test)
        lcnc_deploy_asan_runtime(lcnc_task_manager_test)
        lcnc_deploy_asan_runtime(lcnc_cam_lead_in_test)
        lcnc_deploy_asan_runtime(lcnc_contour_extraction_test)
        lcnc_deploy_asan_runtime(lcnc_process_runtime_configuration_test)
        lcnc_deploy_asan_runtime(lcnc_device_command_queue_test)
        lcnc_deploy_asan_runtime(lcnc_process_preflight_service_test)
        lcnc_deploy_asan_runtime(lcnc_process_cutting_safety_test)
        lcnc_deploy_asan_runtime(lcnc_process_status_service_test)
        lcnc_deploy_asan_runtime(lcnc_process_connection_service_test)
        lcnc_deploy_asan_runtime(lcnc_process_manual_motion_service_test)
        lcnc_deploy_asan_runtime(lcnc_process_workflow_service_test)
        lcnc_deploy_asan_runtime(lcnc_process_loop_if_executor_test)
        lcnc_deploy_asan_runtime(lcnc_process_current_schema_test)
        lcnc_deploy_asan_runtime(lcnc_contour_order_planner_test)
        lcnc_deploy_asan_runtime(lcnc_cam_data_manager_test)
        lcnc_deploy_asan_runtime(lcnc_machine_topology_solver_test)
        lcnc_deploy_asan_runtime(lcnc_toolpath_snapshot_concurrency_test)
        lcnc_deploy_asan_runtime(lcnc_process_run_coordinator_test)
        lcnc_deploy_asan_runtime(lcnc_cad_algorithm_contract_test)
        lcnc_deploy_asan_runtime(lcnc_project_explorer_controller_test)
        lcnc_deploy_asan_runtime(lcnc_toolpath_generation_service_test)
        if(LCNC_WITH_ACS)
            lcnc_deploy_asan_runtime(lcnc_simulator_cmhp_sdk_integration_test)
        endif()
    endif()
    add_test(NAME architecture_checks
        COMMAND "${LCNC_POWERSHELL_EXECUTABLE}" -NoProfile -ExecutionPolicy Bypass
                -File "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_architecture.ps1"
                -Root "${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME lcnc_project_package_test
        COMMAND lcnc_project_package_test)
    add_test(NAME lcnc_task_manager_test
        COMMAND lcnc_task_manager_test)
    add_test(NAME lcnc_cam_lead_in_test
        COMMAND lcnc_cam_lead_in_test)
    add_test(NAME lcnc_contour_extraction_test
        COMMAND lcnc_contour_extraction_test)
    add_test(NAME lcnc_cone_construction_test
        COMMAND lcnc_cone_construction_diag)
    add_test(NAME lcnc_process_runtime_configuration_test
        COMMAND lcnc_process_runtime_configuration_test)
    add_test(NAME lcnc_device_command_queue_test
        COMMAND lcnc_device_command_queue_test)
    add_test(NAME lcnc_process_preflight_service_test
        COMMAND lcnc_process_preflight_service_test)
    add_test(NAME lcnc_process_cutting_safety_test
        COMMAND lcnc_process_cutting_safety_test)
    add_test(NAME lcnc_process_status_service_test
        COMMAND lcnc_process_status_service_test)
    add_test(NAME lcnc_process_connection_service_test
        COMMAND lcnc_process_connection_service_test)
    add_test(NAME lcnc_process_manual_motion_service_test
        COMMAND lcnc_process_manual_motion_service_test)
    add_test(NAME lcnc_process_workflow_service_test
        COMMAND lcnc_process_workflow_service_test)
    add_test(NAME lcnc_process_loop_if_executor_test
        COMMAND lcnc_process_loop_if_executor_test)
    add_test(NAME lcnc_process_current_schema_test
        COMMAND lcnc_process_current_schema_test)
    add_test(NAME lcnc_contour_order_planner_test
        COMMAND lcnc_contour_order_planner_test)
    add_test(NAME lcnc_cam_data_manager_test
        COMMAND lcnc_cam_data_manager_test)
    add_test(NAME lcnc_machine_topology_solver_test
        COMMAND lcnc_machine_topology_solver_test)
    add_test(NAME lcnc_toolpath_snapshot_concurrency_test
        COMMAND lcnc_toolpath_snapshot_concurrency_test)
    add_test(NAME lcnc_process_run_coordinator_test
        COMMAND lcnc_process_run_coordinator_test)
    add_test(NAME lcnc_cad_algorithm_contract_test
        COMMAND lcnc_cad_algorithm_contract_test)
    add_test(NAME lcnc_project_explorer_controller_test
        COMMAND lcnc_project_explorer_controller_test)
    set_tests_properties(lcnc_project_explorer_controller_test PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    )
    add_test(NAME lcnc_toolpath_generation_service_test
        COMMAND lcnc_toolpath_generation_service_test)
    if(LCNC_WITH_ACS)
        add_test(NAME lcnc_simulator_cmhp_sdk_integration_test
            COMMAND lcnc_simulator_cmhp_sdk_integration_test)
        set_tests_properties(lcnc_simulator_cmhp_sdk_integration_test PROPERTIES
            TIMEOUT 600
            RUN_SERIAL TRUE
            RESOURCE_LOCK "simulator_cmhp_sdk"
        )
    endif()
    add_test(NAME lcnc_startup_smoke_test
        COMMAND LaserCNC --smoke-test)
    set_tests_properties(lcnc_startup_smoke_test PROPERTIES
        TIMEOUT 30
        RUN_SERIAL TRUE
    )
endif()

message(STATUS "Qt6 version: ${Qt6_VERSION}")
message(STATUS "OpenCASCADE version: ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")
message(STATUS "OpenCASCADE INCLUDE: ${OpenCASCADE_INCLUDE_DIR}")
