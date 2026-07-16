cmake_minimum_required(VERSION 3.20)
project(LaserCNC VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# AUTOMOC/AUTOUIC/AUTORCC are configured per-target below (not globally) so
# that each static library only scans its own sources. AUTOUIC in particular
# is enabled only on the process library, which is the sole owner of .ui files.

# ─── Qt ───────────────────────────────────────────────────────────────────────
set(LCNC_QT6_ROOT "E:/Qt6.9.1/6.9.1/msvc2022_64" CACHE PATH "Qt6 installation root")
set(Qt6_DIR "${LCNC_QT6_ROOT}/lib/cmake/Qt6" CACHE PATH "Qt6 CMake package directory")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent Network UiTools)

option(LCNC_WITH_SERIAL_COMM "Enable Qt SerialPort communication channel" ON)
if(LCNC_WITH_SERIAL_COMM)
    find_package(Qt6 QUIET COMPONENTS SerialPort)
    if(TARGET Qt6::SerialPort)
        set(LCNC_PROCESS_HAS_SERIAL_COMM ON)
    else()
        set(LCNC_PROCESS_HAS_SERIAL_COMM OFF)
        message(WARNING "LCNC_WITH_SERIAL_COMM=ON but Qt6 SerialPort was not found; serial communication will be unavailable")
    endif()
else()
    set(LCNC_PROCESS_HAS_SERIAL_COMM OFF)
endif()

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

# ─── Optional process device SDKs ─────────────────────────────────────────────
option(LCNC_WITH_ACS "Enable ACS motion controller adapter" OFF)
option(LCNC_WITH_GTN "Enable GTN motion controller adapter" OFF)
option(LCNC_WITH_BDAQ "Enable BDAQ IO adapter" OFF)
option(LCNC_WITH_REAL_LASER "Enable real laser adapters" OFF)

set(LCNC_PROCESS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rd" CACHE PATH "Process device SDK root")
set(LCNC_PROCESS_SDK_INCLUDE_DIR "${LCNC_PROCESS_SDK_ROOT}/include_3rd" CACHE PATH "Process device SDK include directory")
set(LCNC_PROCESS_SDK_LIB_DEBUG_DIR "${LCNC_PROCESS_SDK_ROOT}/lib_3rd/debug" CACHE PATH "Process device SDK debug library directory")
set(LCNC_PROCESS_SDK_LIB_RELEASE_DIR "${LCNC_PROCESS_SDK_ROOT}/lib_3rd/release" CACHE PATH "Process device SDK release library directory")

set(LCNC_ACS_INCLUDE_DIR "${LCNC_PROCESS_SDK_INCLUDE_DIR}" CACHE PATH "ACS SDK include directory")
set(LCNC_ACS_LIB "${LCNC_PROCESS_SDK_LIB_DEBUG_DIR}/ACSCL_x64.LIB" CACHE FILEPATH "ACS controller import library")
set(LCNC_GTN_INCLUDE_DIR "${LCNC_PROCESS_SDK_INCLUDE_DIR}" CACHE PATH "GTN SDK include directory")
set(LCNC_GTN_LIB "" CACHE FILEPATH "GTN controller import library")
set(LCNC_BDAQ_INCLUDE_DIR "${LCNC_PROCESS_SDK_INCLUDE_DIR}" CACHE PATH "BDAQ SDK include directory")
set(LCNC_BDAQ_LIB "" CACHE FILEPATH "BDAQ import library")
set(LCNC_REAL_LASER_SDK_ROOT "${LCNC_PROCESS_SDK_ROOT}" CACHE PATH "Real laser SDK root or protocol support root")

# ─── Boost (consumed by process module: ToolFactory, SerialPort, ACSMotionControl) ──
# Header includes: <boost/lexical_cast.hpp>, <boost/thread.hpp>, <boost/chrono.hpp>
# Compiled libs needed by SerialPort.cpp (boost::this_thread, boost::chrono): boost_thread, boost_chrono, boost_system
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
endif()
if(LCNC_WITH_GTN)
    lcnc_require_existing("${LCNC_GTN_INCLUDE_DIR}/gts.h" "LCNC_WITH_GTN requires gts.h")
    lcnc_require_existing("${LCNC_GTN_LIB}" "LCNC_WITH_GTN requires LCNC_GTN_LIB")
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
    LCNC_PROCESS_HAS_SERIAL_COMM=$<BOOL:${LCNC_PROCESS_HAS_SERIAL_COMM}>
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
    src/core/kinematics/machine_configuration_service.cpp
    src/core/kinematics/machine_pose.cpp
    src/core/kinematics/ik_solver.cpp
    src/core/algorithms/cad/primitives.cpp
    src/core/algorithms/cad/boolean_ops.cpp
    src/core/algorithms/cad/measure.cpp
    src/core/algorithms/cad/sketch.cpp
    src/core/algorithms/cad/sketch_constraints.cpp
    src/core/algorithms/cad/features.cpp
    src/core/algorithms/cad/transform_ops.cpp
    src/core/algorithms/cam/face_classifier.cpp
    src/core/algorithms/cam/laser_toolpath.cpp
    src/core/algorithms/cam/contour_order_planner.cpp
    src/core/task/task_progress.cpp
    src/core/task/task_manager.cpp
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
    src/view/machine_guide_renderer.cpp
    src/view/world_axes_renderer.cpp
    src/view/sketch_overlay_renderer.cpp
    src/view/transform_gizmo_renderer.cpp
)

set(SRCS_MODULE_CAD
    src/modules/cad/cad_module.cpp
    src/modules/cad/document/cad_document_state.cpp
    src/modules/cad/document/cad_document_registry.cpp
    src/modules/cad/services/cad_modeling_session.cpp
    src/modules/cad/services/sketch_manager.cpp
    src/modules/cad/services/shape_service.cpp
    src/modules/cad/selection/cad_selection_resolver.cpp
    src/modules/cad/sketch/sketch_serializer.cpp
    src/modules/cad/task/cad_command_dispatcher.cpp
    src/modules/cad/task/cad_tool_registry.cpp
    src/modules/cad/task/cad_tool_filter.cpp
    src/modules/cad/commands/commands_file.cpp
    src/modules/cad/commands/commands_edit.cpp
    src/modules/cad/commands/command_helpers.cpp
    src/modules/cad/commands/commands_cad.cpp
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
    src/modules/process/System/Service.cpp
    src/modules/process/System/LogModule.cpp
    src/modules/process/System/MessageModule.cpp
    src/modules/process/Setting/Settings.cpp
    src/modules/process/Setting/qg_dlgsetting.cpp
    src/modules/process/Setting/Setting_MotionControl.cpp
    src/modules/process/Setting/Setting_Axis.cpp
    src/modules/process/Setting/BuiltinIODefs.cpp
    src/modules/process/Setting/Setting_Digital.cpp
    src/modules/process/Setting/Setting_Analog.cpp
    src/modules/process/Setting/Setting_Laser.cpp
    src/modules/process/Setting/Setting_Internet.cpp
    src/modules/process/Setting/Setting_ProcessPlugins.cpp
    src/modules/process/Setting/Setting_Tool.cpp
    src/modules/process/Setting/Setting_Gas.cpp
    src/modules/process/Setting/Setting_Water.cpp
    src/modules/process/Setting/Setting_Monitor.cpp
    src/modules/process/Setting/Setting_LoadingPos.cpp
    src/modules/process/Setting/Setting_Camera.cpp
    src/modules/process/device/MotionControl/MotionControl.cpp
    src/modules/process/device/MotionControl/ACSMotionControl.cpp
    src/modules/process/device/MotionControl/SimulateCMHPMotionControl.cpp
    src/modules/process/device/Connect/SerialPort.cpp
    src/modules/process/device/Laser/LDFactory.cpp
    src/modules/process/device/Laser/AnalogLaserDevice.cpp
    src/modules/process/device/Laser/LaserDevice.cpp
    src/modules/process/device/Laser/SimulatorLaserDevice.cpp
    src/modules/process/Tool/Tool.cpp
    src/modules/process/Tool/ToolFactory.cpp
    src/modules/process/communication/communication_types.h
    src/modules/process/communication/communication_types.cpp
    src/modules/process/communication/communication_endpoint.h
    src/modules/process/communication/communication_endpoint.cpp
    src/modules/process/communication/communication_message.h
    src/modules/process/communication/communication_message.cpp
    src/modules/process/communication/i_communication_channel.h
    src/modules/process/communication/communication_channel_base.h
    src/modules/process/communication/communication_channel_base.cpp
    src/modules/process/communication/communication_log_model.h
    src/modules/process/communication/communication_log_model.cpp
    src/modules/process/communication/communication_manager.h
    src/modules/process/communication/communication_manager.cpp
    src/modules/process/communication/communication_device_base.h
    src/modules/process/communication/communication_device_base.cpp
    src/modules/process/communication/protocols/mock_channel.h
    src/modules/process/communication/protocols/mock_channel.cpp
    src/modules/process/communication/protocols/tcp_channel.h
    src/modules/process/communication/protocols/tcp_channel.cpp
    src/modules/process/communication/protocols/http_channel.h
    src/modules/process/communication/protocols/http_channel.cpp
    src/modules/process/communication/ui/communication_settings_page.h
    src/modules/process/communication/ui/communication_settings_page.cpp
    src/modules/process/monitor/process_monitor_types.h
    src/modules/process/monitor/process_monitor_service.h
    src/modules/process/monitor/process_monitor_service.cpp
    src/modules/process/runtime/process_runtime.cpp
    src/modules/process/runtime/process_state_machine.cpp
    src/modules/process/runtime/process_cancellation_token.h
    src/modules/process/runtime/process_interrupt_context.h
    src/modules/process/runtime/process_interrupt_context.cpp
    src/modules/process/runtime/machine_pose5.h
    src/modules/process/runtime/motion_params.h
    src/modules/process/runtime/axis_map.h
    src/modules/process/runtime/axis_map.cpp
    src/modules/process/runtime/i_motion_command_sink.h
    src/modules/process/runtime/acs_text_command_sink.h
    src/modules/process/runtime/acs_text_command_sink.cpp
    src/modules/process/runtime/gtn_buffered_command_sink.h
    src/modules/process/runtime/pure_simulation_sink.h
    src/modules/process/runtime/pure_simulation_sink.cpp
    src/modules/process/runtime/motion_sink_factory.cpp
    src/modules/process/toolpath/process_toolpath_service.cpp
    src/modules/process/toolpath/process_tool_matcher.cpp
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
    src/modules/process/steps/process_step_registry.cpp
    src/modules/process/steps/process_step_builtin_registration.cpp
    src/modules/process/steps/services/legacy_process_services.cpp
    src/modules/process/steps/start/start_step.cpp
    src/modules/process/steps/stop/stop_step.cpp
    src/modules/process/steps/single_axis_move/single_axis_move_step.cpp
    src/modules/process/steps/multi_axis_move/multi_axis_move_step.cpp
    src/modules/process/steps/output_signal/output_signal_step.cpp
    src/modules/process/steps/input_signal_wait/input_signal_wait_step.cpp
    src/modules/process/steps/normal_cutting/normal_cutting_step.cpp
    src/modules/process/ui/process_flow_model.cpp
    src/modules/process/ui/process_flow_tree_view.cpp
    src/modules/process/ui/process_node_edit_dialog.cpp
    src/modules/process/Process/qg_processeswidget.cpp
    src/modules/process/commands/commands_process.cpp
    src/modules/process/ui/widget_laser_control.cpp
    src/modules/process/ui/ribbon_process_tab.cpp
    src/modules/process/execution/process_workflow_executor.cpp
    src/modules/process/execution/process_node_executor_registry.cpp
)

if(LCNC_WITH_ACS)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/controllers/acs_motion_controller_adapter.h
        src/modules/process/controllers/acs_motion_controller_adapter.cpp
    )
endif()
if(LCNC_WITH_GTN)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/controllers/gtn_motion_controller_adapter.h
        src/modules/process/controllers/gtn_motion_controller_adapter.cpp
        src/modules/process/device/MotionControl/GTNMotionControl.cpp
        src/modules/process/runtime/gtn_buffered_command_sink.cpp
    )
endif()
if(TARGET Qt6::SerialPort)
    list(APPEND SRCS_MODULE_PROCESS
        src/modules/process/communication/protocols/serial_channel.h
        src/modules/process/communication/protocols/serial_channel.cpp
    )
endif()

set(SRCS_APP
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

# process module — only target with .ui files, hence the only one with AUTOUIC.
add_library(lcnc_module_process STATIC ${SRCS_MODULE_PROCESS})
set_target_properties(lcnc_module_process PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/Setting;${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/ui/legacy"
)
target_include_directories(lcnc_module_process PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/System
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/Setting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/Tool
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/MotionControl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/Laser
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/process/device/Connect
)
# Boost: ToolFactory.cpp, SerialPort.cpp, ACSMotionControl.cpp pull in
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
# ACSMotionControl.cpp pulls in the ACS SDK
# unconditionally (they are not gated by LCNC_WITH_ACS the way the optional
# adapter is), so link ACSCL_x64.LIB into the process library by default.
# LCNC_WITH_ACS still controls the additional acs_motion_controller_adapter.{h,cpp}.
if(EXISTS "${LCNC_ACS_LIB}")
    target_link_libraries(lcnc_module_process PRIVATE "${LCNC_ACS_LIB}")
endif()
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
if(TARGET Qt6::SerialPort)
    target_link_libraries(lcnc_module_process PUBLIC Qt6::SerialPort)
endif()

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
set_target_properties(LaserCNC PROPERTIES
    AUTOMOC ON
    AUTORCC ON
)
target_link_libraries(LaserCNC PRIVATE lcnc_app)

if(WIN32)
    set_target_properties(LaserCNC PROPERTIES
        WIN32_EXECUTABLE TRUE
        LINK_FLAGS "/ENTRY:mainCRTStartup"
    )
endif()

# ─── Deploy runtime DLLs ──────────────────────────────────────────────────────
# All runtime DLLs (OCC, Qt6 + plugins, SARibbon, boost, ACS, etc.) live as a
# self-contained snapshot in 3rd/runtime/bin_<debug|release>/. POST_BUILD copies
# the matching variant into the exe directory, so running LaserCNC.exe needs no
# external Qt / OCC install. To refresh this snapshot after updating an SDK,
# re-run scripts/refresh_runtime_dlls.* (or windeployqt manually) and commit
# the changed DLLs.
set(LCNC_RUNTIME_BIN_DIR_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}/3rd/runtime/bin_debug"   CACHE PATH "Runtime DLL directory (Debug)")
set(LCNC_RUNTIME_BIN_DIR_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/3rd/runtime/bin_release" CACHE PATH "Runtime DLL directory (Release)")

set(_lcnc_runtime_bin "$<IF:$<CONFIG:Debug>,${LCNC_RUNTIME_BIN_DIR_DEBUG},${LCNC_RUNTIME_BIN_DIR_RELEASE}>")
add_custom_command(TARGET LaserCNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${_lcnc_runtime_bin}"
        "$<TARGET_FILE_DIR:LaserCNC>"
    COMMENT "Copying runtime DLLs from 3rd/runtime/bin_<config> to output directory"
)

message(STATUS "Qt6 version: ${Qt6_VERSION}")
message(STATUS "OpenCASCADE version: ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")
message(STATUS "OpenCASCADE INCLUDE: ${OpenCASCADE_INCLUDE_DIR}")
