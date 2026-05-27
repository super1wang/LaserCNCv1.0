cmake_minimum_required(VERSION 3.20)
project(LaserCNC VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ─── Qt ───────────────────────────────────────────────────────────────────────
set(LCNC_QT6_ROOT "E:/Qt6.9.1/6.9.1/msvc2022_64" CACHE PATH "Qt6 installation root")
set(Qt6_DIR "${LCNC_QT6_ROOT}/lib/cmake/Qt6" CACHE PATH "Qt6 CMake package directory")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent)

# ─── OpenCASCADE ──────────────────────────────────────────────────────────────
set(LCNC_OCCT_ROOT "G:/environment/occt7.9-vc144-64-combined/occt-vc144-64" CACHE PATH "OpenCASCADE installation root")
set(OpenCASCADE_DIR "${LCNC_OCCT_ROOT}/cmake" CACHE PATH "OpenCASCADE CMake package directory")
find_package(OpenCASCADE REQUIRED)

# ─── SARibbon ─────────────────────────────────────────────────────────────────
set(LCNC_SARIBBON_ROOT "F:/wangchao/SARibbon/bin_qt6.9.1_MSVC_x64" CACHE PATH "SARibbon installation root")
set(SARibbonBar_DIR "${LCNC_SARIBBON_ROOT}/lib/cmake/SARibbonBar" CACHE PATH "SARibbonBar CMake package directory")
find_package(SARibbonBar REQUIRED)

# ─── Optional process device SDKs ─────────────────────────────────────────────
option(LCNC_WITH_ACS "Enable ACS motion controller adapter" OFF)
option(LCNC_WITH_GTN "Enable GTN motion controller adapter" OFF)
option(LCNC_WITH_BDAQ "Enable BDAQ IO adapter" OFF)
option(LCNC_WITH_REAL_LASER "Enable real laser adapters" OFF)

set(LCNC_ACS_SDK_ROOT "" CACHE PATH "ACS SDK root")
set(LCNC_GTN_SDK_ROOT "" CACHE PATH "GTN SDK root")
set(LCNC_BDAQ_SDK_ROOT "" CACHE PATH "BDAQ SDK root")
set(LCNC_REAL_LASER_SDK_ROOT "" CACHE PATH "Real laser SDK root or protocol support root")

if(LCNC_WITH_ACS AND NOT EXISTS "${LCNC_ACS_SDK_ROOT}")
    message(FATAL_ERROR "LCNC_WITH_ACS=ON requires LCNC_ACS_SDK_ROOT")
endif()
if(LCNC_WITH_GTN AND NOT EXISTS "${LCNC_GTN_SDK_ROOT}")
    message(FATAL_ERROR "LCNC_WITH_GTN=ON requires LCNC_GTN_SDK_ROOT")
endif()
if(LCNC_WITH_BDAQ AND NOT EXISTS "${LCNC_BDAQ_SDK_ROOT}")
    message(FATAL_ERROR "LCNC_WITH_BDAQ=ON requires LCNC_BDAQ_SDK_ROOT")
endif()
if(LCNC_WITH_REAL_LASER AND NOT EXISTS "${LCNC_REAL_LASER_SDK_ROOT}")
    message(FATAL_ERROR "LCNC_WITH_REAL_LASER=ON requires LCNC_REAL_LASER_SDK_ROOT")
endif()

# ─── Sources ──────────────────────────────────────────────────────────────────
set(SRCS_CORE
    src/core/logging/logger.cpp
    src/core/settings/toml_config.cpp
    src/core/settings/app_settings.cpp
    src/core/project/lcnc_project_manifest.cpp
    src/core/project/lcnc_project_package.cpp
    src/core/project/lcnc_project_session.cpp
    src/core/project/lcnc_project_manager.cpp
    src/core/document/lcnc_document.cpp
    src/core/document/xcaf_utils.cpp
    src/core/kinematics/machine_kinematics.cpp
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
    src/core/algorithms/cam/machine_model_compressor.cpp
    src/core/task/task_progress.cpp
    src/core/task/task_manager.cpp
    src/core/command/commands_api.cpp
    src/core/kernel/service_registry.cpp
    src/core/kernel/event_bus.cpp
    src/core/kernel/module_registry.cpp
    src/core/kernel/kernel.cpp
)

set(SRCS_VIEW
    src/view/graphics_scene.cpp
    src/view/rendering_manager.cpp
    src/view/shape_object_driver.cpp
    src/view/gui_application.cpp
    src/view/gui_document.cpp
    src/view/widget_occ_view.cpp
    src/view/toolpath_renderer.cpp
    src/view/machine_guide_renderer.cpp
    src/view/world_axes_renderer.cpp
    src/view/sketch_overlay_renderer.cpp
    src/view/transform_gizmo_renderer.cpp
)

set(SRCS_MODULES
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
    src/modules/cam/cam_module.cpp
    src/modules/cam/settings/cam_config.cpp
    src/modules/cam/services/cam_data_manager.cpp
    src/modules/cam/services/toolpath_simulator.cpp
    src/modules/cam/services/machine_axis_detector.cpp
    src/modules/cam/services/machine_io.cpp
    src/modules/cam/services/reference_pick.cpp
    src/modules/cam/commands/commands_machine.cpp
    src/modules/cam/commands/commands_cam.cpp
    src/modules/cam/ui/widget_machine_panel.cpp
    src/modules/cam/ui/widget_toolpath_panel.cpp
    src/modules/cam/ui/dialog_mark_axes.cpp
    src/modules/cam/ui/dialog_axis_calibration_wizard.cpp
    src/modules/cam/ui/ribbon_cam_tab.cpp
    src/modules/process/process_module.cpp
    src/modules/process/controllers/simulation_motion_controller.cpp
    src/modules/process/device/process_device_manager.cpp
    src/modules/process/device/simulator_laser_device.cpp
    src/modules/process/device/simulator_process_io.cpp
    src/modules/process/settings/process_settings.cpp
    src/modules/process/workflow/process_node.cpp
    src/modules/process/workflow/process_node_registry.cpp
    src/modules/process/workflow/process_flow_document.cpp
    src/modules/process/workflow/process_flow_store.cpp
    src/modules/process/Setting/process_settings_dialog.cpp
    src/modules/process/ui/process_flow_model.cpp
    src/modules/process/ui/process_flow_tree_view.cpp
    src/modules/process/ui/process_node_edit_dialog.cpp
    src/modules/process/Process/qg_processeswidget.cpp
    src/modules/process/commands/commands_process.cpp
    src/modules/process/ui/widget_laser_control.cpp
    src/modules/process/ui/ribbon_process_tab.cpp
    src/modules/process/execution/process_workflow_executor.cpp
)

set(SRCS_APP
    src/app/app_context.cpp
    src/app/command_registry.cpp
    src/app/commands/commands_display.cpp
    src/app/dialog/dialog_options.cpp
    src/app/dialog/dialog_task_manager.cpp
    src/app/project_explorer_model.cpp
    src/app/project_explorer_tree_utils.cpp
    src/app/main_window.cpp
)

add_executable(LaserCNC WIN32
    src/main.cpp
    ${SRCS_CORE}
    ${SRCS_VIEW}
    ${SRCS_MODULES}
    ${SRCS_APP}
    resources/resources.qrc
)

target_include_directories(LaserCNC PRIVATE
    src
    3rd
    3rd/toml11
    ${OpenCASCADE_INCLUDE_DIR}
)

# OCC modules needed for Phase 1
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

# Add 3rd-party lib paths required by OCCT (openvr, etc.)
target_link_directories(LaserCNC PRIVATE
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

target_link_libraries(LaserCNC PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Concurrent
    SARibbonBar::SARibbonBar
    ${OCC_LIBS}
)

if(WIN32)
    target_compile_definitions(LaserCNC PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
    )
    # Set subsystem and entry point
    set_target_properties(LaserCNC PROPERTIES
        WIN32_EXECUTABLE TRUE
        LINK_FLAGS "/ENTRY:mainCRTStartup"
    )
endif()

# ─── Deploy OCC DLLs ──────────────────────────────────────────────────────────
set(OCC_BIN_DIR "${LCNC_OCCT_ROOT}/win64/vc14/bin" CACHE PATH "OpenCASCADE runtime DLL directory")
if(EXISTS "${OCC_BIN_DIR}")
    add_custom_command(TARGET LaserCNC POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OCC_BIN_DIR}"
            "$<TARGET_FILE_DIR:LaserCNC>"
        COMMENT "Copying OCC DLLs to output directory"
    )
else()
    message(WARNING "OCC_BIN_DIR does not exist: ${OCC_BIN_DIR}")
endif()

# Deploy SARibbon DLLs
set(SARIBBON_BIN_DIR "${LCNC_SARIBBON_ROOT}/bin" CACHE PATH "SARibbon runtime DLL directory")
if(EXISTS "${SARIBBON_BIN_DIR}")
    add_custom_command(TARGET LaserCNC POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SARIBBON_BIN_DIR}"
            "$<TARGET_FILE_DIR:LaserCNC>"
        COMMENT "Copying SARibbon DLLs to output directory"
    )
else()
    message(WARNING "SARIBBON_BIN_DIR does not exist: ${SARIBBON_BIN_DIR}")
endif()

message(STATUS "Qt6 version: ${Qt6_VERSION}")
message(STATUS "OpenCASCADE version: ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")
message(STATUS "OpenCASCADE INCLUDE: ${OpenCASCADE_INCLUDE_DIR}")
