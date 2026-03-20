cmake_minimum_required(VERSION 3.20)
project(LaserCNC VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ─── Qt ───────────────────────────────────────────────────────────────────────
set(Qt6_DIR "E:/Qt6.9.1/6.9.1/msvc2022_64/lib/cmake/Qt6")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent)

# ─── OpenCASCADE ──────────────────────────────────────────────────────────────
set(OpenCASCADE_DIR "G:/environment/occt7.9-vc144-64-combined/occt-vc144-64/cmake")
find_package(OpenCASCADE REQUIRED)

# ─── SARibbon ─────────────────────────────────────────────────────────────────
set(SARibbonBar_DIR "F:/wangchao/SARibbon/bin_qt6.9.1_MSVC_x64/lib/cmake/SARibbonBar")
find_package(SARibbonBar REQUIRED)

# ─── Sources ──────────────────────────────────────────────────────────────────
set(SRCS_BASE
    src/base/lcnc_application.cpp
    src/base/lcnc_document.cpp
    src/base/xcaf_utils.cpp
    src/base/task_progress.cpp
    src/base/task_manager.cpp
)

set(SRCS_GRAPHICS
    src/graphics/graphics_scene.cpp
    src/graphics/shape_object_driver.cpp
)

set(SRCS_GUI
    src/gui/gui_application.cpp
    src/gui/gui_document.cpp
)

set(SRCS_APP
    src/app/app_context.cpp
    src/app/commands_api.cpp
    src/app/commands_file.cpp
    src/app/commands_edit.cpp
    src/app/commands_display.cpp
    src/app/widget_occ_view.cpp
    src/app/widget_model_tree.cpp
    src/app/widget_machine_panel.cpp
    src/app/widget_laser_control.cpp
    src/app/dialog_task_manager.cpp
    src/app/mainwindow.cpp
)

add_executable(LaserCNC WIN32
    src/main.cpp
    ${SRCS_BASE}
    ${SRCS_GRAPHICS}
    ${SRCS_GUI}
    ${SRCS_APP}
    resources/resources.qrc
)

target_include_directories(LaserCNC PRIVATE
    src
    ${OpenCASCADE_INCLUDE_DIR}
)

# OCC modules needed for Phase 1
set(OCC_LIBS
    # Foundation
    TKernel TKMath
    # Modeling Data
    TKG2d TKG3d TKGeomBase TKBRep
    # Modeling Algorithms
    TKGeomAlgo TKTopAlgo TKPrim TKBO TKShHealing TKBool TKHLR TKMesh
    # Visualization
    TKService TKV3d TKOpenGl TKMeshVS
    # Application Framework (XDE/XCAF)
    TKCDF TKLCAF TKCAF TKBinL TKXmlL TKBin TKXml TKStdL TKStd TKVCAF
    # Data Exchange
    TKDE TKXSBase TKDESTEP TKXCAF TKDEIGES TKDESTL TKRWMesh TKDECascade TKBinXCAF TKXmlXCAF TKDEGLTF
)

# Add 3rd-party lib paths required by OCCT (openvr, etc.)
target_link_directories(LaserCNC PRIVATE
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/openvr-1.14.15-64/lib/win64"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/freeimage-3.18.0-x64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/freetype-2.13.3-x64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/gl2ps-1.3.8-vc14-64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/tbb-2021.13.0-x64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/draco-1.4.1-vc14-64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/tcltk-8.6.15-x64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/zlib-1.2.8-vc14-64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/lzma-5.2.2-vc14-64/lib"
    "G:/environment/occt7.9-vc144-64-combined/3rdparty-vc14-64/ffmpeg-3.3.4-64/lib"
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
set(OCC_BIN_DIR "G:/environment/occt7.9-vc144-64-combined/occt-vc144-64/win64/vc14/bin")
add_custom_command(TARGET LaserCNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${OCC_BIN_DIR}"
        "$<TARGET_FILE_DIR:LaserCNC>"
    COMMENT "Copying OCC DLLs to output directory"
)

# Deploy SARibbon DLLs
set(SARIBBON_BIN_DIR "F:/wangchao/SARibbon/bin_qt6.9.1_MSVC_x64/bin")
add_custom_command(TARGET LaserCNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${SARIBBON_BIN_DIR}"
        "$<TARGET_FILE_DIR:LaserCNC>"
    COMMENT "Copying SARibbon DLLs to output directory"
)

message(STATUS "Qt6 version: ${Qt6_VERSION}")
message(STATUS "OpenCASCADE version: ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")
message(STATUS "OpenCASCADE INCLUDE: ${OpenCASCADE_INCLUDE_DIR}")
