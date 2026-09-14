# Source contract only: this does not prove electrical output state on hardware.
file(READ "${CMAKE_CURRENT_LIST_DIR}/../src/modules/process/device/motion_control/gtn_motion_control.cpp" source)
string(REGEX REPLACE "//[^\n]*" "" source "${source}")
if(source MATCHES "GTN_[A-Za-z0-9_]*Laser[A-Za-z0-9_]*[ \t\r\n]*\\(")
    message(FATAL_ERROR "GTN adapter must use configured digital outputs, not native laser APIs")
endif()
foreach(required IN ITEMS
    "if (!stopConfiguredOutputs(\"InitFiveAxisGroupOutputsOff\"))"
    "DigitalOUT::Laser, laserOn, \"GroupLaserControl\")")
    string(FIND "${source}" "${required}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Missing digital laser safety contract: ${required}")
    endif()
endforeach()
foreach(obsolete IN ITEMS "ConfigurationDerivedTrial" "trialLinearVelocityMax"
    "trialRotaryVelocityMax" "GroupLaserControlTrialOff"
    "fConfigurationDerivedLinearVelocityMax" "fConfigurationDerivedRotaryVelocityMax")
    string(FIND "${source}" "${obsolete}" found)
    if(NOT found EQUAL -1)
        message(FATAL_ERROR "RTCP trial restriction remains: ${obsolete}")
    endif()
endforeach()
foreach(required IN ITEMS
    "move.velocity = std::max(tool.m_dLineVelocity, 0.001)"
    "move.acceleration = std::max(tool.m_dLineAcc, 0.001)"
    "axisConstraint.velMax = std::max(configured->motionSpeed, 0.001)"
    "axisConstraint.accMax = std::max(configured->acceleration, 0.001)"
    "predictedAxes, actualAxes, tolerance, &maximumError")
    string(FIND "${source}" "${required}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Missing normal RTCP contract: ${required}")
    endif()
endforeach()
file(READ "${CMAKE_CURRENT_LIST_DIR}/../src/modules/process/runtime/gtn_buffered_command_sink.cpp" sink)
if(sink MATCHES "bAOUTFlag=\\*/true")
    message(FATAL_ERROR "Buffered sink must not request native laser mode")
endif()
message(STATUS "GTN digital laser source contract passed")
