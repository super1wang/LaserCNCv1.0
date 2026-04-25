#pragma once

/**
 * @file log_codes.h
 * @brief Centralized log/error code enumeration.
 *
 * Each event emitted through the LCNC_* logging macros carries a stable
 * numeric code. This makes it easy to grep logs, write release-note style
 * change tracking, and to drive optional UI dialogs from log severity.
 *
 * Codes are grouped by 1000s per subsystem so they remain readable.
 */
namespace lcnc {

enum class LogCode : int {
    // ── 0xxx Generic / framework ────────────────────────────────────────
    Generic                 = 0,
    LoggerInitialized       = 1,
    SettingsLoaded          = 2,
    SettingsSaveFailed      = 3,
    SettingsParseFailed     = 4,

    // ── 1xxx Document / XCAF ───────────────────────────────────────────
    DocumentOpenFailed      = 1001,
    DocumentSaveFailed      = 1002,
    DocumentImportFailed    = 1003,
    DocumentExportFailed    = 1004,

    // ── 2xxx CAD operations ────────────────────────────────────────────
    CadInvalidParameters    = 2001,
    CadOperationFailed      = 2002,

    // ── 3xxx CAM machine / kinematics ─────────────────────────────────
    MachineLoadFailed       = 3001,
    MachineExportFailed     = 3002,
    MachinePresetMissing    = 3003,
    MachineAxisInvalid      = 3004,
    MachineMountFailed      = 3005,

    // ── 4xxx CAM toolpath ──────────────────────────────────────────────
    ToolpathGenerationFailed = 4001,
    ToolpathInputInvalid     = 4002,
    ToolpathLeadInInvalid    = 4003,
    ToolpathSimulationFailed = 4004,

    // ── 5xxx Process / controller ──────────────────────────────────────
    ControllerConnectFailed  = 5001,
    ControllerJogFailed      = 5002,
    ControllerEmergencyStop  = 5003,

    // ── 6xxx Task scheduler ────────────────────────────────────────────
    TaskUnhandled            = 6001,
    TaskAborted              = 6002,

    // ── 9xxx Internal / unexpected ─────────────────────────────────────
    InternalUnexpectedState  = 9001,
    InternalAssertion        = 9002,
};

} // namespace lcnc
