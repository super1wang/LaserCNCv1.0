#pragma once

#include "core/command/commands_api.h"

/**
 * @brief Load a STEP/STL file as an EntityKind::Machine entity.
 *
 * After import the command:
 *  - Loads a kinematic preset chosen by the user.
 *  - Runs auto-detection of axis assignments by shape name.
 *  - Triggers tree and panel refresh via documentModified.
 */
class CmdLoadMachine : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdLoadMachine(IAppContext* ctx);
    static constexpr const char* Name = "machine.load";

    bool isEnabled() const override { return true; }
    void execute()   override;
};

class CmdBuildMachineSafetyPackage : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdBuildMachineSafetyPackage(IAppContext* ctx);
    static constexpr const char* Name = "machine.build_safety_package";

    bool isEnabled() const override;
    void execute() override;
};

/**
 * @brief Open DialogMarkAxes so the user can review / reassign shapes to axes.
 *
 * Requires at least one Machine entity to exist in the active document.
 */
class CmdMarkAxes : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdMarkAxes(IAppContext* ctx);
    static constexpr const char* Name = "machine.mark_axes";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Mount the current project workpiece onto a machine axis.
 *
 * Shows a dialog where the user confirms the current workpiece and target axis.
 * Existing Workpiece labels remain in the project document; the command only
 * writes axis mount relations and moves geometry to the configured install point.
 */
class CmdMountWorkpiece : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdMountWorkpiece(IAppContext* ctx);
    static constexpr const char* Name = "machine.mount_workpiece";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Remove all machine entities and reset the kinematic configuration.
 *
 * Asks for confirmation, then clears the machine model geometry.
 */
class CmdUnloadMachine : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdUnloadMachine(IAppContext* ctx);
    static constexpr const char* Name = "machine.unload";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Export the loaded machine model as a STEP file with LCNC_AXIS_*
 *        named compounds for each axis group.
 *
 * On re-import, CmdLoadMachine's autoDetect() recognises the LCNC_AXIS_
 * prefix and restores axis assignments automatically.
 */
class CmdExportMachine : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdExportMachine(IAppContext* ctx);
    static constexpr const char* Name = "machine.export";

    bool isEnabled() const override;
    void execute()   override;
};


