#pragma once

#include "app/commands_api.h"

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
 * @brief Mount a workpiece document onto a machine axis.
 *
 * Shows a dialog where the user picks a workpiece document (from all open
 * workpiece docs) and a target axis.  All shapes are merged into one compound,
 * added to the machine document, and mounted to the chosen axis.
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
 * Asks for confirmation, then clears the machine workspace document.
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


