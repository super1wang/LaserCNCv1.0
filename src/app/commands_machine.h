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
 * @brief Bind a workpiece to a machine axis so it follows the axis's motion.
 *
 * Shows a small dialog: workpiece combo + axis combo.
 * Requires at least one Machine entity and one Workpiece entity.
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
