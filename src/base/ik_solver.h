#pragma once

#include "base/laser_toolpath.h"
#include "base/machine_kinematics.h"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

/**
 * @brief Generic 5-axis inverse kinematics solver.
 *
 * Supports all four machine configurations:
 *  - Table-tilt types  (VERTICAL_AC_TABLE, VERTICAL_BC_TABLE):
 *      Workpiece sits on rotary table (A→C or B→C).
 *      The laser head is on a linear XYZ gantry.
 *      IK: find rotary angles so that the workpiece normal aligns
 *      with the fixed tool axis (−Z in machine frame).
 *  - Head-tilt types   (AB_HEAD, AC_HEAD):
 *      Workpiece is fixed, laser head tilts via rotary axes (A→B or A→C)
 *      on top of the Z linear axis.
 *      IK: decompose the desired tool direction into two rotary angles.
 *
 * In all cases, the linear axes X/Y/Z absorb the residual translation.
 */
class IKSolver
{
public:
    /// Solve inverse kinematics for a single toolpath point.
    /// @param kinematics  Machine model (provides config type, axis defs, directions)
    /// @param toolPos     Desired tool-tip position in world frame (mm)
    /// @param toolDir     Desired tool direction in world frame (surface normal, pointing outward)
    /// @return MachineCoord with axis values; valid=false on failure.
    static MachineCoord solve(const MachineKinematics* kinematics,
                              const gp_Pnt& toolPos,
                              const gp_Dir& toolDir);

private:
    IKSolver() = delete;

    /// Solve for table-tilt configurations (AC_TABLE / BC_TABLE).
    /// The two rotary axes carry the workpiece; tool axis is fixed along −Z.
    static MachineCoord solveTableType(const MachineKinematics* kin,
                                       const gp_Pnt& toolPos,
                                       const gp_Dir& toolDir,
                                       const QString& r1Name,
                                       const QString& r2Name);

    /// Solve for head-tilt configurations (AB_HEAD / AC_HEAD).
    /// The two rotary axes tilt the laser head; workpiece is fixed.
    static MachineCoord solveHeadType(const MachineKinematics* kin,
                                      const gp_Pnt& toolPos,
                                      const gp_Dir& toolDir,
                                      const QString& r1Name,
                                      const QString& r2Name);
};
