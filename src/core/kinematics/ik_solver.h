#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"

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
 * Table rotary orientation is solved here. The topology solver then derives
 * X/Y/Z from the relative motion between the configured tool-carrier and
 * workpiece-carrier branches. Head machines retain their calibrated TCP path.
 */
class IKSolver
{
public:
    /// Explicit table-tilt solve. Axis selection is supplied by semantic role;
    /// this API never infers a solver from missing axes or configuration text.
    static MachineCoord solveTableContinuous(const MachineKinematics* kinematics,
                                             const gp_Pnt& toolPos,
                                             const gp_Dir& toolDir,
                                             const QString& childSpinAxisName,
                                             const QString& parentTiltAxisName,
                                             const MachineCoord* previous = nullptr);

private:
    IKSolver() = delete;

    static MachineCoord solveTableType(const MachineKinematics* kin,
                                       const gp_Pnt& toolPos,
                                       const gp_Dir& toolDir,
                                       const QString& r1Name,
                                       const QString& r2Name,
                                       const MachineCoord* previous = nullptr);

    /// Solve for head-tilt configurations (AB_HEAD / AC_HEAD).
    /// The two rotary axes tilt the laser head; workpiece is fixed.
    static MachineCoord solveHeadType(const MachineKinematics* kin,
                                      const gp_Pnt& toolPos,
                                      const gp_Dir& toolDir,
                                      const QString& r1Name,
                                      const QString& r2Name);
};
