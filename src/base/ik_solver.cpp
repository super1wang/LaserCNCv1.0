#include "base/ik_solver.h"

#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// IKSolver — public entry point
// =============================================================================

MachineCoord IKSolver::solve(const MachineKinematics* kin,
                             const gp_Pnt& toolPos,
                             const gp_Dir& toolDir)
{
    MachineCoord result;
    if (!kin) return result;

    const QString cfg = kin->configType();

    // Identify the two rotary axes
    QString r1Name, r2Name;
    for (const auto& axis : kin->axes()) {
        if (axis.motionType == MachineAxisDef::Rotary) {
            if (r1Name.isEmpty())
                r1Name = axis.name;
            else if (r2Name.isEmpty())
                r2Name = axis.name;
        }
    }

    if (r1Name.isEmpty() || r2Name.isEmpty()) {
        // No two rotary axes found — return 3-axis solution
        result.x = toolPos.X();
        result.y = toolPos.Y();
        result.z = toolPos.Z();
        result.r1 = 0;   result.r2 = 0;
        result.r1Name = "A"; result.r2Name = "C";
        result.valid = true;
        return result;
    }

    if (cfg == "VERTICAL_AC_TABLE" || cfg == "VERTICAL_BC_TABLE")
        return solveTableType(kin, toolPos, toolDir, r1Name, r2Name);
    else if (cfg == "AB_HEAD" || cfg == "AC_HEAD")
        return solveHeadType(kin, toolPos, toolDir, r1Name, r2Name);

    // Fallback: head-type heuristic
    return solveHeadType(kin, toolPos, toolDir, r1Name, r2Name);
}

// =============================================================================
// Table-type IK (AC_TABLE / BC_TABLE)
//
// Machine geometry:
//   Tool is fixed along −Z (vertical laser pointing down).
//   Workpiece sits on a rotary table: parent-axis → child-axis.
//   The point must be rotated by the table so that:
//     1) The surface normal aligns with +Z (opposite to tool).
//     2) After rotation, the XYZ gantry can reach the point.
//
// Algorithm:
//   Given desired tool direction d (surface normal, pointing outward from part):
//   We need R(r1,r2) such that R(r1,r2) * d = (0, 0, 1)   [+Z = against tool]
//   where R = R_child(r2) * R_parent(r1).
//
//   For AC (A around X, C around Z):
//     R_A(a) * d = d'  →  C rotates around Z  →  d' must be (0,0,1).
//     So first solve A to bring d into the XZ or YZ plane,
//     then solve C for any remaining rotation around Z.
//
//   Actually for table-type: the table rotates the workpiece, so
//   the toolpath point and normal rotate WITH the table.
//   We need: after table rotation, the normal at the contact point
//   should align with −Z (tool axis).
//
//   Let n = the normal we want to align with +Z (the "upward" direction,
//   since tool comes from above along −Z).
//
//   R_total = R2(r2) * R1(r1)  (child after parent)
//   We need R_total * n = (0, 0, 1)
//   So R_total = rotation that maps n to Z+.
//
//   Decompose into two rotations about the known axes.
// =============================================================================

MachineCoord IKSolver::solveTableType(const MachineKinematics* kin,
                                      const gp_Pnt& toolPos,
                                      const gp_Dir& toolDir,
                                      const QString& r1Name,
                                      const QString& r2Name)
{
    MachineCoord result;
    result.r1Name = r1Name;
    result.r2Name = r2Name;

    const MachineAxisDef* ax1 = kin->findAxis(r1Name);
    const MachineAxisDef* ax2 = kin->findAxis(r2Name);
    if (!ax1 || !ax2) return result;

    const gp_Dir axis1Dir = ax1->direction;  // e.g. (1,0,0) for A
    const gp_Dir axis2Dir = ax2->direction;  // e.g. (0,0,1) for C

    // The tool direction is −Z in machine frame (laser points down).
    // We need the workpiece normal to face +Z after table rotation.
    // n = toolDir (surface normal pointing outward)
    // We need R2(r2) * R1(r1) * n = (0, 0, 1)
    //
    // Step 1: Find r1 to rotate n around axis1 so that the Z-component
    //         of the rotated n is maximized (i.e. projection onto +Z).
    //         After R1(r1), the result projected onto the plane perpendicular
    //         to axis2 should vanish (so axis2 rotation can finish alignment).

    gp_Vec n(toolDir.X(), toolDir.Y(), toolDir.Z());

    // Decompose n into:  n = n_parallel_to_axis1 + n_perp_to_axis1
    gp_Vec a1(axis1Dir);
    double n_dot_a1 = n.Dot(a1);
    gp_Vec n_para = a1 * n_dot_a1;
    gp_Vec n_perp = n - n_para;

    // Target: (0, 0, 1)
    gp_Vec target(0, 0, 1);

    // Rotation around axis1 by angle r1 rotates n_perp in the plane
    // perpendicular to axis1.  The parallel component is unchanged.
    // After R1, we get:   n' = n_para + R1(r1) * n_perp
    // Then R2(r2) * n' = target.
    //
    // Since R2 is around axis2, it can only change the component of n'
    // perpendicular to axis2.  The component along axis2 must already match.
    //
    // So: n'.Dot(axis2) = target.Dot(axis2)
    // n_para.Dot(axis2) + R1(r1)*n_perp . axis2 = target.Dot(axis2)
    //
    // Let's solve this numerically with atan2.

    double n_perp_mag = n_perp.Magnitude();
    if (n_perp_mag < 1e-10) {
        // Normal is parallel to axis1 — cannot tilt with r1
        // Just use r2 to align in the remaining plane
        result.r1 = 0;
    } else {
        // Project everything onto the plane perpendicular to axis1
        // to find the angle that brings n_perp to where it needs to be.
        //
        // After rotation by r1 around axis1:
        //   n' = n_para + cos(r1)*n_perp + sin(r1)*(a1 x n_perp)
        //
        // We need n'.Z = target.Z = 1 (or close to 1)
        // n_para.Z + cos(r1)*n_perp.Z + sin(r1)*(a1 x n_perp).Z = 1

        gp_Vec cross_a1_nperp = a1.Crossed(n_perp);

        // But actually we need: n'.Dot(axis2) = target.Dot(axis2)
        // for axis2 to handle the rest.
        gp_Vec a2(axis2Dir);
        double lhs_const = n_para.Dot(a2);
        double coeff_cos = n_perp.Dot(a2);
        double coeff_sin = cross_a1_nperp.Dot(a2);
        double rhs = target.Dot(a2);

        // lhs_const + coeff_cos * cos(r1) + coeff_sin * sin(r1) = rhs
        // A*cos(r1) + B*sin(r1) = C
        double A = coeff_cos;
        double B = coeff_sin;
        double C = rhs - lhs_const;
        double R = std::sqrt(A * A + B * B);

        if (R < 1e-10) {
            result.r1 = 0;
        } else {
            double sinVal = C / R;
            if (sinVal > 1.0) sinVal = 1.0;
            if (sinVal < -1.0) sinVal = -1.0;
            double baseAngle = std::asin(sinVal);
            double phaseAngle = std::atan2(B, A);
            result.r1 = (baseAngle - phaseAngle) * 180.0 / M_PI;
        }
    }

    // Clamp r1 to axis limits
    if (result.r1 > ax1->maxVal) result.r1 = ax1->maxVal;
    if (result.r1 < ax1->minVal) result.r1 = ax1->minVal;

    // Apply R1 rotation to n
    gp_Trsf rot1;
    rot1.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis1Dir), result.r1 * M_PI / 180.0);
    gp_Vec n_after_r1 = n;
    n_after_r1.Transform(rot1);

    // Step 2: Find r2 to rotate n_after_r1 around axis2 to align with target
    gp_Vec a2(axis2Dir);
    double n1_dot_a2 = n_after_r1.Dot(a2);
    gp_Vec n1_para = a2 * n1_dot_a2;
    gp_Vec n1_perp = n_after_r1 - n1_para;
    gp_Vec tgt_perp = target - a2 * target.Dot(a2);

    double n1_perp_mag = n1_perp.Magnitude();
    double tgt_perp_mag = tgt_perp.Magnitude();

    if (n1_perp_mag < 1e-10 || tgt_perp_mag < 1e-10) {
        result.r2 = 0;
    } else {
        n1_perp.Normalize();
        tgt_perp.Normalize();
        double dotVal = n1_perp.Dot(tgt_perp);
        if (dotVal >  1.0) dotVal =  1.0;
        if (dotVal < -1.0) dotVal = -1.0;
        double angle = std::acos(dotVal);
        // Determine sign
        gp_Vec crossVal = n1_perp.Crossed(tgt_perp);
        if (crossVal.Dot(a2) < 0) angle = -angle;
        result.r2 = angle * 180.0 / M_PI;
    }

    // Clamp r2 to axis limits
    if (result.r2 > ax2->maxVal) result.r2 = ax2->maxVal;
    if (result.r2 < ax2->minVal) result.r2 = ax2->minVal;

    // Step 3: Compute the actual rotation applied to the workpiece
    gp_Trsf rot2;
    rot2.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis2Dir), result.r2 * M_PI / 180.0);
    gp_Trsf totalRot = rot2.Multiplied(rot1);

    // The tool tip position in world after table rotation:
    // The workpiece point P rotates with the table → P_world = totalRot * P
    // The gantry must go to P_world, so XYZ = P_world.
    gp_Pnt rotatedPos = toolPos.Transformed(totalRot);
    result.x = rotatedPos.X();
    result.y = rotatedPos.Y();
    result.z = rotatedPos.Z();
    result.valid = true;

    return result;
}

// =============================================================================
// Head-type IK (AB_HEAD / AC_HEAD)
//
// Machine geometry:
//   Workpiece is fixed.
//   Laser head tilts via two rotary axes mounted on the Z/XYZ gantry.
//   The default tool direction is −Z; rotary axes tilt it to match the
//   desired normal direction.
//
// Algorithm:
//   The tool direction after rotation:
//     d_tool = R_child(r2) * R_parent(r1) * (0, 0, -1)
//   We need d_tool = -toolDir  (tool points opposite to surface normal)
//
//   Decompose desired direction into the two rotary axis angles.
// =============================================================================

MachineCoord IKSolver::solveHeadType(const MachineKinematics* kin,
                                     const gp_Pnt& toolPos,
                                     const gp_Dir& toolDir,
                                     const QString& r1Name,
                                     const QString& r2Name)
{
    MachineCoord result;
    result.r1Name = r1Name;
    result.r2Name = r2Name;

    const MachineAxisDef* ax1 = kin->findAxis(r1Name);
    const MachineAxisDef* ax2 = kin->findAxis(r2Name);
    if (!ax1 || !ax2) return result;

    const gp_Dir axis1Dir = ax1->direction;  // e.g. (1,0,0) for A
    const gp_Dir axis2Dir = ax2->direction;  // e.g. (0,1,0) for B or (0,0,1) for C

    // The default tool direction is (0, 0, -1) (laser points down).
    // After rotating by R = R2 * R1 applied to (0,0,-1), we want to get −toolDir.
    // So: R2(r2) * R1(r1) * (0,0,-1) = −toolDir
    //     R2(r2) * R1(r1) * (0,0,-1) = (−nx, −ny, −nz)  where (nx,ny,nz) = toolDir
    //
    // Equivalently: R1(r1)^-1 * R2(r2)^-1 * (−toolDir) = (0,0,−1)
    // Or:           R2(r2) * R1(r1) * Z_neg = desired
    //
    // Let d = −toolDir = desired tool vector (the direction laser should point)
    gp_Vec desired(-toolDir.X(), -toolDir.Y(), -toolDir.Z());

    // Z_neg = (0,0,-1) is the default tool vector.
    // We need: R2(r2) * (R1(r1) * Z_neg) = desired
    //
    // Strategy: Find r1 first such that after R1, the intermediate vector
    // lies in a plane that R2 can reach.
    //
    // R1 rotates around axis1.  R2 rotates around axis2.
    // After R2, the component along axis2 is preserved.
    // So: (R1(r1) * Z_neg).dot(axis2) = desired.dot(axis2)

    gp_Vec z_neg(0, 0, -1);  // default tool dir
    gp_Vec a1(axis1Dir);
    gp_Vec a2(axis2Dir);

    // Decompose z_neg relative to axis1:
    double zn_dot_a1 = z_neg.Dot(a1);
    gp_Vec zn_para = a1 * zn_dot_a1;
    gp_Vec zn_perp = z_neg - zn_para;

    double zn_perp_mag = zn_perp.Magnitude();

    if (zn_perp_mag < 1e-10) {
        // Tool axis is parallel to axis1 — r1 has no effect
        result.r1 = 0;
    } else {
        // After R1(r1):  v = zn_para + cos(r1)*zn_perp + sin(r1)*(a1 x zn_perp)
        // Constraint: v.dot(a2) = desired.dot(a2)
        gp_Vec cross_a1_zn = a1.Crossed(zn_perp);

        double lhs_const = zn_para.Dot(a2);
        double coeff_cos = zn_perp.Dot(a2);
        double coeff_sin = cross_a1_zn.Dot(a2);
        double rhs = desired.Dot(a2);

        double A = coeff_cos;
        double B = coeff_sin;
        double C = rhs - lhs_const;
        double R = std::sqrt(A * A + B * B);

        if (R < 1e-10) {
            result.r1 = 0;
        } else {
            double sinVal = C / R;
            if (sinVal > 1.0) sinVal = 1.0;
            if (sinVal < -1.0) sinVal = -1.0;
            double baseAngle = std::asin(sinVal);
            double phaseAngle = std::atan2(B, A);
            // Two solutions: pick the one with smaller absolute value
            double r1a = (baseAngle - phaseAngle) * 180.0 / M_PI;
            double r1b = (M_PI - baseAngle - phaseAngle) * 180.0 / M_PI;
            // Normalize to [-180, 180]
            auto normalize = [](double deg) {
                while (deg > 180.0) deg -= 360.0;
                while (deg < -180.0) deg += 360.0;
                return deg;
            };
            r1a = normalize(r1a);
            r1b = normalize(r1b);
            // Pick solution within limits with smaller absolute value
            bool a_ok = (r1a >= ax1->minVal && r1a <= ax1->maxVal);
            bool b_ok = (r1b >= ax1->minVal && r1b <= ax1->maxVal);
            if (a_ok && b_ok)
                result.r1 = (std::abs(r1a) <= std::abs(r1b)) ? r1a : r1b;
            else if (a_ok)
                result.r1 = r1a;
            else if (b_ok)
                result.r1 = r1b;
            else
                result.r1 = (std::abs(r1a) <= std::abs(r1b)) ? r1a : r1b;
        }
    }

    // Clamp r1
    if (result.r1 > ax1->maxVal) result.r1 = ax1->maxVal;
    if (result.r1 < ax1->minVal) result.r1 = ax1->minVal;

    // Apply R1 to z_neg
    gp_Trsf rot1;
    rot1.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis1Dir), result.r1 * M_PI / 180.0);
    gp_Vec v_after_r1 = z_neg;
    v_after_r1.Transform(rot1);

    // Step 2: Find r2 so that R2(r2) * v_after_r1 = desired
    double v1_dot_a2 = v_after_r1.Dot(a2);
    gp_Vec v1_perp = v_after_r1 - a2 * v1_dot_a2;
    gp_Vec d_perp  = desired     - a2 * desired.Dot(a2);

    double v1_perp_mag = v1_perp.Magnitude();
    double d_perp_mag  = d_perp.Magnitude();

    if (v1_perp_mag < 1e-10 || d_perp_mag < 1e-10) {
        result.r2 = 0;
    } else {
        v1_perp.Normalize();
        d_perp.Normalize();
        double dotVal = v1_perp.Dot(d_perp);
        if (dotVal >  1.0) dotVal =  1.0;
        if (dotVal < -1.0) dotVal = -1.0;
        double angle = std::acos(dotVal);
        gp_Vec crossVal = v1_perp.Crossed(d_perp);
        if (crossVal.Dot(a2) < 0) angle = -angle;
        result.r2 = angle * 180.0 / M_PI;
    }

    // Clamp r2
    if (result.r2 > ax2->maxVal) result.r2 = ax2->maxVal;
    if (result.r2 < ax2->minVal) result.r2 = ax2->minVal;

    // Step 3: Linear axes = tool position (workpiece is fixed in head-type)
    result.x = toolPos.X();
    result.y = toolPos.Y();
    result.z = toolPos.Z();
    result.valid = true;

    return result;
}
