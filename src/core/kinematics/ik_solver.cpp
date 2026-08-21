#include "core/kinematics/ik_solver.h"
#include "core/math/numeric_constants.h"

#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <algorithm>
#include <cmath>
#include <utility>


namespace {

gp_Trsf axisRotation(const MachineAxisDef& axis, double angleDeg)
{
    gp_Trsf trsf;
    trsf.SetRotation(gp_Ax1(axis.origin, axis.direction),
                     lcnc::math::degreesToRadians(angleDeg));
    return trsf;
}

bool isCAxisName(const QString& axisName)
{
    return axisName.trimmed().toUpper() == QStringLiteral("C");
}

double normalizeSigned180(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return std::abs(deg) < 1e-10 ? 0.0 : deg;
}

double normalizeAxisOutput(const QString& axisName, double deg)
{
    return isCAxisName(axisName)
        ? deg
        : normalizeSigned180(deg);
}

double equivalentAngleNear(double value, double ref)
{
    while (value - ref > 180.0) value -= 360.0;
    while (value - ref < -180.0) value += 360.0;
    return value;
}

struct RotaryBranchCost
{
    bool acceptable{false};
    double err{0.0};
    double tiltDelta{0.0};
    double cDelta{0.0};
};

double axisComparableValue(const QString& axisName,
                           double value,
                           double reference,
                           bool hasReference)
{
    const double comparable = hasReference
        ? equivalentAngleNear(value, reference)
        : normalizeAxisOutput(axisName, value);
    return isCAxisName(axisName) ? comparable : normalizeSigned180(comparable);
}

RotaryBranchCost rotaryBranchCost(const QString& r1Name,
                                  double r1Value,
                                  const QString& r2Name,
                                  double r2Value,
                                  double err,
                                  const MachineCoord* previous)
{
    RotaryBranchCost cost;
    cost.acceptable = err <= 1e-5;
    cost.err = err;

    auto addDelta = [&](const QString& axisName,
                        double value,
                        double reference,
                        bool hasReference) {
        const double comparable = axisComparableValue(axisName, value, reference, hasReference);
        const double delta = hasReference
            ? std::abs(comparable - reference)
            : std::abs(comparable);
        if (isCAxisName(axisName))
            cost.cDelta += delta;
        else
            cost.tiltDelta += delta;
    };

    const bool hasPrevious = previous && previous->valid
        && previous->r1Name == r1Name
        && previous->r2Name == r2Name;
    addDelta(r1Name, r1Value, hasPrevious ? previous->r1 : 0.0, hasPrevious);
    addDelta(r2Name, r2Value, hasPrevious ? previous->r2 : 0.0, hasPrevious);
    return cost;
}

bool rotaryBranchBetter(const RotaryBranchCost& a, const RotaryBranchCost& b)
{
    constexpr double kErrEps = 1e-7;
    constexpr double kAxisEps = 1e-6;
    if (a.acceptable != b.acceptable)
        return a.acceptable;
    if (!a.acceptable && std::abs(a.err - b.err) > kErrEps)
        return a.err < b.err;
    if (std::abs(a.tiltDelta - b.tiltDelta) > kAxisEps)
        return a.tiltDelta < b.tiltDelta;
    if (std::abs(a.cDelta - b.cDelta) > kAxisEps)
        return a.cDelta < b.cDelta;
    return a.err < b.err;
}

bool withinAxisLimits(const MachineAxisDef& axis, double value)
{
    constexpr double kLimitEps = 1e-6;
    return value >= axis.minVal - kLimitEps && value <= axis.maxVal + kLimitEps;
}

double linearAxisCoordinate(const MachineKinematics* kin,
                            const QString& axisName,
                            const gp_Pnt& worldPoint,
                            double fallback)
{
    const MachineAxisDef* axis = kin ? kin->findAxis(axisName) : nullptr;
    if (!axis || axis->motionType != MachineAxisDef::Linear)
        return fallback;

    // MachineKinematics applies a linear axis as direction * coordinate. Its
    // inverse is therefore the scalar projection of the world point onto the
    // configured positive axis direction. In particular, Z=(0,0,-1) maps a
    // negative world-space Z cutting point to a positive machine Z value.
    return gp_Vec(gp_Pnt(0.0, 0.0, 0.0), worldPoint).Dot(gp_Vec(axis->direction));
}

void setLinearMachineCoordinates(MachineCoord& result,
                                 const MachineKinematics* kin,
                                 const gp_Pnt& worldPoint)
{
    result.x = linearAxisCoordinate(kin, QStringLiteral("X"), worldPoint, worldPoint.X());
    result.y = linearAxisCoordinate(kin, QStringLiteral("Y"), worldPoint, worldPoint.Y());
    result.z = linearAxisCoordinate(kin, QStringLiteral("Z"), worldPoint, worldPoint.Z());
}

double tableAlignmentError(const MachineAxisDef& ax1,
                           double r1Value,
                           const MachineAxisDef& ax2,
                           double r2Value,
                           const gp_Vec& normal,
                           const gp_Vec& target)
{
    gp_Trsf rot1 = axisRotation(ax1, r1Value);
    gp_Trsf rot2 = axisRotation(ax2, r2Value);
    gp_Trsf total = rot2.Multiplied(rot1);
    gp_Vec finalNormal = normal;
    finalNormal.Transform(total);
    return (finalNormal - target).Magnitude();
}

bool solveRotationAboutAxis(const MachineAxisDef& axis,
                            const gp_Vec& from,
                            const gp_Vec& to,
                            double& angleDeg)
{
    const gp_Vec axisVec(axis.direction);
    gp_Vec fromPerp = from - axisVec * from.Dot(axisVec);
    gp_Vec toPerp = to - axisVec * to.Dot(axisVec);
    if (fromPerp.Magnitude() <= 1e-10 || toPerp.Magnitude() <= 1e-10)
        return false;

    fromPerp.Normalize();
    toPerp.Normalize();
    double dotV = fromPerp.Dot(toPerp);
    if (dotV > 1.0) dotV = 1.0;
    if (dotV < -1.0) dotV = -1.0;

    double angle = std::acos(dotV);
    if (fromPerp.Crossed(toPerp).Dot(axisVec) < 0.0)
        angle = -angle;
    angleDeg = normalizeSigned180(lcnc::math::radiansToDegrees(angle));
    return true;
}

} // namespace

// =============================================================================
// IKSolver — public entry point
// =============================================================================

MachineCoord IKSolver::solveTableContinuous(const MachineKinematics* kin,
                                             const gp_Pnt& toolPos,
                                             const gp_Dir& toolDir,
                                             const QString& childSpinAxisName,
                                             const QString& parentTiltAxisName,
                                             const MachineCoord* previous)
{
    MachineCoord result;
    if (!kin || childSpinAxisName.isEmpty() || parentTiltAxisName.isEmpty()) return result;
    return solveTableType(kin, toolPos, toolDir,
                          childSpinAxisName, parentTiltAxisName, previous);
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
                                      const QString& r2Name,
                                      const MachineCoord* previous)
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

    // 工具函数：给定 r1（度），返回最优 r2（度）及对齐误差。
    auto evalBranch = [&](double r1deg) {
        gp_Trsf rotTry = axisRotation(*ax1, r1deg);
        gp_Vec n_try = n;
        n_try.Transform(rotTry);

        gp_Vec a2v(axis2Dir);
        gp_Vec t_para = a2v * target.Dot(a2v);
        gp_Vec t_perp = target - t_para;
        gp_Vec n2_para = a2v * n_try.Dot(a2v);
        gp_Vec n2_perp = n_try - n2_para;

        double r2deg = 0.0;
        if (n2_perp.Magnitude() > 1e-10 && t_perp.Magnitude() > 1e-10) {
            gp_Vec n2n = n2_perp; n2n.Normalize();
            gp_Vec ttn = t_perp;  ttn.Normalize();
            double dotV = n2n.Dot(ttn);
            if (dotV >  1.0) dotV =  1.0;
            if (dotV < -1.0) dotV = -1.0;
            double ang = std::acos(dotV);
            gp_Vec cv = n2n.Crossed(ttn);
            if (cv.Dot(a2v) < 0) ang = -ang;
            r2deg = lcnc::math::radiansToDegrees(ang);
        }

        // 真实对齐误差：施加 r2 后看 n_final 与 target 的夹角。
        gp_Trsf rot2Try = axisRotation(*ax2, r2deg);
        gp_Vec n_final = n_try;
        n_final.Transform(rot2Try);
        double err = (n_final - target).Magnitude();
        return std::make_pair(r2deg, err);
    };

    if (n_perp_mag < 1e-10) {
        // Normal is parallel to axis1 — r1 has no effect; let r2 finish alignment.
        result.r1 = 0;
        auto [r2v, err] = evalBranch(0.0);
        result.r2 = r2v;
    } else {
        // n_para.Dot(axis2) + cos(r1)*coeff_cos + sin(r1)*coeff_sin = target.Dot(axis2)
        // 等价形式：A*cos(r1) + B*sin(r1) = C，即 R*sin(r1 + φ) = C 其中 φ = atan2(A,B)。
        // ⇒ sin(r1 + φ) = C/R，有两个解：r1 + φ = asin(C/R) 或 π − asin(C/R)。
        // 两支都对应一个合法的法线对齐方向；选择使后续 |r2| 最小、对齐误差最小的那一支。

        gp_Vec cross_a1_nperp = a1.Crossed(n_perp);
        gp_Vec a2v(axis2Dir);
        double Av = n_perp.Dot(a2v);                       // 系数 A
        double Bv = cross_a1_nperp.Dot(a2v);               // 系数 B
        double Cv = target.Dot(a2v) - n_para.Dot(a2v);     // 常数 C
        double Rv = std::sqrt(Av * Av + Bv * Bv);

        auto computeAndPick = [&](double r1cand1, double r1cand2) {
            // 归一化到 [-180, 180]
            auto norm180 = [](double d) {
                while (d >  180.0) d -= 360.0;
                while (d < -180.0) d += 360.0;
                return d;
            };
            r1cand1 = norm180(r1cand1);
            r1cand2 = norm180(r1cand2);

            // 截断到 r1 轴限位（在选解前）
            auto clampR1 = [&](double v) {
                if (v > ax1->maxVal) return ax1->maxVal;
                if (v < ax1->minVal) return ax1->minVal;
                return v;
            };
            double a1deg_a = clampR1(r1cand1);
            double a1deg_b = clampR1(r1cand2);

            auto [r2_a, err_a] = evalBranch(a1deg_a);
            auto [r2_b, err_b] = evalBranch(a1deg_b);

            // 截断 r2 限位
            auto clampR2 = [&](double v) {
                if (v > ax2->maxVal) return ax2->maxVal;
                if (v < ax2->minVal) return ax2->minVal;
                return v;
            };
            r2_a = clampR2(r2_a);
            r2_b = clampR2(r2_b);

            const RotaryBranchCost costA =
                rotaryBranchCost(r1Name, a1deg_a, r2Name, r2_a, err_a, previous);
            const RotaryBranchCost costB =
                rotaryBranchCost(r1Name, a1deg_b, r2Name, r2_b, err_b, previous);
            const bool pickB = rotaryBranchBetter(costB, costA);
            if (pickB) { result.r1 = a1deg_b; result.r2 = r2_b; }
            else       { result.r1 = a1deg_a; result.r2 = r2_a; }
        };

        if (Rv < 1e-10) {
            // 法线在 axis2 上的投影与 axis1 的旋转面无相关分量；r1 任意都能让 r2 完成对齐。
            // 选 r1=0，给 r2 处理。
            result.r1 = 0;
            auto [r2v, err] = evalBranch(0.0);
            result.r2 = std::clamp(r2v, ax2->minVal, ax2->maxVal);
        } else {
            double sinVal = Cv / Rv;
            if (sinVal > 1.0)  sinVal = 1.0;
            if (sinVal < -1.0) sinVal = -1.0;
            // A·cos(r1) + B·sin(r1) = C ⇔ R·sin(r1+φ) = C，匹配系数得
            //   R·sin φ = A、R·cos φ = B   →   φ = atan2(A, B)
            double phaseAngle = std::atan2(Av, Bv);
            // 主分支：r1 + φ = asin(sinVal)  ⇒  r1 = asin - φ
            double r1cand1 = lcnc::math::radiansToDegrees(
                std::asin(sinVal) - phaseAngle);
            // 次分支：r1 + φ = π - asin(sinVal)
            double r1cand2 = lcnc::math::radiansToDegrees(
                (lcnc::math::kPi - std::asin(sinVal)) - phaseAngle);
            computeAndPick(r1cand1, r1cand2);
        }
    }

    if (previous && previous->valid
        && previous->r1Name == r1Name
        && previous->r2Name == r2Name) {
        result.r1 = equivalentAngleNear(result.r1, previous->r1);
        result.r2 = equivalentAngleNear(result.r2, previous->r2);
    }

    // If one rotary axis is C, first try a forced-tilt candidate: keep the
    // previous A/B angle and solve the C angle from the current normal. This
    // handles tube side holes where both normals are 90 degrees to the tool and
    // the ordinary two-branch solve may flip A/B instead of rotating C.
    if (previous && previous->valid
        && previous->r1Name == r1Name
        && previous->r2Name == r2Name
        && (isCAxisName(r1Name) != isCAxisName(r2Name))) {
        double forcedR1 = result.r1;
        double forcedR2 = result.r2;
        bool hasForced = false;

        if (isCAxisName(r1Name)) {
            forcedR2 = normalizeAxisOutput(r2Name, previous->r2);
            if (withinAxisLimits(*ax2, forcedR2)) {
                gp_Vec desired = target;
                gp_Trsf invTilt = axisRotation(*ax2, -forcedR2);
                desired.Transform(invTilt);
                double solvedC = 0.0;
                if (solveRotationAboutAxis(*ax1, n, desired, solvedC)) {
                    forcedR1 = equivalentAngleNear(solvedC, previous->r1);
                    hasForced = withinAxisLimits(*ax1, forcedR1);
                }
            }
        } else {
            forcedR1 = normalizeAxisOutput(r1Name, previous->r1);
            if (withinAxisLimits(*ax1, forcedR1)) {
                gp_Vec afterTilt = n;
                gp_Trsf tilt = axisRotation(*ax1, forcedR1);
                afterTilt.Transform(tilt);
                double solvedC = 0.0;
                if (solveRotationAboutAxis(*ax2, afterTilt, target, solvedC)) {
                    forcedR2 = equivalentAngleNear(solvedC, previous->r2);
                    hasForced = withinAxisLimits(*ax2, forcedR2);
                }
            }
        }

        if (hasForced) {
            const double currentErr = tableAlignmentError(*ax1, result.r1, *ax2, result.r2, n, target);
            const double forcedErr = tableAlignmentError(*ax1, forcedR1, *ax2, forcedR2, n, target);
            const RotaryBranchCost currentCost =
                rotaryBranchCost(r1Name, result.r1, r2Name, result.r2, currentErr, previous);
            const RotaryBranchCost forcedCost =
                rotaryBranchCost(r1Name, forcedR1, r2Name, forcedR2, forcedErr, previous);
            if (rotaryBranchBetter(forcedCost, currentCost)) {
                result.r1 = forcedR1;
                result.r2 = forcedR2;
            }
        }
    }

    // For table AC/BC tube work, (C, tilt) and (C + 180, -tilt) are equivalent
    // orientation branches. Add this branch explicitly so A/B can stay stable
    // and the C axis absorbs opposite-side tube holes.
    if (previous && previous->valid
        && previous->r1Name == r1Name
        && previous->r2Name == r2Name
        && (isCAxisName(r1Name) != isCAxisName(r2Name))) {
        double altR1 = result.r1;
        double altR2 = result.r2;
        if (isCAxisName(r1Name)) {
            altR1 = result.r1 + 180.0;
            altR2 = -result.r2;
        } else {
            altR1 = -result.r1;
            altR2 = result.r2 + 180.0;
        }

        altR1 = axisComparableValue(r1Name, altR1, previous->r1, true);
        altR2 = axisComparableValue(r2Name, altR2, previous->r2, true);

        if (withinAxisLimits(*ax1, altR1) && withinAxisLimits(*ax2, altR2)) {
            const double currentErr = tableAlignmentError(*ax1, result.r1, *ax2, result.r2, n, target);
            const double alternateErr = tableAlignmentError(*ax1, altR1, *ax2, altR2, n, target);
            const RotaryBranchCost currentCost =
                rotaryBranchCost(r1Name, result.r1, r2Name, result.r2, currentErr, previous);
            const RotaryBranchCost alternateCost =
                rotaryBranchCost(r1Name, altR1, r2Name, altR2, alternateErr, previous);
            if (rotaryBranchBetter(alternateCost, currentCost)) {
                result.r1 = altR1;
                result.r2 = altR2;
            }
        }
    }

    result.r1 = normalizeAxisOutput(r1Name, result.r1);
    result.r2 = normalizeAxisOutput(r2Name, result.r2);

    const double finalAlignmentError = tableAlignmentError(
        *ax1, result.r1, *ax2, result.r2, n, target);
    if (!withinAxisLimits(*ax1, result.r1)
        || !withinAxisLimits(*ax2, result.r2)
        || finalAlignmentError > 1e-5) {
        return result;
    }

    // Step 3: Compute the actual rotation applied to the workpiece
    gp_Trsf rot1Final = axisRotation(*ax1, result.r1);
    gp_Trsf rot2Final = axisRotation(*ax2, result.r2);
    // 链路顺序与 MachineKinematics::chainTrsf(BASE→parent→child) 一致：
    // totalRot = T_parent * T_child（child=r1 先施加，parent=r2 后施加）。
    gp_Trsf totalRot = rot2Final.Multiplied(rot1Final);

    // The tool tip position in world after table rotation:
    // The workpiece point P rotates with the table → P_world = totalRot * P
    // The gantry must go to P_world, so XYZ = P_world.
    gp_Pnt rotatedPos = toolPos.Transformed(totalRot);
    setLinearMachineCoordinates(result, kin, rotatedPos);
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
            double r1a = lcnc::math::radiansToDegrees(baseAngle - phaseAngle);
            double r1b = lcnc::math::radiansToDegrees(
                lcnc::math::kPi - baseAngle - phaseAngle);
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
    rot1.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis1Dir),
                     lcnc::math::degreesToRadians(result.r1));
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
        result.r2 = lcnc::math::radiansToDegrees(angle);
    }

    // Clamp r2
    if (result.r2 > ax2->maxVal) result.r2 = ax2->maxVal;
    if (result.r2 < ax2->minVal) result.r2 = ax2->minVal;

    result.r1 = normalizeAxisOutput(r1Name, result.r1);
    result.r2 = normalizeAxisOutput(r2Name, result.r2);

    // Step 3: Linear axes = tool position (workpiece is fixed in head-type)
    setLinearMachineCoordinates(result, kin, toolPos);
    result.valid = true;

    return result;
}
