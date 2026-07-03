#include "core/kinematics/ik_solver.h"

#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <algorithm>
#include <cmath>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

gp_Trsf axisRotation(const MachineAxisDef& axis, double angleDeg)
{
    gp_Trsf trsf;
    trsf.SetRotation(gp_Ax1(axis.origin, axis.direction), angleDeg * M_PI / 180.0);
    return trsf;
}

double tableAxisJumpWeight(const QString& axisName)
{
    const QString n = axisName.trimmed().toUpper();
    if (n == QStringLiteral("C"))
        return 0.25;
    if (n == QStringLiteral("A") || n == QStringLiteral("B"))
        return 24.0;
    return 1.0;
}

double tableAxisHomeWeight(const QString& axisName)
{
    const QString n = axisName.trimmed().toUpper();
    if (n == QStringLiteral("C"))
        return 0.02;
    if (n == QStringLiteral("A") || n == QStringLiteral("B"))
        return 12.0;
    return 1.0;
}

} // namespace

// =============================================================================
// IKSolver — public entry point
// =============================================================================

MachineCoord IKSolver::solve(const MachineKinematics* kin,
                             const gp_Pnt& toolPos,
                             const gp_Dir& toolDir)
{
    return solveContinuous(kin, toolPos, toolDir, nullptr);
}

MachineCoord IKSolver::solveContinuous(const MachineKinematics* kin,
                                        const gp_Pnt& toolPos,
                                        const gp_Dir& toolDir,
                                        const MachineCoord* previous)
{
    MachineCoord result;
    if (!kin) return result;

    const QString cfg = kin->configType();

    // Identify the two rotary axes, distinguishing parent/child by parentAxis chain.
    // 关键：IK 的数学需要 r1 = 先施加的旋转，r2 = 后施加的旋转。
    // MachineKinematics::chainTrsf 走 BASE→parent→child 累乘，OCC 的 Multiplied 语义使
    // 子轴的局部变换被先应用到点上、父轴的变换后应用 —— 因此 **r1 必须是 child**，
    // **r2 必须是 parent**，否则旋转组合反向，机床实际姿态与 IK 求解的不一致，
    // 表现为切割头偏离轮廓点 / 法线对不齐。
    QString rotaryAxes[2];
    int rotaryCount = 0;
    for (const auto& axis : kin->axes()) {
        if (axis.motionType == MachineAxisDef::Rotary && rotaryCount < 2) {
            rotaryAxes[rotaryCount++] = axis.name;
        }
    }
    QString r1Name, r2Name;
    if (rotaryCount == 2) {
        // 父子判定：若 rotaryAxes[1] 的 parentAxis 链路上能到达 rotaryAxes[0]，
        // 则 rotaryAxes[0] 是父；否则反过来。
        const MachineAxisDef* a0 = kin->findAxis(rotaryAxes[0]);
        const MachineAxisDef* a1 = kin->findAxis(rotaryAxes[1]);
        const MachineAxisDef* child  = nullptr;
        const MachineAxisDef* parent = nullptr;
        if (a0 && a1) {
            // 检查 a1 的祖先里是否有 a0
            QString cur = a1->parentAxis;
            while (!cur.isEmpty()) {
                if (cur == a0->name) { child = a1; parent = a0; break; }
                const MachineAxisDef* d = kin->findAxis(cur);
                if (!d) break;
                cur = d->parentAxis;
            }
            if (!child) {
                cur = a0->parentAxis;
                while (!cur.isEmpty()) {
                    if (cur == a1->name) { child = a0; parent = a1; break; }
                    const MachineAxisDef* d = kin->findAxis(cur);
                    if (!d) break;
                    cur = d->parentAxis;
                }
            }
        }
        if (child && parent) {
            r1Name = child->name;   // 先施加（chain 中子轴先作用于点）
            r2Name = parent->name;  // 后施加
        } else {
            // 兼容兜底：保持原有的 m_axes 遍历顺序
            r1Name = rotaryAxes[0];
            r2Name = rotaryAxes[1];
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
        return solveTableType(kin, toolPos, toolDir, r1Name, r2Name, previous);
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
            r2deg = ang * 180.0 / M_PI;
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

            auto unwrapNear = [](double value, double ref) {
                while (value - ref > 180.0) value -= 360.0;
                while (value - ref < -180.0) value += 360.0;
                return value;
            };
            auto rotaryCost = [&](double r1v, double r2v, double err) {
                if (previous && previous->valid
                    && previous->r1Name == r1Name
                    && previous->r2Name == r2Name) {
                    const double u1 = unwrapNear(r1v, previous->r1);
                    const double u2 = unwrapNear(r2v, previous->r2);
                    return err * 100000.0
                         + std::abs(u1 - previous->r1) * tableAxisJumpWeight(r1Name)
                         + std::abs(u2 - previous->r2) * tableAxisJumpWeight(r2Name);
                }
                // 首点没有上一姿态时，仍按轴语义选分支：A/B 是摆角轴，尽量保持稳定；
                // C 是管件夹持旋转轴，允许承担较大的绕管转角。
                return err * 100000.0
                     + std::abs(r1v) * tableAxisHomeWeight(r1Name)
                     + std::abs(r2v) * tableAxisHomeWeight(r2Name);
            };
            const double costA = rotaryCost(a1deg_a, r2_a, err_a);
            const double costB = rotaryCost(a1deg_b, r2_b, err_b);
            const bool pickB = costB < costA;
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
            double r1cand1 = (std::asin(sinVal) - phaseAngle) * 180.0 / M_PI;
            // 次分支：r1 + φ = π - asin(sinVal)
            double r1cand2 = ((M_PI - std::asin(sinVal)) - phaseAngle) * 180.0 / M_PI;
            computeAndPick(r1cand1, r1cand2);
        }
    }

    if (previous && previous->valid
        && previous->r1Name == r1Name
        && previous->r2Name == r2Name) {
        while (result.r1 - previous->r1 > 180.0) result.r1 -= 360.0;
        while (result.r1 - previous->r1 < -180.0) result.r1 += 360.0;
        while (result.r2 - previous->r2 > 180.0) result.r2 -= 360.0;
        while (result.r2 - previous->r2 < -180.0) result.r2 += 360.0;
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
