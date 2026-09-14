#pragma once

#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <initializer_list>

namespace lcnc::kinematics {

// A table RTCP command describes a point in the zero-pose machine reference,
// not the point after the workpiece carrier has moved in the scene.
// T_home * inverse(T_current) removes only carrier motion; setup is retained.
// Both transforms map the SAME workpiece-local frame to machine world.
// 中文翻译：去除工件载体运动，保留安装变换；不能把场景世界 TCP 直接当作 RTCP 指令点。
inline bool tableRtcpReferencePoint(const gp_Pnt& worldTcp,
                                    const gp_Trsf& workpieceToWorld,
                                    const gp_Trsf& workpieceToHome,
                                    gp_Pnt* referenceTcp)
{
    if (!referenceTcp || !std::isfinite(worldTcp.X())
        || !std::isfinite(worldTcp.Y()) || !std::isfinite(worldTcp.Z()))
        return false;
    for (const gp_Trsf* transform : {&workpieceToWorld, &workpieceToHome}) {
        if (!std::isfinite(transform->ScaleFactor())
            || std::abs(transform->ScaleFactor() - 1.0) > 1e-9)
            return false;
        for (int row = 1; row <= 3; ++row)
            for (int column = 1; column <= 4; ++column)
                if (!std::isfinite(transform->Value(row, column)))
                    return false;
    }
    const gp_Pnt result = worldTcp.Transformed(
        workpieceToHome.Multiplied(workpieceToWorld.Inverted()));
    if (!std::isfinite(result.X()) || !std::isfinite(result.Y())
        || !std::isfinite(result.Z()))
        return false;
    *referenceTcp = result;
    return true;
}

} // namespace lcnc::kinematics
