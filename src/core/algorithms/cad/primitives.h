#pragma once

#include <TopoDS_Shape.hxx>

namespace lcnc::cad_algo {

/**
 * @brief CAD 基本体生成算法（v2.2 从 commands_cad 抽出）。
 *
 * 参数错误抛出 std::invalid_argument；OCC Standard_Failure 保持传播。
 */

/// 长方体：原点位于 (0,0,0)，沿 +X/+Y/+Z 方向。
TopoDS_Shape makeBox(double dx, double dy, double dz);

/// 圆柱体：底面位于 XY 平面，沿 +Z 方向。
TopoDS_Shape makeCylinder(double radius, double height);

/// 球体：球心位于原点。
TopoDS_Shape makeSphere(double radius);

/// 圆锥/截锥：r2=0 为标准锥；r1==r2 为圆柱。
TopoDS_Shape makeCone(double r1, double r2, double height);

/// 圆环体：r1 主半径（中心距）, r2 管半径。要求 r1 > r2。
TopoDS_Shape makeTorus(double r1, double r2);

} // namespace lcnc::cad_algo
