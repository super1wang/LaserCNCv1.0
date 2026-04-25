#pragma once

#include <TopoDS_Shape.hxx>

class QString;

namespace lcnc::cad_algo {

/**
 * @brief CAD 基本体生成算法（v2.2 从 commands_cad 抽出）。
 *
 * 所有函数为纯 OCC 操作，不涉及 LcncDocument / Qt UI。生成失败返回
 * Null shape 并通过 errMsg（若提供）回写说明。
 */

/// 长方体：原点位于 (0,0,0)，沿 +X/+Y/+Z 方向。
TopoDS_Shape makeBox(double dx, double dy, double dz, QString* errMsg = nullptr);

/// 圆柱体：底面位于 XY 平面，沿 +Z 方向。
TopoDS_Shape makeCylinder(double radius, double height, QString* errMsg = nullptr);

/// 球体：球心位于原点。
TopoDS_Shape makeSphere(double radius, QString* errMsg = nullptr);

/// 圆锥/截锥：r2=0 为标准锥；r1==r2 为圆柱。
TopoDS_Shape makeCone(double r1, double r2, double height, QString* errMsg = nullptr);

/// 圆环体：r1 主半径（中心距）, r2 管半径。要求 r1 > r2。
TopoDS_Shape makeTorus(double r1, double r2, QString* errMsg = nullptr);

} // namespace lcnc::cad_algo
