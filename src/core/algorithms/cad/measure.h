#pragma once

#include <gp_Vec.hxx>

class TopoDS_Shape;

namespace lcnc::cad_algo {

/**
 * @brief 测量算法（v2.2 从 commands_cad 抽出）。
 */

/// 两形体最小距离。参数错误或 OCC 失败向上传播。
double minDistance(const TopoDS_Shape& a, const TopoDS_Shape& b);

/// 形体首面（TopExp_Explorer 的第一个 FACE）的法向量。没有面时返回零向量。
gp_Vec firstFaceNormal(const TopoDS_Shape& shape);

/// 两向量夹角（度）。任意一方为零向量返回 -1。
double angleBetween(const gp_Vec& a, const gp_Vec& b);

/// 形体表面积（mm²）。参数错误或 OCC 失败向上传播。
double surfaceArea(const TopoDS_Shape& shape);

} // namespace lcnc::cad_algo
