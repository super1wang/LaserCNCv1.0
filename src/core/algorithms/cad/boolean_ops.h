#pragma once

#include <TopoDS_Shape.hxx>

namespace lcnc::cad_algo {

/**
 * @brief 布尔运算（v2.2 从 commands_cad 抽出）。
 *
 * 参数错误抛出 std::invalid_argument；OCC Standard_Failure 保持传播。
 */

/// A ∪ B
TopoDS_Shape fuseShapes(const TopoDS_Shape& a, const TopoDS_Shape& b);

/// A − B
TopoDS_Shape cutShapes(const TopoDS_Shape& a, const TopoDS_Shape& b);

/// A ∩ B
TopoDS_Shape commonShapes(const TopoDS_Shape& a, const TopoDS_Shape& b);

} // namespace lcnc::cad_algo
