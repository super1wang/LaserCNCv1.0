#pragma once

#include <TopoDS_Shape.hxx>

class QString;

namespace lcnc::cad_algo {

/**
 * @brief 布尔运算（v2.2 从 commands_cad 抽出）。
 *
 * 失败时返回 Null shape 并通过 errMsg 回写说明。
 */

/// A ∪ B
TopoDS_Shape fuseShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg = nullptr);

/// A − B
TopoDS_Shape cutShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg = nullptr);

/// A ∩ B
TopoDS_Shape commonShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg = nullptr);

} // namespace lcnc::cad_algo
