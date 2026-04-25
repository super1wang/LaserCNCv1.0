#pragma once

#include <gp_Pnt.hxx>

class QPoint;
class QString;
class WidgetOccView;
class LaserToolpath;

namespace lcnc::cam::reference_pick {

/**
 * @brief 屏幕拾取纯工具（v2.3 从 CamModule 抽出）。
 *
 * 只读 WidgetOccView + LaserToolpath，不修改外部状态。失败时返回 false，
 * 通过 errorMessage 回写原因。调用方负责把结果写回 CamModule 字段 +
 * emit operationFailed 信号。
 */

/// 屏幕坐标拾取 OCC 视图中的平面面片，返回其几何中心。
/// 支持 LocalTransformation 叠加，满足挂载工件场景。
bool resolveReferencePlaneCenter(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 gp_Pnt& center,
                                 QString* errorMessage = nullptr);

/// 在给定 LaserToolpath 上寻找离屏幕点最近的（启用轮廓中的）采样点。
/// 距离阈值 24 像素，找不到返回 false。
bool resolveLeadInHit(WidgetOccView* occView,
                      const QPoint& screenPos,
                      const LaserToolpath& toolpath,
                      int& contourIdx,
                      gp_Pnt& entryPoint,
                      double& entryParam);

} // namespace lcnc::cam::reference_pick
