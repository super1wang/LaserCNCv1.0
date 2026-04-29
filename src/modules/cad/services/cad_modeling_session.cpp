#include "modules/cad/services/cad_modeling_session.h"

#include "core/algorithms/cad/features.h"
#include "core/algorithms/cad/sketch.h"
#include "core/logging/logger.h"

#include <QtGlobal>
#include <gp_Ax1.hxx>

#include <cmath>

namespace lcnc::cad {

namespace {

void setErr(QString* errMsg, const QString& message)
{
    if (errMsg)
        *errMsg = message;
}

lcnc::cad_algo::SketchPlane toSketchPlane(SketchPlaneKind planeKind)
{
    switch (planeKind) {
    case SketchPlaneKind::YZ:
        return lcnc::cad_algo::SketchPlane::yz();
    case SketchPlaneKind::ZX:
        return lcnc::cad_algo::SketchPlane::zx();
    case SketchPlaneKind::XY:
    default:
        return lcnc::cad_algo::SketchPlane::xy();
    }
}

} // namespace

void CadModelingSession::beginSketch(SketchPlaneKind planeKind)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::beginSketch plane={}",
               static_cast<int>(planeKind));
    m_planeKind = planeKind;
    m_state = SessionState::EditingSketch;
    m_sketchTool = SketchToolKind::None;
    m_elements.clear();
    m_nextElementId = 1;
    m_profileFace.Nullify();
}

void CadModelingSession::setSketchProfile(SketchProfileKind profileKind,
                                           double width,
                                           double height,
                                           double radius)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::setSketchProfile kind={} width={} height={} radius={}",
               static_cast<int>(profileKind), width, height, radius);
    m_profileKind = profileKind;
    m_profileWidth = qMax(0.001, width);
    m_profileHeight = qMax(0.001, height);
    m_profileRadius = qMax(0.001, radius);
    if (m_state == SessionState::FinishedSketch)
        m_state = SessionState::EditingSketch;
    m_profileFace.Nullify();
}

SketchToolKind CadModelingSession::sketchTool() const
{
    return m_sketchTool;
}

void CadModelingSession::setSketchTool(SketchToolKind tool)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::setSketchTool kind={}",
               static_cast<int>(tool));
    m_sketchTool = tool;
}

namespace {

int expectedParamCount(SketchToolKind kind)
{
    switch (kind) {
    case SketchToolKind::Point: return 2;
    case SketchToolKind::Line: return 4;
    case SketchToolKind::Arc: return 6;
    case SketchToolKind::Circle: return 3;
    case SketchToolKind::Rectangle: return 4;
    case SketchToolKind::Polygon: return 4;
    default: return 0;
    }
}

QString defaultElementLabel(SketchToolKind kind, int id)
{
    switch (kind) {
    case SketchToolKind::Point: return QStringLiteral("点 %1").arg(id);
    case SketchToolKind::Line: return QStringLiteral("直线 %1").arg(id);
    case SketchToolKind::Arc: return QStringLiteral("圆弧 %1").arg(id);
    case SketchToolKind::Circle: return QStringLiteral("圆 %1").arg(id);
    case SketchToolKind::Rectangle: return QStringLiteral("矩形 %1").arg(id);
    case SketchToolKind::Polygon: return QStringLiteral("多边形 %1").arg(id);
    default: return QStringLiteral("元素 %1").arg(id);
    }
}

} // namespace

int CadModelingSession::addSketchElement(SketchToolKind kind,
                                         const QVector<double>& params,
                                         QString* errMsg)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::addSketchElement begin kind={} params={}",
               static_cast<int>(kind), params.size());
    if (m_state != SessionState::EditingSketch) {
        setErr(errMsg, QStringLiteral("请先新建草图"));
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::addSketchElement end success=false reason=not-editing");
        return -1;
    }
    const int expected = expectedParamCount(kind);
    if (expected == 0 || params.size() != expected) {
        setErr(errMsg, QStringLiteral("草图工具参数数量不正确"));
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::addSketchElement end success=false reason=params");
        return -1;
    }

    SketchElement element;
    element.id = m_nextElementId++;
    element.kind = kind;
    element.params = params;
    element.label = defaultElementLabel(kind, element.id);

    QString wireError;
    const TopoDS_Wire wire = buildElementWire(element, &wireError);
    if (kind != SketchToolKind::Point && wire.IsNull()) {
        setErr(errMsg, wireError.isEmpty() ? QStringLiteral("草图元素几何无效") : wireError);
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::addSketchElement end success=false reason=geometry");
        return -1;
    }

    m_elements.push_back(element);
    if (m_profileFace.IsNull() == false) {
        m_profileFace.Nullify();
        m_state = SessionState::EditingSketch;
    }
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::addSketchElement end success=true id={}", element.id);
    return element.id;
}

bool CadModelingSession::removeSketchElement(int elementId)
{
    for (auto it = m_elements.begin(); it != m_elements.end(); ++it) {
        if (it->id == elementId) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "CadModelingSession::removeSketchElement id={}", elementId);
            m_elements.erase(it);
            m_profileFace.Nullify();
            if (m_state == SessionState::FinishedSketch)
                m_state = SessionState::EditingSketch;
            return true;
        }
    }
    return false;
}

bool CadModelingSession::moveSketchElement(int elementId,
                                           double deltaX,
                                           double deltaY,
                                           QString* errMsg)
{
    if (m_state != SessionState::EditingSketch) {
        setErr(errMsg, QStringLiteral("请先进入草图编辑"));
        return false;
    }

    for (auto& element : m_elements) {
        if (element.id != elementId)
            continue;

        auto movePair = [&](int xIndex, int yIndex) {
            if (element.params.size() > yIndex) {
                element.params[xIndex] += deltaX;
                element.params[yIndex] += deltaY;
            }
        };

        switch (element.kind) {
        case SketchToolKind::Point:
        case SketchToolKind::Circle:
        case SketchToolKind::Rectangle:
        case SketchToolKind::Polygon:
            movePair(0, 1);
            break;
        case SketchToolKind::Line:
            movePair(0, 1);
            movePair(2, 3);
            break;
        case SketchToolKind::Arc:
            movePair(0, 1);
            movePair(2, 3);
            movePair(4, 5);
            break;
        case SketchToolKind::None:
        default:
            setErr(errMsg, QStringLiteral("草图元素类型不支持移动"));
            return false;
        }

        m_profileFace.Nullify();
        return true;
    }

    setErr(errMsg, QStringLiteral("草图元素不存在"));
    return false;
}

bool CadModelingSession::moveSketchElementHandle(int elementId,
                                                 int handleIndex,
                                                 double deltaX,
                                                 double deltaY,
                                                 QString* errMsg)
{
    if (m_state != SessionState::EditingSketch) {
        setErr(errMsg, QStringLiteral("请先进入草图编辑"));
        return false;
    }

    for (auto& element : m_elements) {
        if (element.id != elementId)
            continue;

        auto movePair = [&](int xIndex, int yIndex) {
            if (element.params.size() > yIndex) {
                element.params[xIndex] += deltaX;
                element.params[yIndex] += deltaY;
                return true;
            }
            return false;
        };

        auto updateRadialHandle = [&](double centerX,
                                      double centerY,
                                      double oldRadius,
                                      int radiusIndex) {
            const double handleX = centerX + oldRadius + deltaX;
            const double handleY = centerY + deltaY;
            const double newRadius = std::sqrt((handleX - centerX) * (handleX - centerX)
                                             + (handleY - centerY) * (handleY - centerY));
            if (newRadius <= 1.0e-6)
                return false;
            element.params[radiusIndex] = qMax(0.001, newRadius);
            return true;
        };

        bool handled = false;
        switch (element.kind) {
        case SketchToolKind::Point:
            handled = handleIndex == 0 && movePair(0, 1);
            break;
        case SketchToolKind::Line:
            if (handleIndex == 0)
                handled = movePair(0, 1);
            else if (handleIndex == 1)
                handled = movePair(2, 3);
            break;
        case SketchToolKind::Arc:
            if (handleIndex == 0)
                handled = movePair(0, 1);
            else if (handleIndex == 1)
                handled = movePair(2, 3);
            else if (handleIndex == 2)
                handled = movePair(4, 5);
            break;
        case SketchToolKind::Circle:
            if (handleIndex == 0) {
                handled = movePair(0, 1);
            } else if (handleIndex == 1 && element.params.size() >= 3) {
                handled = updateRadialHandle(element.params[0], element.params[1], element.params[2], 2);
            }
            break;
        case SketchToolKind::Rectangle:
            if (handleIndex == 0) {
                handled = movePair(0, 1);
            } else if (handleIndex >= 1 && handleIndex <= 4 && element.params.size() >= 4) {
                const double centerX = element.params[0];
                const double centerY = element.params[1];
                const double halfWidth = element.params[2] * 0.5;
                const double halfHeight = element.params[3] * 0.5;
                const double signX = (handleIndex == 1 || handleIndex == 4) ? 1.0 : -1.0;
                const double signY = (handleIndex == 1 || handleIndex == 2) ? 1.0 : -1.0;
                const double handleX = centerX + signX * halfWidth + deltaX;
                const double handleY = centerY + signY * halfHeight + deltaY;
                element.params[2] = qMax(0.001, 2.0 * std::abs(handleX - centerX));
                element.params[3] = qMax(0.001, 2.0 * std::abs(handleY - centerY));
                handled = true;
            }
            break;
        case SketchToolKind::Polygon:
            if (handleIndex == 0) {
                handled = movePair(0, 1);
            } else if (handleIndex == 1 && element.params.size() >= 3) {
                handled = updateRadialHandle(element.params[0], element.params[1], element.params[2], 2);
            }
            break;
        case SketchToolKind::None:
        default:
            break;
        }

        if (!handled) {
            setErr(errMsg, QStringLiteral("草图元素手柄不支持移动"));
            return false;
        }

        m_profileFace.Nullify();
        return true;
    }

    setErr(errMsg, QStringLiteral("草图元素不存在"));
    return false;
}

void CadModelingSession::clearSketchElements()
{
    if (m_elements.empty())
        return;
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModelingSession::clearSketchElements");
    m_elements.clear();
    m_profileFace.Nullify();
    if (m_state == SessionState::FinishedSketch)
        m_state = SessionState::EditingSketch;
}

const std::vector<SketchElement>& CadModelingSession::sketchElements() const
{
    return m_elements;
}

TopoDS_Wire CadModelingSession::buildElementWire(const SketchElement& element,
                                                 QString* errMsg) const
{
    const lcnc::cad_algo::SketchPlane plane = toSketchPlane(m_planeKind);
    const auto& p = element.params;
    switch (element.kind) {
    case SketchToolKind::Line:
        return lcnc::cad_algo::makeLineWire(plane,
                                            {p[0], p[1]},
                                            {p[2], p[3]},
                                            errMsg);
    case SketchToolKind::Arc:
        return lcnc::cad_algo::makeArcWire(plane,
                                           {p[0], p[1]},
                                           {p[2], p[3]},
                                           {p[4], p[5]},
                                           errMsg);
    case SketchToolKind::Circle:
        return lcnc::cad_algo::makeCircleWire(plane, p[2], {p[0], p[1]}, errMsg);
    case SketchToolKind::Rectangle:
        return lcnc::cad_algo::makeRectangleWire(plane, p[2], p[3], {p[0], p[1]}, errMsg);
    case SketchToolKind::Polygon:
        return lcnc::cad_algo::makePolygonWire(plane,
                                               static_cast<int>(p[3]),
                                               p[2],
                                               {p[0], p[1]},
                                               errMsg);
    case SketchToolKind::Point:
    case SketchToolKind::None:
    default:
        return {};
    }
}

bool CadModelingSession::finishSketch(QString* errMsg)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::finishSketch begin state={} profile={} elements={}",
               static_cast<int>(m_state),
               static_cast<int>(m_profileKind),
               m_elements.size());
    if (m_state == SessionState::Idle) {
        setErr(errMsg, QStringLiteral("请先新建草图"));
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::finishSketch end success=false reason=idle");
        return false;
    }

    const lcnc::cad_algo::SketchPlane plane = toSketchPlane(m_planeKind);
    TopoDS_Wire wire;
    QString wireError;
    bool fromElements = false;

    // 自动约束骨架：优先尝试用绘制元素列表里第一条闭合 wire；若无可用闭合元素，
    // 再回落到原有参数化矩形/圆。Phase 2b 将引入鼠标驱动的端点容差合并。
    for (const auto& element : m_elements) {
        if (element.kind != SketchToolKind::Circle
            && element.kind != SketchToolKind::Rectangle
            && element.kind != SketchToolKind::Polygon) {
            continue;
        }
        QString elementError;
        wire = buildElementWire(element, &elementError);
        if (!wire.IsNull()) {
            fromElements = true;
            break;
        }
        if (wireError.isEmpty())
            wireError = elementError;
    }

    if (wire.IsNull()) {
        switch (m_profileKind) {
        case SketchProfileKind::Circle:
            wire = lcnc::cad_algo::makeCircleWire(plane, m_profileRadius, {}, &wireError);
            break;
        case SketchProfileKind::Rectangle:
        default:
            wire = lcnc::cad_algo::makeRectangleWire(plane,
                                                     m_profileWidth,
                                                     m_profileHeight,
                                                     {},
                                                     &wireError);
            break;
        }
    }

    if (wire.IsNull()) {
        setErr(errMsg, wireError.isEmpty() ? QStringLiteral("草图轮廓生成失败") : wireError);
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::finishSketch end success=false reason=wire fromElements={}",
                   fromElements);
        return false;
    }

    QString faceError;
    m_profileFace = lcnc::cad_algo::makeFaceFromWire(wire, &faceError);
    if (m_profileFace.IsNull()) {
        setErr(errMsg, faceError.isEmpty() ? QStringLiteral("草图面生成失败") : faceError);
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModelingSession::finishSketch end success=false reason=face");
        return false;
    }

    m_state = SessionState::FinishedSketch;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::finishSketch end success=true fromElements={}",
               fromElements);
    return true;
}

void CadModelingSession::clear()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModelingSession::clear");
    m_state = SessionState::Idle;
    m_sketchTool = SketchToolKind::None;
    m_elements.clear();
    m_nextElementId = 1;
    m_profileFace.Nullify();
}

bool CadModelingSession::isSketchEditing() const
{
    return m_state == SessionState::EditingSketch;
}

bool CadModelingSession::hasFinishedProfile() const
{
    return m_state == SessionState::FinishedSketch && !m_profileFace.IsNull();
}

TopoDS_Shape CadModelingSession::buildFeature(FeatureKind featureKind,
                                               double length,
                                               double angleDeg,
                                               QString* errMsg) const
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModelingSession::buildFeature begin feature={} length={} angle={}",
               static_cast<int>(featureKind), length, angleDeg);
    if (!hasFinishedProfile()) {
        setErr(errMsg, QStringLiteral("请先退出草图以生成可用轮廓"));
        return {};
    }
    return buildFeatureFromRecord(m_planeKind, m_profileFace, featureKind, length, angleDeg, errMsg);
}

TopoDS_Shape CadModelingSession::buildFeatureFromRecord(SketchPlaneKind planeKind,
                                                       const TopoDS_Face& profileFace,
                                                       FeatureKind featureKind,
                                                       double length,
                                                       double angleDeg,
                                                       QString* errMsg)
{
    if (profileFace.IsNull()) {
        setErr(errMsg, QStringLiteral("草图轮廓为空"));
        return {};
    }
    const lcnc::cad_algo::SketchPlane plane = toSketchPlane(planeKind);
    QString featureError;
    TopoDS_Shape result;
    switch (featureKind) {
    case FeatureKind::Revolve:
        result = lcnc::cad_algo::revolveShape(
            profileFace,
            gp_Ax1(plane.axes.Location(), plane.axes.YDirection()),
            angleDeg,
            &featureError);
        break;
    case FeatureKind::Sweep:
        setErr(errMsg, QStringLiteral("扫掠需要路径草图，当前阶段尚未接入路径会话"));
        return {};
    case FeatureKind::Extrude:
    default:
        result = lcnc::cad_algo::extrudeShape(
            profileFace,
            plane.axes.Direction(),
            length,
            &featureError);
        break;
    }
    if (result.IsNull()) {
        setErr(errMsg, featureError.isEmpty() ? QStringLiteral("特征生成失败") : featureError);
        return {};
    }
    return result;
}

QString CadModelingSession::defaultFeatureName(FeatureKind featureKind) const
{
    switch (featureKind) {
    case FeatureKind::Revolve:
        return QStringLiteral("旋转凸台");
    case FeatureKind::Sweep:
        return QStringLiteral("扫掠");
    case FeatureKind::Extrude:
    default:
        return QStringLiteral("拉伸凸台");
    }
}

} // namespace lcnc::cad