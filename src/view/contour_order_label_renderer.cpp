#include "view/contour_order_label_renderer.h"

#include "view/gui_document.h"
#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveContext.hxx>
#include <Aspect_TypeOfDisplayText.hxx>
#include <Font_FontAspect.hxx>
#include <Graphic3d_HorizontalTextAlignment.hxx>
#include <Graphic3d_VerticalTextAlignment.hxx>
#include <Quantity_Color.hxx>
#include <TCollection_ExtendedString.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <QByteArray>

namespace lcnc::view {

namespace {
/// 把轮廓局部起点经 WPC 变换换算到世界坐标。kin/entry 缺省时退化为恒等。
gp_Pnt worldPosition(MachineKinematics* kin, const QString& entry, const gp_Pnt& localPoint)
{
    if (kin && !entry.isEmpty())
        return localPoint.Transformed(kin->computeWpcTransform(entry));
    return localPoint;
}
} // namespace

ContourOrderLabelRenderer::ContourOrderLabelRenderer() = default;
ContourOrderLabelRenderer::~ContourOrderLabelRenderer() = default;

void ContourOrderLabelRenderer::setVisible(bool on)
{
    m_visible = on;
}

void ContourOrderLabelRenderer::erase(GuiDocument* gd)
{
    if (m_entries.isEmpty()) return;
    if (gd) {
        const Handle(AIS_InteractiveContext)& ctx = gd->context();
        if (!ctx.IsNull()) {
            for (Entry& e : m_entries) {
                if (!e.ais.IsNull())
                    ctx->Erase(e.ais, false);
            }
        }
    }
    m_entries.clear();
}

void ContourOrderLabelRenderer::refresh(GuiDocument* gd, MachineKinematics* kin, const QVector<Label>& labels)
{
    // 不可见 / 数据为空 -> 仅擦除
    if (!gd || !m_visible || labels.isEmpty()) {
        erase(gd);
        return;
    }
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    // 先擦旧
    erase(gd);

    m_entries.reserve(labels.size());
    for (const Label& label : labels) {
        // 序号文字（纯 ASCII 数字）。
        const QByteArray orderText = QString::number(label.order).toUtf8();
        const gp_Pnt localPoint(label.sx, label.sy, label.sz);

        Handle(AIS_TextLabel) text = new AIS_TextLabel();
        text->SetText(TCollection_ExtendedString(orderText.constData()));
        // AIS_TextLabel 不响应 SetLocalTransformation，直接把局部起点变换到世界坐标后写入。
        text->SetPosition(worldPosition(kin, label.workpieceEntry, localPoint));

        // 暖黄色文字 + 黑色描边底，保证在任意背景上的可读性。
        text->SetColor(Quantity_Color(1.0, 0.82, 0.20, Quantity_TOC_RGB));
        text->SetDisplayType(Aspect_TODT_DEKALE);
        text->SetColorSubTitle(Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_RGB));
        // 固定屏幕像素高度，缩放视图时序号不随模型尺度变化。
        text->SetZoomable(false);
        text->SetHeight(16.0);
        text->SetOwnAnchorPoint(true);
        text->SetHJustification(Graphic3d_HTA_CENTER);
        text->SetVJustification(Graphic3d_VTA_CENTER);

        ctx->Display(text, false);
        ctx->Deactivate(text); // 禁拾取，避免干扰轮廓多选

        m_entries.append({text, label.workpieceEntry, localPoint});
    }
}

void ContourOrderLabelRenderer::updateTransforms(GuiDocument* gd, MachineKinematics* kin)
{
    if (!gd || m_entries.isEmpty())
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    for (Entry& e : m_entries) {
        if (e.ais.IsNull()) continue;
        e.ais->SetPosition(worldPosition(kin, e.workpieceEntry, e.localPoint));
        ctx->RecomputePrsOnly(e.ais, false);
    }
}

} // namespace lcnc::view
