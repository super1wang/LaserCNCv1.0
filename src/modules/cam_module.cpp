#include "modules/cam_module.h"
#include "modules/shape_service.h"

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/laser_toolpath.h"
#include "base/machine_kinematics.h"
#include "base/task_manager.h"
#include "base/xcaf_utils.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"
#include "graphics/graphics_scene.h"

#include <QFileInfo>
#include <QTimer>
#include <QSet>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS_Compound.hxx>
#include <Quantity_Color.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

CamModule* CamModule::s_instance = nullptr;

CamModule* CamModule::instance()
{
    if (!s_instance)
        s_instance = new CamModule();
    return s_instance;
}

CamModule::CamModule(QObject* parent)
    : QObject(parent)
{
    LcncApplication::instance()->ensureMachineDocument();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(50);
    connect(m_simTimer, &QTimer::timeout, this, &CamModule::onSimTick);
}

// ── Machine Document ──────────────────────────────────────────────────────────

LcncDocument* CamModule::machineDocument() const
{
    return LcncApplication::instance()->machineDocument();
}

GuiDocument* CamModule::machineGuiDocument() const
{
    return GuiApplication::instance()->machineGuiDocument();
}

DocumentId CamModule::machineDocumentId() const
{
    return LcncApplication::instance()->machineDocumentId();
}

MachineKinematics* CamModule::kinematics() const
{
    if (auto* doc = machineDocument())
        return doc->machineKinematics();
    return nullptr;
}

// ── Machine Management ────────────────────────────────────────────────────────

void CamModule::loadMachine(const QString& filePath, const QString& presetName)
{
    LcncDocument* doc = machineDocument();
    if (!doc || filePath.isEmpty()) return;

    QFileInfo fi(filePath);
    const QString ext = fi.suffix().toLower();

    // Clear previous machine entities
    {
        TDF_LabelSequence existing = doc->entityLabels(LcncDocument::EntityKind::Machine);
        QStringList entriesToRemove;
        for (int i = 1; i <= existing.Length(); ++i)
            entriesToRemove << XcafUtils::entry(existing.Value(i));
        for (const QString& e : entriesToRemove)
            ShapeService::deleteShape(doc, e);
    }

    // Clear mounted workpieces as well
    unmountAllWorkpieces();

    // Reset kinematics
    doc->machineKinematics()->clear();
    doc->machineKinematics()->loadPreset(presetName);

    TaskId taskId = TaskManager::instance()->run(tr("加载机台: %1").arg(fi.fileName()),
        [filePath, ext, doc](TaskProgress* prog) {
            prog->setRange(0, 100);

            if (ext == "stp" || ext == "step") {
                prog->setStepName(QStringLiteral("读取 STEP..."));
                Handle(TDocStd_Document) xdeDoc =
                    new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
                XCAFDoc_DocumentTool::Set(xdeDoc->Main());
                STEPCAFControl_Reader cafReader;
                cafReader.SetNameMode(Standard_True);
                if (cafReader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
                    prog->setValue(50);
                    prog->setStepName(QStringLiteral("转换形体..."));
                    cafReader.Transfer(xdeDoc);
                    prog->setValue(80);
                    doc->importFromXcafRoots(xdeDoc, LcncDocument::EntityKind::Machine);
                }
            } else if (ext == "stl") {
                prog->setStepName(QStringLiteral("读取 STL..."));
                TopoDS_Shape shape;
                StlAPI_Reader stlReader;
                stlReader.Read(shape, filePath.toUtf8().constData());
                prog->setValue(80);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(filePath).baseName(), LcncDocument::EntityKind::Machine);
            } else if (ext == "brep") {
                prog->setStepName(QStringLiteral("读取 BREP..."));
                TopoDS_Shape shape;
                BRep_Builder builder;
                BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(filePath).baseName(), LcncDocument::EntityKind::Machine);
            }

            prog->setValue(100);
        });

    connect(TaskManager::instance(), &TaskManager::taskFinished, this,
            [this, doc, taskId](TaskId id, bool ok) {
                if (id != taskId || !ok) return;
                autoDetectAxes();
                refreshMachineDisplay();
                emit machineLoaded();
            });
}

void CamModule::unloadMachine()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    // Remove machine entities
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QStringList entries;
    for (int i = 1; i <= labels.Length(); ++i)
        entries << XcafUtils::entry(labels.Value(i));
    for (const QString& e : entries)
        ShapeService::deleteShape(doc, e);

    // Remove mounted workpieces
    unmountAllWorkpieces();

    doc->machineKinematics()->clear();
    clearToolpath();
    refreshMachineDisplay();
    emit machineUnloaded();
}

void CamModule::exportMachine(const QString& filePath)
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc || filePath.isEmpty()) return;

    MachineKinematics* kin = machDoc->machineKinematics();

    Handle(TDocStd_Document) xdeExport =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stExp = XCAFDoc_DocumentTool::ShapeTool(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stMach = machDoc->shapeTool();

    QSet<QString> assignedEntries;

    for (const MachineAxisDef& axis : kin->axes()) {
        const QStringList entries = kin->shapesForAxis(axis.name);
        if (entries.isEmpty()) continue;

        BRep_Builder bb;
        TopoDS_Compound axisCompound;
        bb.MakeCompound(axisCompound);
        bool hasShape = false;

        TDF_LabelSequence freeShapes;
        stMach->GetFreeShapes(freeShapes);

        for (const QString& entry : entries) {
            assignedEntries.insert(entry);
            for (int i = 1; i <= freeShapes.Length(); ++i) {
                if (XcafUtils::entry(freeShapes.Value(i)) == entry) {
                    TopoDS_Shape sh = stMach->GetShape(freeShapes.Value(i));
                    if (!sh.IsNull()) {
                        bb.Add(axisCompound, sh);
                        hasShape = true;
                    }
                    break;
                }
            }
        }

        if (!hasShape) continue;

        const QString axisLabel = QStringLiteral("LCNC_AXIS_") + axis.name;
        TDF_Label lbl = stExp->AddShape(axisCompound, Standard_False);
        TDataStd_Name::Set(lbl, TCollection_ExtendedString(axisLabel.toStdString().c_str()));
    }

    // Unassigned group
    {
        TDF_LabelSequence freeShapes;
        stMach->GetFreeShapes(freeShapes);
        BRep_Builder bb;
        TopoDS_Compound unassigned;
        bb.MakeCompound(unassigned);
        bool hasUnassigned = false;

        for (int i = 1; i <= freeShapes.Length(); ++i) {
            const QString entry = XcafUtils::entry(freeShapes.Value(i));
            if (!assignedEntries.contains(entry)) {
                TopoDS_Shape sh = stMach->GetShape(freeShapes.Value(i));
                if (!sh.IsNull()) {
                    bb.Add(unassigned, sh);
                    hasUnassigned = true;
                }
            }
        }

        if (hasUnassigned) {
            TDF_Label lbl = stExp->AddShape(unassigned, Standard_False);
            TDataStd_Name::Set(lbl, TCollection_ExtendedString("LCNC_AXIS_UNASSIGNED"));
        }
    }

    STEPCAFControl_Writer writer;
    writer.SetNameMode(Standard_True);
    if (writer.Transfer(xdeExport) != IFSelect_RetDone) return;
    writer.Write(filePath.toUtf8().constData());
}

void CamModule::autoDetectAxes()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, QString> entryToName;
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        entryToName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
    }
    doc->machineKinematics()->autoDetect(entryToName);
}

// ── Workpiece Mounting ────────────────────────────────────────────────────────

void CamModule::mountWorkpiece(DocumentId sourceDocId, const QString& axisName)
{
    LcncDocument* machDoc = machineDocument();
    LcncDocument* srcDoc = LcncApplication::instance()->documentById(sourceDocId);
    if (!machDoc || !srcDoc) return;

    MachineKinematics* kin = machDoc->machineKinematics();
    if (kin->axes().isEmpty()) return;

    if (axisName.isEmpty()) {
        unmountAllWorkpieces();
        return;
    }

    TDF_LabelSequence srcLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    Handle(XCAFDoc_ShapeTool) srcSt = srcDoc->shapeTool();

    BRep_Builder bb;
    TopoDS_Compound compound;
    bb.MakeCompound(compound);
    bool hasShape = false;

    for (int i = 1; i <= srcLabels.Length(); ++i) {
        TopoDS_Shape sh = srcSt->GetShape(srcLabels.Value(i));
        if (!sh.IsNull()) {
            bb.Add(compound, sh);
            hasShape = true;
        }
    }
    if (!hasShape) return;

    unmountAllWorkpieces();

    // Auto-snap on Z
    TopoDS_Shape mountShape = compound;
    const QStringList axisEntries = kin->shapesForAxis(axisName);
    Handle(XCAFDoc_ShapeTool) machSt = machDoc->shapeTool();
    TDF_LabelSequence machFree;
    machSt->GetFreeShapes(machFree);

    Bnd_Box axisBox;
    for (const QString& ae : axisEntries) {
        for (int i = 1; i <= machFree.Length(); ++i) {
            if (XcafUtils::entry(machFree.Value(i)) == ae) {
                const TopoDS_Shape sh = machSt->GetShape(machFree.Value(i));
                if (!sh.IsNull()) BRepBndLib::Add(sh, axisBox);
                break;
            }
        }
    }

    if (!axisBox.IsVoid()) {
        Bnd_Box wpcBox;
        BRepBndLib::Add(compound, wpcBox);
        if (!wpcBox.IsVoid()) {
            Standard_Real x0, y0, z0, x1, y1, z1;
            axisBox.Get(x0, y0, z0, x1, y1, z1);
            Standard_Real wx0, wy0, wz0, wx1, wy1, wz1;
            wpcBox.Get(wx0, wy0, wz0, wx1, wy1, wz1);
            const double dZ = z1 - wz0;
            if (std::abs(dZ) > 1e-6) {
                gp_Trsf trsf;
                trsf.SetTranslation(gp_Vec(0.0, 0.0, dZ));
                BRepBuilderAPI_Transform xform(compound, trsf, Standard_True);
                if (xform.IsDone()) mountShape = xform.Shape();
            }
        }
    }

    const QString wpcName = tr("工件 — %1").arg(srcDoc->name());
    TDF_Label wpcLabel = machDoc->addShapeEntity(mountShape, wpcName,
                                                 LcncDocument::EntityKind::Workpiece);
    const QString wpcEntry = XcafUtils::entry(wpcLabel);
    kin->mountWorkpiece(wpcEntry, axisName);

    refreshMachineDisplay();
    emit workpieceMounted(wpcEntry);
}

void CamModule::unmountAllWorkpieces()
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc) return;

    MachineKinematics* kin = machDoc->machineKinematics();
    TDF_LabelSequence existingWpc = machDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    QStringList toRemove;
    for (int i = 1; i <= existingWpc.Length(); ++i)
        toRemove << XcafUtils::entry(existingWpc.Value(i));
    for (const QString& e : toRemove) {
        kin->unmountWorkpiece(e);
        ShapeService::deleteShape(machDoc, e);
    }

    clearToolpath();
    refreshMachineDisplay();
    emit workpieceUnmounted();
}

// ── Shape Operations ──────────────────────────────────────────────────────────

bool CamModule::moveShape(const QString& entry, const gp_Vec& translation)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, labels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, wpcLabels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

bool CamModule::rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, labels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, wpcLabels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

void CamModule::deleteShape(const QString& entry)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return;

    if (auto* gd = machineGuiDocument())
        gd->eraseEntity(entry);

    ShapeService::deleteShape(doc, entry);
    refreshMachineDisplay();
}

// ── Toolpath ──────────────────────────────────────────────────────────────────

TopoDS_Shape CamModule::collectWorkpieceShape() const
{
    LcncDocument* doc = machineDocument();
    if (!doc) return {};

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (labels.Length() == 0) return {};

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    if (labels.Length() == 1)
        return st->GetShape(labels.Value(1));

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (int i = 1; i <= labels.Length(); ++i)
        builder.Add(compound, st->GetShape(labels.Value(i)));
    return compound;
}

bool CamModule::generateToolpath(double smoothAngle, bool useFaceClassification, double deflection)
{
    TopoDS_Shape wpcShape = collectWorkpieceShape();
    if (wpcShape.IsNull()) return false;

    clearToolpath();
    m_workpieceShape = wpcShape;
    m_smoothAngle = smoothAngle;
    m_useFaceClassification = useFaceClassification;

    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = smoothAngle;
    params.useFaceClassification = useFaceClassification;
    params.deflection = deflection;

    auto contours = LaserToolpathBuilder::extractContours(wpcShape, params);
    if (contours.empty()) return false;

    for (auto& c : contours) {
        if (c.points.empty())
            LaserToolpathBuilder::discretizeContour(c, wpcShape);
    }

    m_toolpath.contours() = std::move(contours);
    m_toolpathVisible = true;

    LcncDocument* machDoc = machineDocument();
    if (machDoc) {
        MachineKinematics* kin = machDoc->machineKinematics();
        gp_Trsf wpcTrsf;
        const auto& wpcMounts = kin->wpcMounts();
        if (!wpcMounts.isEmpty())
            wpcTrsf = kin->computeWpcTransform(wpcMounts.firstKey());
        for (int i = 0; i < m_toolpath.contourCount(); ++i)
            LaserToolpathBuilder::computeMachineCoordinates(m_toolpath.contour(i), kin, wpcTrsf);
    }

    refreshToolpathDisplay();
    emit toolpathGenerated();
    return true;
}

void CamModule::clearToolpath()
{
    eraseToolpathDisplay();
    m_toolpath.clear();
    m_workpieceShape.Nullify();
    emit toolpathCleared();
}

const LaserToolpath& CamModule::toolpath() const
{
    return m_toolpath;
}

LaserToolpath& CamModule::toolpathRef()
{
    return m_toolpath;
}

bool CamModule::hasToolpath() const
{
    return m_toolpath.contourCount() > 0;
}

void CamModule::setLeadInEntry(int contourIdx, const gp_Pnt& entryPoint, double entryParam)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount()) return;

    LaserContour& c = m_toolpath.contour(contourIdx);
    c.leadIn.entryPoint = entryPoint;
    c.leadIn.entryParam = entryParam;
    c.leadIn.valid = true;

    refreshToolpathDisplay();
}

void CamModule::setLeadInLength(double mm)
{
    m_toolpath.setGlobalLeadInLength(mm);
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

double CamModule::leadInLength() const
{
    return m_toolpath.globalLeadInLength();
}

void CamModule::setNormalAngle(double deg)
{
    m_toolpath.setGlobalNormalAngle(deg);
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

double CamModule::normalAngle() const
{
    return m_toolpath.globalNormalAngle();
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return;

    LaserContour& contour = m_toolpath.contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return;

    QSet<int> seen;
    std::vector<LaserContour> current = std::move(m_toolpath.contours());
    std::vector<LaserContour> reordered;
    reordered.reserve(current.size());

    for (int index : order) {
        if (index < 0 || index >= static_cast<int>(current.size()) || seen.contains(index)) {
            m_toolpath.contours() = std::move(current);
            return;
        }

        seen.insert(index);
        reordered.push_back(std::move(current[index]));
    }

    m_toolpath.contours() = std::move(reordered);
}

void CamModule::recalcToolpath()
{
    if (!m_workpieceShape.IsNull()) {
        for (int i = 0; i < m_toolpath.contourCount(); ++i) {
            LaserContour& c = m_toolpath.contour(i);
            LaserToolpathBuilder::discretizeContour(c, m_workpieceShape);
        }
    }

    LcncDocument* machDoc = machineDocument();
    if (machDoc) {
        MachineKinematics* kin = machDoc->machineKinematics();
        gp_Trsf wpcTrsf;
        const auto& wpcMounts = kin->wpcMounts();
        if (!wpcMounts.isEmpty())
            wpcTrsf = kin->computeWpcTransform(wpcMounts.firstKey());
        for (int i = 0; i < m_toolpath.contourCount(); ++i)
            LaserToolpathBuilder::computeMachineCoordinates(m_toolpath.contour(i), kin, wpcTrsf);
    }

    refreshToolpathDisplay();
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathVisible == visible) return;
    m_toolpathVisible = visible;

    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    const Handle(AIS_InteractiveContext)& aisCtx = gd->context();
    if (aisCtx.IsNull()) return;

    auto toggleList = [&](QList<Handle(AIS_Shape)>& list) {
        for (auto& ais : list) {
            if (ais.IsNull()) continue;
            if (m_toolpathVisible)
                aisCtx->Display(ais, Standard_False);
            else
                aisCtx->Erase(ais, Standard_False);
        }
    };

    toggleList(m_contourAis);
    toggleList(m_leadInAis);

    if (gd->hasView())
        gd->view()->Redraw();

    emit toolpathVisibilityChanged(visible);
}

bool CamModule::isToolpathVisible() const
{
    return m_toolpathVisible;
}

double CamModule::smoothAngle() const
{
    return m_smoothAngle;
}

void CamModule::setSmoothAngle(double deg)
{
    m_smoothAngle = deg;
}

bool CamModule::useFaceClassification() const
{
    return m_useFaceClassification;
}

void CamModule::setUseFaceClassification(bool on)
{
    m_useFaceClassification = on;
}

void CamModule::displayContours()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();
    const Quantity_Color green(0.1, 0.8, 0.2, Quantity_TOC_RGB);

    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const LaserContour& c = m_toolpath.contour(i);
        if (!c.enabled || c.wire.IsNull()) continue;

        Handle(AIS_Shape) ais = scene->displayShape(c.wire, false, true);
        scene->setShapeColor(ais, green);
        m_contourAis.append(ais);
    }
}

void CamModule::displayLeadIns()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();
    const Quantity_Color red(0.9, 0.15, 0.15, Quantity_TOC_RGB);

    const double length = m_toolpath.globalLeadInLength();
    const double angle = m_toolpath.globalNormalAngle();

    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const LaserContour& c = m_toolpath.contour(i);
        if (!c.enabled || !c.leadIn.valid) continue;

        TopoDS_Edge leadEdge = LaserToolpathBuilder::computeLeadInEdge(c, length, angle);
        if (leadEdge.IsNull()) continue;

        Handle(AIS_Shape) ais = scene->displayShape(leadEdge, false, false);
        scene->setShapeColor(ais, red);
        m_leadInAis.append(ais);
    }
}

void CamModule::refreshToolpathDisplay()
{
    eraseToolpathDisplay();
    if (m_toolpathVisible) {
        displayContours();
        displayLeadIns();
    }

    if (auto* gd = machineGuiDocument()) {
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::eraseToolpathDisplay()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();

    for (auto& ais : m_contourAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    for (auto& ais : m_leadInAis)
        if (!ais.IsNull()) scene->eraseShape(ais);

    m_contourAis.clear();
    m_leadInAis.clear();
}

const QList<Handle(AIS_Shape)>& CamModule::contourAis() const
{
    return m_contourAis;
}

// ── Simulation ────────────────────────────────────────────────────────────────

void CamModule::simulatePlay()
{
    if (m_simPaused) {
        m_simPaused = false;
        m_simPlaying = true;
        m_simTimer->start();
        emit simulationStateChanged(true);
        return;
    }

    m_simCurrentContour = 0;
    m_simCurrentPoint = 0;
    m_simTotalPoints = 0;
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const auto& c = m_toolpath.contour(i);
        if (c.enabled)
            m_simTotalPoints += static_cast<int>(c.points.size());
    }
    if (m_simTotalPoints == 0) return;

    while (m_simCurrentContour < m_toolpath.contourCount()
           && !m_toolpath.contour(m_simCurrentContour).enabled)
        ++m_simCurrentContour;

    m_simPlaying = true;
    m_simPaused = false;
    m_simTimer->setInterval(std::max(10, static_cast<int>(50.0 / m_simSpeed)));
    m_simTimer->start();
    emit simulationStateChanged(true);
}

void CamModule::simulatePause()
{
    m_simTimer->stop();
    m_simPaused = true;
    m_simPlaying = false;
    emit simulationStateChanged(false);
}

void CamModule::simulateStop()
{
    m_simTimer->stop();
    m_simPlaying = false;
    m_simPaused = false;
    m_simCurrentContour = 0;
    m_simCurrentPoint = 0;
    emit simulationStateChanged(false);
    emit simulationFinished();
}

void CamModule::setSimulationSpeed(double factor)
{
    m_simSpeed = (factor > 0.1) ? factor : 0.1;
    if (m_simTimer->isActive())
        m_simTimer->setInterval(std::max(10, static_cast<int>(50.0 / m_simSpeed)));
}

bool CamModule::isSimulating() const
{
    return m_simPlaying;
}

bool CamModule::isSimPaused() const
{
    return m_simPaused;
}

void CamModule::setAxisPosition(const QString& axisName, double value)
{
    if (auto* kin = kinematics()) {
        kin->setAxisPosition(axisName, value);
        if (auto* gd = machineGuiDocument()) {
            gd->updateAxisTransforms();
            if (gd->hasView())
                gd->view()->Redraw();
        }
    }
}

void CamModule::onSimTick()
{
    if (m_simCurrentContour >= m_toolpath.contourCount()) {
        simulateStop();
        return;
    }

    const LaserContour& c = m_toolpath.contour(m_simCurrentContour);
    if (m_simCurrentPoint >= static_cast<int>(c.points.size())) {
        ++m_simCurrentContour;
        m_simCurrentPoint = 0;
        while (m_simCurrentContour < m_toolpath.contourCount()
               && !m_toolpath.contour(m_simCurrentContour).enabled)
            ++m_simCurrentContour;
        if (m_simCurrentContour >= m_toolpath.contourCount()) {
            simulateStop();
            return;
        }
        return;
    }

    const MachineCoord& mc = c.points[m_simCurrentPoint].machineCoord;
    if (!mc.valid) {
        ++m_simCurrentPoint;
        return;
    }

    if (auto* kin = kinematics()) {
        kin->setAxisPosition("X", mc.x);
        kin->setAxisPosition("Y", mc.y);
        kin->setAxisPosition("Z", mc.z);
        if (!mc.r1Name.isEmpty())
            kin->setAxisPosition(mc.r1Name, mc.r1);
        if (!mc.r2Name.isEmpty())
            kin->setAxisPosition(mc.r2Name, mc.r2);

        if (auto* gd = machineGuiDocument()) {
            gd->updateAxisTransforms();
            if (gd->hasView())
                gd->view()->Redraw();
        }
    }

    emit simulationTick(m_simCurrentContour, m_simCurrentPoint, m_simTotalPoints);
    ++m_simCurrentPoint;
}

void CamModule::refreshMachineDisplay()
{
    if (auto* gd = machineGuiDocument()) {
        gd->rebuildDisplay();
        gd->updateAxisTransforms();
        if (gd->hasView())
            gd->view()->Redraw();
    }
    LcncApplication::instance()->notifyDocumentModified(machineDocumentId());
}
