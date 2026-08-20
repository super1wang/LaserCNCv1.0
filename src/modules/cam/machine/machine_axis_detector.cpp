#include "modules/cam/machine/machine_axis_detector.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/settings/cam_config.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Shape.hxx>

namespace lcnc::cam::machine_axis_detector {

void autoDetectAxisNames(LcncDocument* doc, MachineKinematics* kin)
{
    if (!doc || !kin)
        return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, QString> entryToName;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        entryToName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
    }
    kin->autoDetect(entryToName);
}

void autoDetectAxisOrigins(LcncDocument* doc, MachineKinematics* kin)
{
    if (!doc || !kin)
        return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, TDF_Label> labelByEntry;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        labelByEntry.insert(XcafUtils::entry(lbl), lbl);
    }

    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.name == QStringLiteral("BASE") || axis.motionType != MachineAxisDef::Rotary)
            continue;

        Bnd_Box bbox;
        bool hasShape = false;
        for (const QString& entry : kin->shapesForAxis(axis.name)) {
            const auto it = labelByEntry.constFind(entry);
            if (it == labelByEntry.cend())
                continue;

            const TopoDS_Shape shape = XcafUtils::shape(it.value());
            if (shape.IsNull())
                continue;

            BRepBndLib::Add(shape, bbox);
            hasShape = true;
        }

        if (!hasShape || bbox.IsVoid())
            continue;

        Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
        Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        kin->setAxisOrigin(axis.name,
                           gp_Pnt(0.5 * (xmin + xmax),
                                  0.5 * (ymin + ymax),
                                  0.5 * (zmin + zmax)));
    }
}

StoredProfile applyStoredMachineProfile(MachineKinematics* kin,
                                        ::CamConfig& config,
                                        const QString& machinePath)
{
    StoredProfile profile;
    if (!kin || machinePath.isEmpty())
        return profile;

    Q_UNUSED(kin);

    gp_Pnt storedPosition;
    if (config.cutterHeadModelPositionForMachine(machinePath, &storedPosition)) {
        profile.cutterHeadModelPosition = storedPosition;
        profile.hasCutterHeadModel = true;
    }
    if (config.cutterHeadPhysicalPositionForMachine(machinePath, &storedPosition)) {
        profile.cutterHeadPhysicalPosition = storedPosition;
        profile.hasCutterHeadPhysical = true;
    }
    if (config.workpieceInstallPositionForMachine(machinePath, &storedPosition)) {
        profile.workpieceInstallPosition = storedPosition;
        profile.hasWorkpieceInstall = true;
    }
    return profile;
}

} // namespace lcnc::cam::machine_axis_detector
