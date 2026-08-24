#include "modules/cam/collision/cutter_collision_geometry.h"

#include "modules/cam/settings/cam_config.h"
#include "core/logging/logger.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Reader.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QObject>

#include <cmath>

namespace lcnc::cam {
namespace {

TopoDS_Shape normalizeNozzleDisplayShape(const TopoDS_Shape& source)
{
    if (source.IsNull())
        return {};
    Bnd_Box bounds;
    BRepBndLib::Add(source, bounds);
    if (bounds.IsVoid())
        return {};
    double xMin = 0.0, yMin = 0.0, zMin = 0.0;
    double xMax = 0.0, yMax = 0.0, zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    gp_Trsf normalization;
    normalization.SetTranslation(gp_Vec(
        -0.5 * (xMin + xMax), -0.5 * (yMin + yMax), -zMin));
    BRepBuilderAPI_Transform moved(source, normalization, true);
    return moved.IsDone() ? moved.Shape() : TopoDS_Shape{};
}

TopoDS_Shape loadNozzleDisplayShape(const QString& filePath, QString* error)
{
    const QFileInfo info(filePath);
    if (!info.exists() || !info.isFile()) {
        if (error)
            *error = QObject::tr("The cutting nozzle display model file does not exist");
        return {};
    }
    const QByteArray nativePath = QFile::encodeName(info.absoluteFilePath());
    const QString suffix = info.suffix().toLower();
    try {
        TopoDS_Shape shape;
        if (suffix == QStringLiteral("stp") || suffix == QStringLiteral("step")) {
            const auto readStep = [](const QByteArray& encodedPath) {
                STEPControl_Reader reader;
                if (reader.ReadFile(encodedPath.constData()) != IFSelect_RetDone)
                    return TopoDS_Shape{};
                reader.TransferRoots();
                return reader.OneShape();
            };
            const QByteArray utf8Path = info.absoluteFilePath().toUtf8();
            shape = readStep(utf8Path);
            if (shape.IsNull() && nativePath != utf8Path)
                shape = readStep(nativePath);
            if (shape.IsNull()) {
                if (error)
                    *error = QObject::tr("Unable to read the cutting nozzle STEP model");
                return {};
            }
        } else if (suffix == QStringLiteral("stl")) {
            StlAPI_Reader reader;
            if (!reader.Read(shape, nativePath.constData())) {
                if (error)
                    *error = QObject::tr("Unable to read the cutting nozzle STL model");
                return {};
            }
        } else if (suffix == QStringLiteral("brep")) {
            BRep_Builder builder;
            if (!BRepTools::Read(shape, nativePath.constData(), builder)) {
                if (error)
                    *error = QObject::tr("Unable to read the cutting nozzle BREP model");
                return {};
            }
        } else {
            if (error)
                *error = QObject::tr("Unsupported cutting nozzle display model format");
            return {};
        }
        shape = normalizeNozzleDisplayShape(shape);
        if (shape.IsNull() && error)
            *error = QObject::tr("The cutting nozzle display model contains no usable geometry");
        return shape;
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.cutter_display_proxy: failed to load nozzle geometry: {}",
                 failure.GetMessageString());
        if (error)
            *error = QString::fromUtf8(failure.GetMessageString());
        return {};
    }
}

} // namespace

TopoDS_Shape buildCutterDisplayProxy(const CamConfig& config, QString* errorMessage)
{
    if (config.cutterCollisionProxyMode() == CutterCollisionProxyMode::ModelFile)
        return loadNozzleDisplayShape(config.cutterNozzleModelPath(), errorMessage);
    try {
        if (std::abs(config.simulatedConeTipRadiusMm()
                     - config.simulatedConeBaseRadiusMm()) <= Precision::Confusion()) {
            return BRepPrimAPI_MakeCylinder(config.simulatedConeBaseRadiusMm(),
                                            config.simulatedConeLengthMm()).Shape();
        }
        return BRepPrimAPI_MakeCone(config.simulatedConeTipRadiusMm(),
                                   config.simulatedConeBaseRadiusMm(),
                                   config.simulatedConeLengthMm()).Shape();
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.cutter_display_proxy: failed to build analytic nozzle geometry: {}",
                 failure.GetMessageString());
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.GetMessageString());
        return {};
    }
}

} // namespace lcnc::cam
