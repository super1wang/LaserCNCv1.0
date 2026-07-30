// Verification for the strategy-based contour extraction:
//   - PlanarFaceWires on a perforated plate yields N+1 contours (1 outer + N
//     holes), all holes classified InnerHole, ordered holes-first / outer-last.
//   - Auto (machine+posture-driven via a beam direction) picks the top face and
//     produces the same N+1 result without kinematics.
//   - TubeClassification on a cylinder still yields cross-section contours
//     (regression guard for the existing tube path after the dispatch refactor).

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/project/cam/cam_data_manager.h"

#include <QTextStream>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

/// A 100x100x10 plate (z: 0..10) with N through-holes drilled along +Z.
TopoDS_Shape makePerforatedPlate(int holeCount)
{
    const TopoDS_Solid box = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(100.0, 100.0, 10.0)).Solid();

    TopoDS_Shape result = box;
    for (int i = 0; i < holeCount; ++i) {
        const double cx = 20.0 + 30.0 * i;
        const double cy = 50.0;
        const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(cx, cy, -5.0), gp_Dir(0.0, 0.0, 1.0)), 5.0, 20.0).Shape();
        result = BRepAlgoAPI_Cut(result, cyl).Shape();
    }
    return result;
}

int countByKind(const std::vector<LaserContour>& contours, ContourKind kind)
{
    int n = 0;
    for (const auto& c : contours)
        if (static_cast<ContourKind>(c.contourType) == kind)
            ++n;
    return n;
}

/// Assert a perforated plate (holeCount holes) extracts to holeCount+1 contours:
/// 1 outer boundary, holeCount inner holes, holes ordered before the outer ring.
int verifyPerforatedPlate(const TopoDS_Shape& plate, int holeCount,
                          ExtractionStrategy strategy, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = strategy;
    params.machiningBeamDirection = gp_Dir(0.0, 0.0, -1.0);

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(plate, params, &classification);

    const int expected = holeCount + 1;
    if (static_cast<int>(contours.size()) != expected)
        return fail(label + QStringLiteral(": expected %1 contours, got %2")
                        .arg(expected).arg(contours.size()));
    if (countByKind(contours, ContourKind::OuterBoundary) != 1)
        return fail(label + QStringLiteral(": expected exactly 1 outer boundary"));
    if (countByKind(contours, ContourKind::InnerHole) != holeCount)
        return fail(label + QStringLiteral(": expected %1 inner holes, got %2")
                        .arg(holeCount).arg(countByKind(contours, ContourKind::InnerHole)));

    // Holes first, outer last: every contour before the last must be a hole,
    // and the last must be the outer boundary.
    for (std::size_t i = 0; i + 1 < contours.size(); ++i)
        if (static_cast<ContourKind>(contours[i].contourType) != ContourKind::InnerHole)
            return fail(label + QStringLiteral(": non-hole contour before the outer ring"));
    if (static_cast<ContourKind>(contours.back().contourType) != ContourKind::OuterBoundary)
        return fail(label + QStringLiteral(": last contour is not the outer boundary"));

    return 0;
}

int verifyTubeRegression(const TopoDS_Shape& cylinder, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::TubeClassification;

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(cylinder, params, &classification);
    if (contours.empty())
        return fail(label + QStringLiteral(": tube path produced no contours"));
    if (!classification.hasOuter() || !classification.hasCrossSection())
        return fail(label + QStringLiteral(": tube classification incomplete"));
    for (const auto& c : contours)
        if (static_cast<ContourKind>(c.contourType) != ContourKind::TubeCrossSection)
            return fail(label + QStringLiteral(": tube contour mislabeled"));

    const auto* outer = classification.outerGroup();
    if (!outer || outer->faces.empty())
        return fail(label + QStringLiteral(": missing outer face group for manual regression"));
    auto manualContours = LaserToolpathBuilder::extractContoursFromFaces(
        cylinder, outer->faces, gp_Dir(0.0, 0.0, -1.0), params);
    if (manualContours.size() != contours.size())
        return fail(label + QStringLiteral(": manual face-group boundary differs from tube extraction"));
    for (const auto& c : manualContours)
        if (static_cast<ContourKind>(c.contourType) != ContourKind::TubeCrossSection)
            return fail(label + QStringLiteral(": manual tube boundary mislabeled"));
    return 0;
}

int verifyPipelineChain()
{
    lcnc::cam::CamDataManager data;
    using lcnc::cam::CamPipelineStage;
    data.commitPipelineStage(CamPipelineStage::FaceSeparation);
    const auto face = data.pipelineStageState(CamPipelineStage::FaceSeparation).revision;
    data.commitPipelineStage(CamPipelineStage::ContourExtraction, face);
    const auto contour = data.pipelineStageState(CamPipelineStage::ContourExtraction).revision;
    data.commitPipelineStage(CamPipelineStage::PointDiscretization, contour);
    const auto points = data.pipelineStageState(CamPipelineStage::PointDiscretization).revision;
    data.commitPipelineStage(CamPipelineStage::GeometricToolpath, points);
    const auto path = data.pipelineStageState(CamPipelineStage::GeometricToolpath).revision;
    data.commitPipelineStage(CamPipelineStage::MachineSolve, path);
    if (!data.hasCompletePipelineChain())
        return fail(QStringLiteral("pipeline chain should be valid"));

    auto invalid = data.pipelineStageState(CamPipelineStage::MachineSolve);
    invalid.inputRevision = path + 1;
    data.restorePipelineStageState(CamPipelineStage::MachineSolve, invalid);
    if (data.hasCompletePipelineChain())
        return fail(QStringLiteral("pipeline chain accepted mismatched machine-solve input"));
    return 0;
}

} // namespace

int main()
{
    const int holeCount = 3;
    const TopoDS_Shape plate = makePerforatedPlate(holeCount);

    if (int rc = verifyPerforatedPlate(plate, holeCount,
                                       ExtractionStrategy::PlanarFaceWires,
                                       QStringLiteral("PlanarFaceWires")))
        return rc;

    if (int rc = verifyPerforatedPlate(plate, holeCount,
                                       ExtractionStrategy::Auto,
                                       QStringLiteral("Auto")))
        return rc;

    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 20.0, 80.0).Shape();
    if (int rc = verifyTubeRegression(cylinder, QStringLiteral("TubeClassification")))
        return rc;
    if (int rc = verifyPipelineChain())
        return rc;

    QTextStream(stderr) << "contour_extraction_test: ok\n";
    return 0;
}
