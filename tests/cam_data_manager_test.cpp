#include "core/project/cam/cam_data_manager.h"

#include <QCoreApplication>
#include <QTextStream>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    using namespace lcnc::cam;

    CamDataManager manager;
    std::uint64_t upstreamRevision = 0;
    for (int value = static_cast<int>(CamPipelineStage::FaceSeparation);
         value < static_cast<int>(CamPipelineStage::Count); ++value) {
        const auto stage = static_cast<CamPipelineStage>(value);
        manager.commitPipelineStage(stage, upstreamRevision);
        upstreamRevision = manager.pipelineStageState(stage).revision;
    }
    if (!manager.hasCompletePipelineChain())
        return fail(QStringLiteral("Committed CAM pipeline was not complete"));

    manager.invalidatePipelineAfter(CamPipelineStage::ContourExtraction,
                                    QStringLiteral("test invalidation"));
    if (!manager.pipelineStageState(CamPipelineStage::PointDiscretization).dirty
        || manager.hasCompletePipelineChain()) {
        return fail(QStringLiteral("CAM pipeline invalidation did not dirty downstream stages"));
    }
    manager.failPipelineStage(CamPipelineStage::ContourExtraction,
                              QStringLiteral("expected failure"));
    if (manager.pipelineStageState(CamPipelineStage::ContourExtraction).failureReason
        != QStringLiteral("expected failure")) {
        return fail(QStringLiteral("CAM pipeline failure reason was not retained"));
    }

    manager.setGenerationParamsDirty(true);
    manager.generationParams().leadInLength = 7.5;
    if (!manager.generationParamsDirty()
        || manager.appliedGenerationParams().leadInLength == 7.5) {
        return fail(QStringLiteral("Pending/applied CAM generation parameters were mixed"));
    }

    LaserContour first;
    first.signature = 11;
    LaserContour second;
    second.signature = 22;
    manager.toolpath().contours().push_back(first);
    manager.toolpath().contours().push_back(second);
    manager.ensureContourIds();
    const ContourId firstId = manager.toolpath().contour(0).contourId;
    const ContourId secondId = manager.toolpath().contour(1).contourId;
    if (firstId == 0 || secondId == 0 || firstId == secondId)
        return fail(QStringLiteral("CAM contour IDs were not allocated"));
    manager.commitToolpathStates();
    manager.clearToolpath(false);

    std::swap(first.signature, second.signature);
    manager.toolpath().contours().push_back(first);
    manager.toolpath().contours().push_back(second);
    manager.ensureContourIds();
    if (manager.toolpath().contour(0).contourId != secondId
        || manager.toolpath().contour(1).contourId != firstId) {
        return fail(QStringLiteral("CAM contour IDs were not stable by signature"));
    }

    manager.ensureToolpathLayers();
    const std::uint64_t extraLayer =
        manager.addLayer(QStringLiteral("Finish"), QColor(10, 20, 30));
    if (extraLayer == 0
        || !manager.updateToolpathLayer(extraLayer, QStringLiteral("Finish2"),
                                        QColor(30, 20, 10), QStringLiteral("Default"))
        || !manager.setToolpathLayerEnabled(extraLayer, false)
        || !manager.removeLayer(extraLayer)) {
        return fail(QStringLiteral("CAM layer add/update/remove regression"));
    }

    CamDataManager layerManager;
    LaserContour layerContourA;
    layerContourA.signature = 101;
    LaserContour layerContourB;
    layerContourB.signature = 102;
    layerManager.toolpath().contours().push_back(layerContourA);
    layerManager.toolpath().contours().push_back(layerContourB);
    // Layer normalization is the single public adoption pass: it must assign
    // contour IDs itself rather than requiring an immediate duplicate scan.
    layerManager.ensureToolpathLayers();
    const ContourId layerContourAId = layerManager.toolpath().contour(0).contourId;
    const ContourId layerContourBId = layerManager.toolpath().contour(1).contourId;
    const std::uint64_t sourceLayer = layerManager.toolpath().contour(0).layerId;
    const std::uint64_t destinationLayer = layerManager.addLayer(QStringLiteral("Delete me"));
    if (sourceLayer == 0 || destinationLayer == 0
        || !layerManager.assignContoursToLayer({layerContourAId, layerContourBId}, destinationLayer)
        || layerManager.toolpath().contour(0).layerId != destinationLayer
        || layerManager.toolpath().contour(1).layerId != destinationLayer
        || layerManager.toolpathLayer(destinationLayer)->contourIds.size() != 2
        || !layerManager.removeLayerWithContours(destinationLayer)
        || layerManager.toolpath().contourCount() != 0
        || layerManager.toolpathLayer(destinationLayer) != nullptr
        || layerManager.toolpathLayer(sourceLayer) == nullptr) {
        return fail(QStringLiteral("CAM layer bulk move/delete regression"));
    }
    if (!layerManager.removeLayerWithContours(sourceLayer)
        || layerManager.toolpath().layers().size() != 1
        || layerManager.toolpath().layers().front().layerId != sourceLayer) {
        return fail(QStringLiteral("CAM layer deletion did not retain one empty layer"));
    }

    manager.clearToolpath();
    if (manager.hasToolpath() || !manager.isDirty())
        return fail(QStringLiteral("CAM clear did not remove toolpath and mark project dirty (hasToolpath=%1, dirty=%2)")
                    .arg(manager.hasToolpath()).arg(manager.isDirty()));

    return 0;
}
