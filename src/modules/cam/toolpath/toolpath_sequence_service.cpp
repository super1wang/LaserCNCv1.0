#include "modules/cam/toolpath/toolpath_sequence_service.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/cam/layer_container.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QHash>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lcnc::cam {

std::uint64_t ToolpathSequenceService::computeToolpathRevision(
    const LaserToolpath& toolpath,
    bool generationParamsDirty,
    bool pipelineComplete)
{
    std::uint64_t revision = 1469598103934665603ull;
    auto mix = [&revision](std::uint64_t value) {
        revision ^= value + 0x9e3779b97f4a7c15ull + (revision << 6) + (revision >> 2);
    };
    auto mixRounded = [&mix](double value) {
        if (std::isnan(value)) {
            mix(0x7ff8000000000000ull);
            return;
        }
        if (!std::isfinite(value)) {
            mix(value > 0.0 ? 0x7ff0000000000000ull : 0xfff0000000000000ull);
            return;
        }
        constexpr double kLimit = static_cast<double>(std::numeric_limits<std::int64_t>::max());
        const double scaled = value * 1000.0;
        if (scaled >= kLimit) {
            mix(0x7fffffffffffffffull);
        } else if (scaled <= -kLimit) {
            mix(0x8000000000000000ull);
        } else {
            mix(static_cast<std::uint64_t>(std::llround(scaled)));
        }
    };
    auto mixString = [&mix](const QString& value) {
        const QByteArray bytes = value.toUtf8();
        mix(static_cast<std::uint64_t>(bytes.size()));
        for (const char ch : bytes)
            mix(static_cast<unsigned char>(ch));
    };

    mix(static_cast<std::uint64_t>(toolpath.contourCount()));
    mixRounded(toolpath.globalLeadInLength());
    mixRounded(toolpath.globalCuttingOffsetMm());
    mixRounded(toolpath.globalRapidOffsetMm());
    mix(generationParamsDirty ? 1ull : 0ull);
    mix(pipelineComplete ? 1ull : 0ull);
    for (const LaserContour& contour : toolpath.contours()) {
        mix(contour.contourId);
        mix(contour.layerId);
        mix(contour.signature);
        mix(contour.enabled ? 1ull : 0ull);
        mix(contour.leadIn.valid ? 1ull : 0ull);
        mix(contour.leadInSolution.valid ? 1ull : 0ull);
        mixString(contour.leadInSolution.error);
        mix(contour.needsRecalculation ? 1ull : 0ull);
        mix(contour.geometrySamplingComplete ? 1ull : 0ull);
        mixRounded(contour.appliedParams.leadInLength);
        mixRounded(contour.appliedParams.deflection);
        mixRounded(contour.appliedParams.cuttingOffsetMm);
        mixRounded(contour.appliedParams.rapidOffsetMm);
        mixRounded(contour.pendingParams.leadInLength);
        mixRounded(contour.pendingParams.deflection);
        mixRounded(contour.pendingParams.cuttingOffsetMm);
        mixRounded(contour.pendingParams.rapidOffsetMm);
        mix(static_cast<std::uint64_t>(contour.dirtyStages));
        mixString(contour.name);
        mixString(contour.workpieceEntry);
        if (contour.leadInSolution.valid) {
            const ToolpathPoint& lead = contour.leadInSolution.point;
            mixRounded(lead.position.X());
            mixRounded(lead.position.Y());
            mixRounded(lead.position.Z());
            mixRounded(lead.machineCoord.x);
            mixRounded(lead.machineCoord.y);
            mixRounded(lead.machineCoord.z);
            mixRounded(lead.machineCoord.r1);
            mixRounded(lead.machineCoord.r2);
        }
        mix(static_cast<std::uint64_t>(contour.points.size()));
        if (!contour.points.empty()) {
            const ToolpathPoint& first = contour.points.front();
            const ToolpathPoint& last = contour.points.back();
            mixRounded(first.position.X());
            mixRounded(first.position.Y());
            mixRounded(last.position.X());
            mixRounded(last.position.Y());
        }
        for (const ToolpathPoint& point : contour.points) {
            mix(static_cast<std::uint64_t>(point.sourceEdgeIndex + 1));
            mixRounded(point.param);
            mix(static_cast<std::uint64_t>(point.departureSourceEdgeIndex + 1));
            mixRounded(point.departureSourceParameter);
            mix(point.semanticHardBarrier ? 1ull : 0ull);
            mixRounded(point.position.X());
            mixRounded(point.position.Y());
            mixRounded(point.position.Z());
            mixRounded(point.normal.X());
            mixRounded(point.normal.Y());
            mixRounded(point.normal.Z());
            mixRounded(point.tangent.X());
            mixRounded(point.tangent.Y());
            mixRounded(point.tangent.Z());
            mixRounded(point.machineCoord.x);
            mixRounded(point.machineCoord.y);
            mixRounded(point.machineCoord.z);
            mixRounded(point.machineCoord.r1);
            mixRounded(point.machineCoord.r2);
            mixString(point.machineCoord.r1Name);
            mixString(point.machineCoord.r2Name);
            mix(point.machineCoord.valid ? 1ull : 0ull);
        }
    }
    for (const ToolpathLayer& layer : toolpath.layers()) {
        mix(layer.layerId);
        mix(layer.signature);
        mix(layer.enabled ? 1ull : 0ull);
        mixString(layer.name);
        mixString(layer.toolName);
        mixString(layer.compensationIndex);
        mix(static_cast<std::uint64_t>(layer.manualOrder));
        mix(static_cast<std::uint64_t>(layer.contourIds.size()));
        for (const std::uint64_t contourId : layer.contourIds)
            mix(contourId);
        QVector<std::uint64_t> included(layer.includedContours.cbegin(),
                                        layer.includedContours.cend());
        std::sort(included.begin(), included.end());
        mix(static_cast<std::uint64_t>(included.size()));
        for (const std::uint64_t contourId : included)
            mix(contourId);
    }
    return revision;
}

ContourSequenceSnapshot ToolpathSequenceService::buildSequenceSnapshot(
    const ToolpathExportSnapshot& snapshot,
    const LayerContainer* layers)
{
    ContourSequenceSnapshot result;
    result.strategy = layers ? layers->sortStrategy()
                             : CuttingPlanSortStrategy::LayerThenContour;
    if (snapshot.contours.isEmpty())
        return result;

    QHash<ContourId, int> manualRank;
    if (layers && result.strategy == CuttingPlanSortStrategy::Manual) {
        const auto& manual = layers->manualContourOrder();
        manualRank.reserve(manual.size());
        for (int index = 0; index < manual.size(); ++index)
            manualRank.insert(manual.at(index), index);
    }

    struct Item {
        const ToolpathExportContour* contour{nullptr};
        const ToolpathLayer* layer{nullptr};
    };
    QVector<Item> items;
    items.reserve(snapshot.contours.size());
    for (const auto& contour : snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;
        const ToolpathLayer* layer = layers ? layers->layer(contour.layerId) : nullptr;
        if (layer && (!layer->enabled || (!layer->includedContours.isEmpty()
            && !layer->includedContours.contains(contour.contourId)))) {
            continue;
        }
        if (result.strategy == CuttingPlanSortStrategy::Manual
            && !manualRank.contains(contour.contourId)) {
            continue;
        }
        items.append({&contour, layer});
    }

    std::stable_sort(items.begin(), items.end(),
                     [strategy = result.strategy, &manualRank](const Item& a, const Item& b) {
        if (strategy == CuttingPlanSortStrategy::CamOrder)
            return false;
        if (strategy == CuttingPlanSortStrategy::Manual) {
            const int ar = manualRank.value(a.contour->contourId,
                                            std::numeric_limits<int>::max());
            const int br = manualRank.value(b.contour->contourId,
                                            std::numeric_limits<int>::max());
            return ar == br ? a.contour->contourId < b.contour->contourId : ar < br;
        }
        if (strategy == CuttingPlanSortStrategy::ToolThenLayer) {
            const QString at = a.layer ? a.layer->toolName : QString();
            const QString bt = b.layer ? b.layer->toolName : QString();
            const int compared = QString::localeAwareCompare(at, bt);
            if (compared != 0)
                return compared < 0;
        }
        if (a.contour->layerId != b.contour->layerId)
            return a.contour->layerId < b.contour->layerId;
        return a.contour->contourId < b.contour->contourId;
    });

    result.orderedContourIds.reserve(items.size());
    std::uint64_t revision = snapshot.revision
        ^ (static_cast<std::uint64_t>(result.strategy) << 56);
    for (const Item& item : items) {
        result.orderedContourIds.append(item.contour->contourId);
        revision ^= item.contour->contourId + 0x9e3779b97f4a7c15ull
            + (revision << 6) + (revision >> 2);
    }
    result.revision = revision;
    return result;
}

QVector<ContourId> ToolpathSequenceService::planAutomaticOrder(
    const LaserToolpath& toolpath,
    const LayerContainer& layers,
    const MachineKinematics* kinematics,
    AutoSortAxis axis,
    QString* errorMessage)
{
    QVector<ContourEndpoints> inputs;
    inputs.reserve(toolpath.contourCount());
    for (const LaserContour& contour : toolpath.contours()) {
        if (!contour.enabled || contour.points.empty())
            continue;
        const ToolpathLayer* layer = layers.layer(contour.layerId);
        if (layer && (!layer->enabled
            || (!layer->includedContours.isEmpty()
                && !layer->includedContours.contains(contour.contourId)))) {
            continue;
        }
        ContourEndpoints endpoint;
        endpoint.id = contour.contourId;
        gp_Pnt start = contour.leadInSolution.valid
            ? contour.leadInSolution.point.position : contour.points.front().position;
        gp_Pnt end = contour.points.back().position;
        if (kinematics) {
            const gp_Trsf homeWpc = kinematics->computeWpcTransformHome(
                contour.workpieceEntry);
            start.Transform(homeWpc);
            end.Transform(homeWpc);
        }
        endpoint.sx = start.X(); endpoint.sy = start.Y(); endpoint.sz = start.Z();
        endpoint.ex = end.X(); endpoint.ey = end.Y(); endpoint.ez = end.Z();
        inputs.append(endpoint);
    }
    if (inputs.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QCoreApplication::translate(
                "CamModule", "There are currently no enabled profiles to participate in sorting");
        }
        return {};
    }

    ContourOrderParams params;
    switch (axis) {
    case AutoSortAxis::XPos: params.axis = PrimaryAxis::XPos; break;
    case AutoSortAxis::XNeg: params.axis = PrimaryAxis::XNeg; break;
    case AutoSortAxis::YPos: params.axis = PrimaryAxis::YPos; break;
    case AutoSortAxis::YNeg: params.axis = PrimaryAxis::YNeg; break;
    case AutoSortAxis::ZPos: params.axis = PrimaryAxis::ZPos; break;
    case AutoSortAxis::ZNeg: params.axis = PrimaryAxis::ZNeg; break;
    }
    const auto ordered = planContourOrder(inputs, params);
    if (ordered.isEmpty() && errorMessage) {
        *errorMessage = QCoreApplication::translate(
            "CamModule", "Autosort does not generate valid contour order");
    }
    return ordered;
}

} // namespace lcnc::cam
