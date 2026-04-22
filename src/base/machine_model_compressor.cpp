#include "base/machine_model_compressor.h"

#include "base/task_progress.h"

#include <QSet>
#include <QVector>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kClusterTolerance = 0.5;

struct ShapeNode {
    TopoDS_Shape shape;
    Bnd_Box box;
};

Bnd_Box shapeBox(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    if (!shape.IsNull())
        BRepBndLib::Add(shape, box);
    return box;
}

Bnd_Box mergeBoxes(const QList<TopoDS_Shape>& shapes)
{
    Bnd_Box merged;
    for (const TopoDS_Shape& shape : shapes) {
        if (!shape.IsNull())
            BRepBndLib::Add(shape, merged);
    }
    return merged;
}

TopoDS_Shape makeBoundingBoxProxy(const QList<TopoDS_Shape>& shapes)
{
    const Bnd_Box box = mergeBoxes(shapes);
    if (box.IsVoid())
        return {};

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    constexpr double kMinThickness = 1.0;
    if ((xmax - xmin) < kMinThickness) {
        const double center = 0.5 * (xmin + xmax);
        xmin = center - 0.5 * kMinThickness;
        xmax = center + 0.5 * kMinThickness;
    }
    if ((ymax - ymin) < kMinThickness) {
        const double center = 0.5 * (ymin + ymax);
        ymin = center - 0.5 * kMinThickness;
        ymax = center + 0.5 * kMinThickness;
    }
    if ((zmax - zmin) < kMinThickness) {
        const double center = 0.5 * (zmin + zmax);
        zmin = center - 0.5 * kMinThickness;
        zmax = center + 0.5 * kMinThickness;
    }

    return BRepPrimAPI_MakeBox(gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymax, zmax)).Shape();
}

double boxDiagonal(const Bnd_Box& box)
{
    if (box.IsVoid())
        return 0.0;

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dz = zmax - zmin;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double boxVolume(const Bnd_Box& box)
{
    if (box.IsVoid())
        return 0.0;

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return std::max(0.0, xmax - xmin)
         * std::max(0.0, ymax - ymin)
         * std::max(0.0, zmax - zmin);
}

bool boxesOverlap(const Bnd_Box& lhs, const Bnd_Box& rhs)
{
    if (lhs.IsVoid() || rhs.IsVoid())
        return false;

    Bnd_Box a = lhs;
    Bnd_Box b = rhs;
    a.Enlarge(kClusterTolerance);
    b.Enlarge(kClusterTolerance);
    return !a.IsOut(b);
}

QList<ShapeNode> buildNodes(const QList<TopoDS_Shape>& shapes)
{
    QList<ShapeNode> nodes;
    for (const TopoDS_Shape& shape : shapes) {
        if (shape.IsNull())
            continue;
        nodes.append({shape, shapeBox(shape)});
    }
    return nodes;
}

QList<QList<ShapeNode>> clusterNodes(const QList<ShapeNode>& nodes)
{
    QList<QList<ShapeNode>> clusters;
    QVector<bool> visited(nodes.size(), false);

    for (int i = 0; i < nodes.size(); ++i) {
        if (visited[i])
            continue;

        QList<ShapeNode> cluster;
        QVector<int> queue;
        queue.append(i);
        visited[i] = true;

        for (int cursor = 0; cursor < queue.size(); ++cursor) {
            const int index = queue[cursor];
            cluster.append(nodes[index]);

            for (int other = 0; other < nodes.size(); ++other) {
                if (visited[other])
                    continue;
                if (!boxesOverlap(nodes[index].box, nodes[other].box))
                    continue;

                visited[other] = true;
                queue.append(other);
            }
        }

        clusters.append(cluster);
    }

    return clusters;
}

QList<TopoDS_Shape> clusterShapes(const QList<ShapeNode>& cluster)
{
    QList<TopoDS_Shape> shapes;
    shapes.reserve(cluster.size());
    for (const ShapeNode& node : cluster)
        shapes.append(node.shape);
    return shapes;
}

TopoDS_Shape buildCompoundShape(const QList<TopoDS_Shape>& shapes)
{
    if (shapes.isEmpty())
        return {};
    if (shapes.size() == 1)
        return shapes.front();

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const TopoDS_Shape& shape : shapes) {
        if (!shape.IsNull())
            builder.Add(compound, shape);
    }
    return compound;
}

QList<TopoDS_Solid> collectSolids(const TopoDS_Shape& shape)
{
    QList<TopoDS_Solid> solids;
    if (shape.IsNull())
        return solids;

    if (shape.ShapeType() == TopAbs_SOLID) {
        solids.append(TopoDS::Solid(shape));
        return solids;
    }

    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
        solids.append(TopoDS::Solid(exp.Current()));
    return solids;
}

QList<TopoDS_Shell> collectShells(const TopoDS_Shape& shape)
{
    QList<TopoDS_Shell> shells;
    if (shape.IsNull())
        return shells;

    if (shape.ShapeType() == TopAbs_SHELL) {
        shells.append(TopoDS::Shell(shape));
        return shells;
    }

    for (TopExp_Explorer exp(shape, TopAbs_SHELL); exp.More(); exp.Next())
        shells.append(TopoDS::Shell(exp.Current()));
    return shells;
}

TopoDS_Shape trySewShapes(const QList<TopoDS_Shape>& shapes)
{
    if (shapes.isEmpty())
        return {};

    BRepBuilderAPI_Sewing sewing(1.0e-2);
    for (const TopoDS_Shape& shape : shapes) {
        if (!shape.IsNull())
            sewing.Add(shape);
    }

    sewing.Perform();
    return sewing.SewedShape();
}

TopoDS_Shape extractExteriorShellShape(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return {};

    const QList<TopoDS_Solid> solids = collectSolids(shape);
    QList<TopoDS_Shape> shellShapes;

    if (!solids.isEmpty()) {
        shellShapes.reserve(solids.size());
        for (const TopoDS_Solid& solid : solids) {
            const QList<TopoDS_Shell> shells = collectShells(solid);
            if (shells.isEmpty()) {
                shellShapes.append(solid);
                continue;
            }

            auto bestShell = std::max_element(shells.cbegin(), shells.cend(),
                [](const TopoDS_Shell& lhs, const TopoDS_Shell& rhs) {
                    return boxDiagonal(shapeBox(lhs)) < boxDiagonal(shapeBox(rhs));
                });
            shellShapes.append(*bestShell);
        }

        return buildCompoundShape(shellShapes);
    }

    const QList<TopoDS_Shell> shells = collectShells(shape);
    if (!shells.isEmpty()) {
        shellShapes.reserve(shells.size());
        for (const TopoDS_Shell& shell : shells)
            shellShapes.append(shell);
        return buildCompoundShape(shellShapes);
    }

    return shape;
}

QString strategyDisplayName(MachineModelCompressor::Strategy strategy)
{
    switch (strategy) {
    case MachineModelCompressor::Strategy::FilledSolid:
        return QStringLiteral("实体填充并集");
    case MachineModelCompressor::Strategy::ExteriorShell:
        return QStringLiteral("外壳抽取");
    case MachineModelCompressor::Strategy::SewingShell:
        return QStringLiteral("缝合壳体");
    case MachineModelCompressor::Strategy::BoundingBoxProxy:
        return QStringLiteral("平衡外观代理");
    }

    return QStringLiteral("压缩");
}

int workUnitsForCluster(int clusterSize, MachineModelCompressor::Strategy strategy)
{
    if (clusterSize <= 1)
        return 1;

    if (strategy == MachineModelCompressor::Strategy::SewingShell
        || strategy == MachineModelCompressor::Strategy::BoundingBoxProxy)
        return 1;

    return std::max(1, clusterSize - 1);
}

TopoDS_Shape fuseClusterShapes(const QList<TopoDS_Shape>& shapes,
                              TaskProgress* progress,
                              int& completedWork)
{
    if (shapes.isEmpty())
        return {};

    TopoDS_Shape fused = shapes.front();
    if (fused.IsNull())
        return {};

    for (int i = 1; i < shapes.size(); ++i) {
        BRepAlgoAPI_Fuse fuse(fused, shapes[i]);
        fuse.Build();

        ++completedWork;
        if (progress)
            progress->setValue(completedWork);

        if (!fuse.IsDone() || fuse.Shape().IsNull())
            return {};

        fused = fuse.Shape();
        BRepTools::Clean(fused);
    }

    return fused;
}

TopoDS_Shape compressClusterShapes(const QList<TopoDS_Shape>& shapes,
                                   MachineModelCompressor::Strategy strategy,
                                   TaskProgress* progress,
                                   int& completedWork)
{
    if (shapes.isEmpty())
        return {};

    if (shapes.size() == 1) {
        ++completedWork;
        if (progress)
            progress->setValue(completedWork);
        return shapes.front();
    }

    if (strategy == MachineModelCompressor::Strategy::SewingShell) {
        const TopoDS_Shape sewed = trySewShapes(shapes);
        ++completedWork;
        if (progress)
            progress->setValue(completedWork);

        if (!sewed.IsNull())
            return extractExteriorShellShape(sewed);

        TopoDS_Shape fused = fuseClusterShapes(shapes, progress, completedWork);
        if (!fused.IsNull())
            return extractExteriorShellShape(fused);

        return extractExteriorShellShape(buildCompoundShape(shapes));
    }

    TopoDS_Shape fused = fuseClusterShapes(shapes, progress, completedWork);
    if (fused.IsNull()) {
        const TopoDS_Shape sewed = trySewShapes(shapes);
        if (!sewed.IsNull())
            fused = sewed;
        else
            fused = buildCompoundShape(shapes);
    }

    if (strategy == MachineModelCompressor::Strategy::ExteriorShell)
        return extractExteriorShellShape(fused);

    return fused;
}

TopoDS_Shape makeBalancedProxy(const QList<ShapeNode>& cluster,
                              TaskProgress* progress,
                              int& completedWork)
{
    if (cluster.isEmpty())
        return {};

    if (cluster.size() == 1) {
        ++completedWork;
        if (progress)
            progress->setValue(completedWork);
        return extractExteriorShellShape(cluster.front().shape);
    }

    const QList<TopoDS_Shape> allShapes = clusterShapes(cluster);
    const Bnd_Box clusterBox = mergeBoxes(allShapes);
    const double clusterDiag = boxDiagonal(clusterBox);
    const double clusterVolume = boxVolume(clusterBox);

    struct RankedNode {
        int index{-1};
        double volume{0.0};
        double diagonal{0.0};
    };

    QVector<RankedNode> rankedNodes;
    rankedNodes.reserve(cluster.size());
    for (int index = 0; index < cluster.size(); ++index) {
        rankedNodes.append({index,
                            boxVolume(cluster[index].box),
                            boxDiagonal(cluster[index].box)});
    }

    std::sort(rankedNodes.begin(), rankedNodes.end(),
              [](const RankedNode& lhs, const RankedNode& rhs) {
                  if (lhs.volume == rhs.volume)
                      return lhs.diagonal > rhs.diagonal;
                  return lhs.volume > rhs.volume;
              });

    constexpr int kMinKeepCount = 2;
    constexpr int kMaxKeepCount = 8;
    constexpr int kMaxProxyBoxes = 24;
    constexpr double kKeepCoverageRatio = 0.78;
    constexpr double kKeepDiagonalRatio = 0.32;

    QSet<int> keepIndices;
    double coveredVolume = 0.0;
    for (const RankedNode& node : rankedNodes) {
        const bool mustKeepByCount = keepIndices.size() < kMinKeepCount;
        const bool keepByCoverage = keepIndices.size() < kMaxKeepCount
            && coveredVolume < clusterVolume * kKeepCoverageRatio;
        const bool keepBySpan = node.diagonal >= clusterDiag * kKeepDiagonalRatio;

        if (!mustKeepByCount && !keepByCoverage && !keepBySpan)
            continue;

        keepIndices.insert(node.index);
        coveredVolume += node.volume;
    }

    if (keepIndices.isEmpty() && !rankedNodes.isEmpty())
        keepIndices.insert(rankedNodes.front().index);

    QList<TopoDS_Shape> outputs;
    QList<ShapeNode> proxyNodes;
    for (int index = 0; index < cluster.size(); ++index) {
        if (keepIndices.contains(index))
            outputs.append(extractExteriorShellShape(cluster[index].shape));
        else
            proxyNodes.append(cluster[index]);
    }

    if (!proxyNodes.isEmpty()) {
        if (proxyNodes.size() <= kMaxProxyBoxes) {
            for (const ShapeNode& node : proxyNodes)
                outputs.append(makeBoundingBoxProxy({node.shape}));
        } else {
            const QList<QList<ShapeNode>> proxyClusters = clusterNodes(proxyNodes);
            for (const QList<ShapeNode>& proxyCluster : proxyClusters)
                outputs.append(makeBoundingBoxProxy(clusterShapes(proxyCluster)));
        }
    }

    ++completedWork;
    if (progress)
        progress->setValue(completedWork);

    return buildCompoundShape(outputs);
}

} // namespace

MachineModelCompressor::Result MachineModelCompressor::compressGroups(
    const QList<GroupInput>& groups,
    const Options& options,
    TaskProgress* progress)
{
    Result result;
    if (groups.isEmpty()) {
        result.error = QStringLiteral("没有可压缩的机台形体。");
        return result;
    }

    struct PreparedGroup {
        QString axisName;
        QString displayName;
        QList<QList<ShapeNode>> clusters;
    };

    QList<PreparedGroup> preparedGroups;
    int totalWorkUnits = 0;
    for (const GroupInput& group : groups) {
        const QList<ShapeNode> nodes = buildNodes(group.shapes);
        if (nodes.isEmpty())
            continue;

        PreparedGroup prepared;
        prepared.axisName = group.axisName;
        prepared.displayName = group.displayName;
        prepared.clusters = clusterNodes(nodes);
        preparedGroups.append(prepared);

        for (const QList<ShapeNode>& cluster : prepared.clusters)
            totalWorkUnits += workUnitsForCluster(static_cast<int>(cluster.size()), options.strategy);
    }

    if (preparedGroups.isEmpty()) {
        result.error = QStringLiteral("没有可压缩的有效机台形体。");
        return result;
    }

    if (progress) {
        progress->setRange(0, std::max(1, totalWorkUnits));
        progress->setStepName(QStringLiteral("准备%1...").arg(strategyDisplayName(options.strategy)));
    }

    int completedWork = 0;
    for (const PreparedGroup& group : preparedGroups) {
        if (progress && progress->isAbortRequested()) {
            result.aborted = true;
            result.error = QStringLiteral("机台压缩已中止。");
            return result;
        }

        if (progress)
            progress->setStepName(QStringLiteral("%1: %2...")
                                      .arg(strategyDisplayName(options.strategy), group.displayName));

        QList<TopoDS_Shape> outputs;
        for (const QList<ShapeNode>& cluster : group.clusters) {
            if (progress && progress->isAbortRequested()) {
                result.aborted = true;
                result.error = QStringLiteral("机台压缩已中止。");
                return result;
            }

            TopoDS_Shape clusterShape;
            if (options.strategy == MachineModelCompressor::Strategy::BoundingBoxProxy) {
                clusterShape = makeBalancedProxy(cluster, progress, completedWork);
            } else {
                const QList<TopoDS_Shape> clusterInput = clusterShapes(cluster);
                clusterShape = compressClusterShapes(
                    clusterInput,
                    options.strategy,
                    progress,
                    completedWork);
            }
            if (!clusterShape.IsNull())
                outputs.append(clusterShape);
        }

        const TopoDS_Shape groupShape = buildCompoundShape(outputs);
        if (!groupShape.IsNull())
            result.groups.append({group.axisName, group.displayName, groupShape});
    }

    if (result.groups.isEmpty() && !result.aborted)
        result.error = QStringLiteral("机台压缩未生成任何结果。");

    return result;
}