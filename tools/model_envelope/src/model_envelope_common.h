#pragma once

#include <QJsonObject>
#include <QString>
#include <QVector>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lcnc::tools::model_envelope {

struct Point3d
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct TriangleMesh
{
    std::vector<Point3d> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

struct MachineBodyMesh
{
    QString name;
    QString axisName;
    TriangleMesh mesh;
};

struct ImportMetrics
{
    qint64 stepImportMs{0};
    qint64 tessellationMs{0};
    std::uint64_t sourceVertices{0};
    std::uint64_t sourceTriangles{0};
};

struct MeshValidationMetrics
{
    bool nonEmpty{false};
    bool watertight{false};
    bool twoManifold{false};
    bool finite{false};
    std::uint64_t boundaryEdges{0};
    std::uint64_t nonManifoldEdges{0};
    std::uint64_t sampledSourcePoints{0};
    std::uint64_t outsideSourcePoints{0};
    double minimumSampledMarginMm{0.0};
    double meanSampledSourceDistanceMm{0.0};
    double maximumSampledSourceDistanceMm{0.0};
    double meanSampledOutputDistanceMm{0.0};
    double maximumSampledOutputDistanceMm{0.0};
};

bool loadMachineBodyMeshes(const QString& stepPath,
                           double linearDeflectionMm,
                           double angularDeflectionRad,
                           QVector<MachineBodyMesh>* bodies,
                           ImportMetrics* metrics,
                           QString* errorMessage);

MeshValidationMetrics validateEnvelope(const TriangleMesh& source,
                                       const TriangleMesh& envelope,
                                       std::size_t maximumSamples = 20'000);

bool writeBinaryPly(const QString& path,
                    const TriangleMesh& mesh,
                    QString* errorMessage);

QString sanitizedBodyFileName(const QString& axisName, const QString& name);
QJsonObject importMetricsJson(const ImportMetrics& metrics);
QJsonObject validationMetricsJson(const MeshValidationMetrics& metrics);

std::uint64_t peakWorkingSetBytes();
std::uint64_t peakPrivateBytes();

} // namespace lcnc::tools::model_envelope
