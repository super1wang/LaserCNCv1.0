#pragma once

#include "core/algorithms/cam/surface_collision_prefilter.h"

#include <QByteArray>
#include <QString>
#include <QVector>

#include <cstdint>

namespace lcnc {

struct ModelEnvelopeBodyResource
{
    QString name;
    QString axisName;
    QString parentAxis;
    QString meshPath;
    QByteArray meshSha256;
    double alphaMm{0.0};
    double offsetMm{0.0};
    std::uint64_t vertexCount{0};
    std::uint64_t triangleCount{0};
};

struct ModelEnvelopeAsset
{
    static constexpr int kCurrentFormatVersion = 1;

    QString schema{QStringLiteral("lcnc.model-envelope/v1")};
    int formatVersion{kCurrentFormatVersion};
    QString generator;
    QString generatorVersion;
    QByteArray sourceModelSha256;
    double deflectionMm{0.0};
    double angleRad{0.0};
    double alphaMm{0.0};
    double offsetMm{0.0};
    bool collisionConservative{false};
    QVector<ModelEnvelopeBodyResource> bodies;
    QString simplifiedStepPath;
    QByteArray simplifiedStepSha256;
    QString manifestPath;
    QByteArray manifestSha256;

    static bool loadAndValidate(const QString& manifestPath,
                                const QString& expectedSourceModelPath,
                                ModelEnvelopeAsset* asset,
                                QString* errorMessage = nullptr);

    static bool loadBodyMesh(const ModelEnvelopeAsset& asset,
                             const ModelEnvelopeBodyResource& body,
                             cam_algo::SurfaceTriangleSoup* soup,
                             QString* errorMessage = nullptr);
};

} // namespace lcnc
