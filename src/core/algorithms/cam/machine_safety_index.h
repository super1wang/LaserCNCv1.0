#pragma once

#include "core/algorithms/cam/surface_collision_prefilter.h"

#include <QByteArray>
#include <QPair>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>

namespace lcnc::cam_algo {

constexpr int kMachineSafetyMaximumAxes = 5;

enum class MachineSafetyIndexState : std::uint8_t
{
    Unknown = 0,
    CertifiedSafe,
    CollisionSample
};

struct MachineSafetyPose
{
    std::array<double, kMachineSafetyMaximumAxes> values{};
    std::uint8_t count{0};
};

struct MachineSafetyAxisGrid
{
    QString name;
    QString parentAxis;
    bool rotary{false};
    std::array<double, 3> direction{0.0, 0.0, 1.0};
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    double minimum{0.0};
    double maximum{0.0};
    double step{1.0};
    std::uint32_t cellCount{0};
};

struct MachineSafetyBodySummary
{
    QString name;
    QString axisName;
};

struct MachineSafetyPairSummary
{
    std::uint16_t firstBody{0};
    std::uint16_t secondBody{0};
};

struct MachineSafetyQueryResult
{
    MachineSafetyIndexState state{MachineSafetyIndexState::Unknown};
    bool inRange{false};
    bool exactPoseCertificate{false};
    std::uint64_t cellIndex{0};
    std::uint8_t refinementLevel{0};
    double clearanceLowerBoundMm{-1.0};
    int blockingPair{-1};
};

struct MachineSafetyBuildOptions
{
    QVector<MachineSafetyAxisGrid> axes;
    QVector<QPair<QString, QString>> axisPairs;
    double clearanceMm{0.5};
    int maximumExactQueries{32};
    bool useLeafExact{true};
    bool useDirectionalMotionBounds{true};
    bool useDependencyCache{true};
    bool useLeafBvhCertification{false};
    bool useSurfaceBvhCertification{false};
    double surfaceMeshDeflectionMm{0.5};
    int refinementLevels{1};
    int refinementThreads{8};
    std::uint64_t maximumCells{2'000'000};
    std::uint64_t maximumRefinedCells{4'000'000};
    /// Levels deeper than one are restricted to these production/path poses
    /// when non-empty. This makes AC-table and observed BoundaryUnknown zones
    /// receive detail without multiplying the complete configuration space.
    QVector<MachineSafetyPose> hotPoses;
    double hotRefinementRadiusCells{2.0};
    /// An atomically replaced, valid .lmsi is written after the base grid and
    /// each completed refinement level. A later build may resume from it.
    QString checkpointPath;
    bool resumeCheckpoint{true};
    /// Optional, thread-safe observer used by the standalone builder. Values
    /// are monotonic in [0, 100]; the stage is a stable protocol token.
    std::function<void(int, const QString&)> progressCallback;
};

struct MachineSafetyBuildMetrics
{
    bool resumedFromCheckpoint{false};
    int checkpointRefinementLevel{-1};
    qint64 importMs{0};
    qint64 gridMs{0};
    qint64 refinementMs{0};
    qint64 surfaceBvhCompileMs{0};
    qint64 surfaceBvhQueryMs{0};
    qint64 leafBvhQueryMs{0};
    qint64 exactMs{0};
    qint64 saveMs{0};
    qint64 totalMs{0};
    std::uint64_t totalCells{0};
    std::uint64_t certifiedSafeCells{0};
    std::uint64_t collisionSamples{0};
    std::uint64_t unknownCells{0};
    std::uint64_t refinedParentCells{0};
    std::uint64_t refinedCells{0};
    std::uint64_t refinedSafeCells{0};
    std::uint64_t refinedUnknownCells{0};
    std::uint64_t refinementBodyEvaluations{0};
    std::uint64_t refinementBodyCacheHits{0};
    std::uint64_t refinementPairEvaluations{0};
    std::uint64_t refinementPairCacheHits{0};
    std::uint64_t surfaceBvhQueries{0};
    std::uint64_t surfaceBvhCertifiedPairs{0};
    std::uint64_t leafBvhQueries{0};
    std::uint64_t leafBvhCertifiedPairs{0};
    std::uint64_t exactQueries{0};
    std::uint64_t exactFailures{0};
    std::uint64_t geometryLeaves{0};
    std::uint64_t leafCandidatePairs{0};
    std::uint64_t leafExactQueries{0};
    std::uint64_t wholeShapeExactQueries{0};
    qint64 fileBytes{0};
};

struct MachineSafetyExactResult
{
    MachineSafetyIndexState state{MachineSafetyIndexState::Unknown};
    double minimumDistanceMm{-1.0};
    int blockingPair{-1};
    QString failureReason;
};

class MachineSafetyIndex final
{
public:
    bool load(const QString& filePath, QString* errorMessage = nullptr);
    bool save(const QString& filePath, QString* errorMessage = nullptr) const;

    MachineSafetyQueryResult query(const MachineSafetyPose& pose) const;
    MachineSafetyPose cellCenter(std::uint64_t cellIndex) const;
    MachineSafetyPose firstPose(MachineSafetyIndexState state) const;

    bool isValid() const;
    const QByteArray& sourceSha256() const { return m_sourceSha256; }
    const QByteArray& contentSha256() const { return m_contentSha256; }
    const QVector<MachineSafetyAxisGrid>& axes() const { return m_axes; }
    const QVector<MachineSafetyBodySummary>& bodies() const { return m_bodies; }
    const QVector<MachineSafetyPairSummary>& pairs() const { return m_pairs; }
    double clearanceMm() const { return m_clearanceMm; }
    std::uint64_t cellCount() const;
    std::uint64_t stateCount(MachineSafetyIndexState state) const;
    std::uint64_t effectiveCellCount() const;
    std::uint64_t effectiveStateCount(MachineSafetyIndexState state) const;
    double decisionCoverage() const;
    int maximumRefinementLevel() const;
    const SurfaceCollisionModel* persistedSurfaceModel(int bodyIndex) const;

private:
    friend class MachineSafetyIndexCompiler;

    QByteArray m_sourceSha256;
    QByteArray m_contentSha256;
    QVector<MachineSafetyAxisGrid> m_axes;
    QVector<MachineSafetyBodySummary> m_bodies;
    QVector<MachineSafetyPairSummary> m_pairs;
    QVector<SurfaceCollisionModel> m_persistedSurfaceModels;
    double m_clearanceMm{0.0};
    QVector<std::uint8_t> m_states;
    QVector<float> m_clearanceLowerBounds;
    QVector<qint16> m_blockingPairs;
    QVector<qint32> m_refinementOffsets;
    QVector<std::uint8_t> m_refinedStates;
    QVector<float> m_refinedClearanceLowerBounds;
    QVector<qint16> m_refinedBlockingPairs;
    /// Parallel to m_refinedStates. A non-negative value points to the first
    /// child of the next sparse level in the same refined arrays.
    QVector<qint32> m_refinedOffsets;
};

class MachineSafetyIndexCompiler final
{
public:
    MachineSafetyIndexCompiler();
    ~MachineSafetyIndexCompiler();

    MachineSafetyIndexCompiler(const MachineSafetyIndexCompiler&) = delete;
    MachineSafetyIndexCompiler& operator=(const MachineSafetyIndexCompiler&) = delete;

    bool loadMachine(const QString& machineFilePath,
                     QString* errorMessage = nullptr);
    bool build(const MachineSafetyBuildOptions& options,
               MachineSafetyIndex* index,
               MachineSafetyBuildMetrics* metrics = nullptr,
               QString* errorMessage = nullptr) const;
    MachineSafetyExactResult validatePoseExact(
        const MachineSafetyIndex& index,
        const MachineSafetyPose& pose) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

MachineSafetyBuildOptions defaultAcTableSafetyBuildOptions();
QString machineSafetyIndexStateName(MachineSafetyIndexState state);

} // namespace lcnc::cam_algo
