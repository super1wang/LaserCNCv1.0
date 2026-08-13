#pragma once

#include "core/kinematics/machine_topology.h"

#include <QColor>
#include <QString>
#include <QList>
#include <QSet>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <cstdint>
#include <vector>

// Forward declaration
enum class FaceGroupKind;
struct FaceClassification;
class MachineKinematics;

/**
 * @brief Strategy for extracting machining contours from a workpiece.
 *
 * The CAM layer selects the strategy from the user's choice.  The generic
 * default finds the largest smooth-connected exterior surface; the planar
 * strategy is intentionally defined in workpiece XY/Z coordinates.
 */
enum class ExtractionStrategy
{
    LargestSmoothConnectedSurface = 0, ///< Largest smooth-connected exterior surface.
    PlanarFaceWires = 1,               ///< Topmost outer faces visible to a workpiece -Z parallel beam.
    ManualFaceSelection = 3,           ///< User-picked faces -> all wires of those faces.
    LegacyOuterWire = 4                ///< Pre-strategy fallback: OuterWire of every face.
};

/// Convert a persisted strategy value from the former four-option UI.
/// Former Auto (0) and Pipe/section (2) both become the new default.  Manual
/// intentionally retains value 3 so existing manual-selection projects remain
/// manual after the UI removes the obsolete modes.
constexpr ExtractionStrategy extractionStrategyFromPersistedValue(int value)
{
    switch (value) {
    case static_cast<int>(ExtractionStrategy::PlanarFaceWires):
        return ExtractionStrategy::PlanarFaceWires;
    case static_cast<int>(ExtractionStrategy::ManualFaceSelection):
        return ExtractionStrategy::ManualFaceSelection;
    case static_cast<int>(ExtractionStrategy::LegacyOuterWire):
        return ExtractionStrategy::LegacyOuterWire;
    default:
        return ExtractionStrategy::LargestSmoothConnectedSurface;
    }
}

/**
 * @brief Boundary role of a contour, persisted on LaserContour::contourType.
 *
 * Replaces the loose ``FaceGroupKind`` cast so ordering ("holes first, outer
 * last") is driven by an explicit marker rather than name/sequence.
 */
enum class ContourKind
{
    OuterBoundary      = 0, ///< Outer material boundary of the machining face.
    TubeCrossSection   = 1, ///< Tube cross-section contour (outer ∩ cross-section).
                             ///< =1 preserves back-compat with the old persisted
                             ///< FaceGroupKind::CrossSection value.
    InnerHole          = 2, ///< A through-hole boundary inside the machining face.
    Unknown            = 3  ///< Legacy / unclassified.
};

/**
 * @brief Machine-space coordinates for one toolpath point (5-axis: X Y Z R1 R2).
 */
struct MachineCoord
{
    double x{0}, y{0}, z{0};     ///< Linear axis positions (mm)
    double r1{0}, r2{0};         ///< Rotary axis positions (°)
    QString r1Name, r2Name;      ///< Rotary axis names (e.g. "A","C")
    bool   valid{false};         ///< True when IK succeeded
    lcnc::SolvedMachinePose solvedPose; ///< Authoritative v5 layout-ordered pose.
};

/**
 * @brief A single sampled point along a laser cutting contour.
 */
struct ToolpathPoint
{
    gp_Pnt position;      ///< 3D point on the contour
    gp_Dir normal;        ///< Surface normal at this point (laser beam direction)
    gp_Dir crossSectionNormal{0, 0, 1}; ///< Normal of the cross-section owning this point
    bool   crossSectionNormalValid{false};
    gp_Dir tangent;       ///< Tangent direction along the contour (for 5-axis)
    double param{0.0};    ///< Curve parameter on the source edge
    int sourceEdgeIndex{-1}; ///< Stable wire-edge index owning this sample
    MachineCoord machineCoord; ///< Computed machine coordinates (filled by IK)
};

struct ContourGenerationParams
{
    double leadInLength{5.0};
    double deflection{0.1};
};

/**
 * @brief Parameters for a lead-in line (引刀线) on one contour.
 */
struct LeadInParams
{
    double  length{5.0};       ///< Lead-in length in mm
    gp_Pnt  entryPoint;        ///< Entry point on the contour where lead-in meets the path
    double  entryParam{0.0};   ///< Curve parameter at the entry point
    int     entryEdgeIndex{-1};///< Stable wire-edge index of the selected start
    int     entryPointIndex{-1};///< Selected sampled point (-1 = not set)
    bool    valid{false};      ///< True when the user has picked an entry point
};

/**
 * @brief Derived lead-in pose. It is rebuilt from the selected contour start.
 */
struct LeadInSolution
{
    ToolpathPoint point;
    gp_Dir direction{1, 0, 0}; ///< From contour start towards the lead-in point
    QString error;
    bool valid{false};
};

/**
 * @brief Transient topology adjacent to one ordered contour edge.
 *
 * Faces are rebound from the source shape whenever a contour is extracted or
 * recalculated. They are intentionally not persisted in the project package.
 */
struct LeadInEdgeSurfaceContext
{
    std::vector<TopoDS_Face> outerFaces;
    std::vector<TopoDS_Face> crossSectionFaces;
};

/**
 * @brief One closed or open contour extracted from the workpiece.
 */
struct LaserContour
{
    std::uint64_t              contourId{0}; ///< Runtime-stable id, preserved across reordering.
    std::uint64_t              layerId{0};   ///< Runtime-stable layer id used by CAM layer management.
    std::uint64_t              signature{0}; ///< Deterministic geometry fingerprint, used by CamDataManager
                                             ///< to keep contourId stable across regeneration & sessions.
    TopoDS_Wire                wire;     ///< The original topological wire
    TopoDS_Shape               sourceShape; ///< Top-level source shape used for contour extraction/discretisation
    std::vector<ToolpathPoint> points;   ///< Discretised points along the contour
    std::vector<LeadInEdgeSurfaceContext> leadInSurfaceContext; ///< Transient edge-to-face adjacency
    LeadInParams               leadIn;   ///< Lead-in parameters for this contour
    LeadInSolution             leadInSolution; ///< Derived geometry and machine pose
    ContourGenerationParams    appliedParams; ///< Parameters matching the stored points
    ContourGenerationParams    pendingParams; ///< Explicitly edited, not yet applied values
    bool                       needsRecalculation{false};
    bool                       enabled{true};
    QString                    name;
    QString                    workpieceEntry; ///< Mounted workpiece entry owning this contour
    QString                    xcafEntry;   ///< Label entry of this contour's wire inside the unified
                                            ///< project document (EntityKind::Cam). Empty until the
                                            ///< wire has been written into the doc (see CamModule).

    // ── Face-classification metadata (set when using face-based extraction) ──
    int  contourType{static_cast<int>(ContourKind::Unknown)};   ///< ContourKind cast to int
    QString sourceInfo;    ///< Debug info, e.g. "outer ∩ crossSection"
};

struct ToolpathLayer
{
    std::uint64_t layerId{0};
    std::uint64_t signature{0};   ///< Deterministic key fingerprint (mirrors LaserContour::signature)
    QString name;
    QColor color{QColor(80, 190, 150)};
    QString toolName;             ///< 该图层的工具名（自 Phase B 起为唯一权威，替代原 Process 端副本）。
    bool enabled{true};
    std::vector<std::uint64_t> contourIds;

    // ── Phase A: layer-level state moved in from ProcessCuttingPlanService ───
    // These fields default to the previous Process-side semantics so old code
    // paths that ignore them stay correct. They become the single source of
    // truth in Phase B; readers should prefer LayerContainer accessors.
    QString compensationIndex;            ///< Default compensation index for this layer.
    QSet<std::uint64_t> includedContours; ///< Empty = all enabled contours; non-empty = explicit subset (ContourId).
    int manualOrder{0};                   ///< Layer-level order hint; sorting still uses LayerContainer-level order.
};

/**
 * @brief Collection of laser cutting contours with global parameters.
 */
class LaserToolpath
{
public:
    LaserToolpath() = default;

    void clear();
    int  contourCount() const { return static_cast<int>(m_contours.size()); }

    LaserContour&       contour(int i)       { return m_contours[i]; }
    const LaserContour& contour(int i) const { return m_contours[i]; }

    std::vector<LaserContour>&       contours()       { return m_contours; }
    const std::vector<LaserContour>& contours() const { return m_contours; }

    std::vector<ToolpathLayer>&       layers()       { return m_layers; }
    const std::vector<ToolpathLayer>& layers() const { return m_layers; }
    int layerCount() const { return static_cast<int>(m_layers.size()); }

    // Global parameters applied to all contours
    double globalLeadInLength()  const { return m_globalLeadInLength; }

    void setGlobalLeadInLength(double mm)  { m_globalLeadInLength = mm; }

private:
    std::vector<LaserContour> m_contours;
    std::vector<ToolpathLayer> m_layers;
    double m_globalLeadInLength{5.0};
};

/**
 * @brief Static utility class for laser toolpath computation.
 *
 * Responsibilities:
 *  - Extract contour wires from a workpiece shape.
 *  - Discretise contours into sampled points with surface normals.
 *  - Compute a lead-in in the machining face's tangent plane towards free space.
 */
/**
 * @brief Parameters for face-classification-based contour extraction.
 */
struct ContourExtractionParams
{
    double smoothAngleThresholdDeg{5.0}; ///< Angle threshold for smooth face adjacency
    double deflection{0.1};              ///< Chordal deflection for discretisation (mm)
    ExtractionStrategy strategy{ExtractionStrategy::LargestSmoothConnectedSurface}; ///< Extraction strategy

    /// Retained for manual-face callers that need a machining-ray context.
    /// PlanarFaceWires always uses the workpiece -Z direction by definition.
    gp_Dir machiningBeamDirection{0.0, 0.0, -1.0};

    /// User-picked machining faces (ManualFaceSelection). When non-empty, the
    /// algorithm extracts all wires of exactly these faces. They must be
    /// sub-shapes of \a workpiece so edge adjacency resolves.
    std::vector<TopoDS_Face> selectedMachiningFaces;
};

class LaserToolpathBuilder
{
public:
    /// Extract all Wire contours from the workpiece shape.
    /// Falls back to individual edges if no explicit wires are found.
    static std::vector<LaserContour> extractContours(const TopoDS_Shape& workpiece);

    /// Face-classification-based contour extraction (generalised pipe workflow).
    /// Falls back to the legacy method if face classification yields no result.
    static std::vector<LaserContour> extractContours(
        const TopoDS_Shape& workpiece,
        const ContourExtractionParams& params,
        FaceClassification* classificationOut = nullptr);

    /// Retain only trimmed-face samples whose outward normal has a positive Z
    /// component, then select faces first hit by workpiece -Z rays. This keeps
    /// a vertical cylinder's small top cap regardless of smooth-group area,
    /// while vertical side walls and hole walls never enter ray analysis.
    static std::vector<TopoDS_Face> selectTopVisibleFacesFromPositiveZ(
        const TopoDS_Shape& workpiece, QString* info = nullptr);

    /// Legacy single-face helper.  Prefer selectTopVisibleFacesFromPositiveZ() for
    /// global planar extraction.
    static TopoDS_Face selectMachiningFace(const TopoDS_Shape& workpiece,
                                           const gp_Dir& beamDirWpc,
                                           QString* info = nullptr);

    /// Whether \a face is a planar surface.
    static bool isPlanarFace(const TopoDS_Face& face);

    /// Extract contours from an explicit machining-face group. Shared edges
    /// inside that group are removed before chaining, so a manually selected
    /// tube surface has the same external boundary as the classified tube path.
    static std::vector<LaserContour> extractContoursFromFaces(
        const TopoDS_Shape& workpiece,
        const std::vector<TopoDS_Face>& faces,
        const gp_Dir& beamDirWpc,
        const ContourExtractionParams& params);

    /// Extract tube cross-section contours from already separated face groups.
    /// This is deliberately separate from face classification so a user can
    /// review and edit outer/cross-section faces before contour extraction.
    static std::vector<LaserContour> extractTubeContoursFromFaceGroups(
        const TopoDS_Shape& workpiece,
        const std::vector<TopoDS_Face>& outerFaces,
        const std::vector<TopoDS_Face>& crossSectionFaces,
        const ContourExtractionParams& params);

    /// Discretise a contour wire into sampled ToolpathPoints.
    /// @param contour    The contour to populate with sampled points.
    /// @param workpiece  The original workpiece shape (used for surface normal lookup).
    /// @param deflection Chordal deflection for sampling density (mm).
    static void discretizeContour(LaserContour& contour,
                                  const TopoDS_Shape& workpiece,
                                  double deflection = 0.1);

    /// Compute the lead-in approach edge for one contour.
    /// The lead-in line goes from the approach start to the entry point on the contour.
    /// @return A TopoDS_Edge representing the lead-in line, or a null edge if invalid.
    static TopoDS_Edge computeLeadInEdge(const LaserContour& contour);

    /// Build a lead-in in the machining face's tangent plane. Two nearby points
    /// normal to the contour tangent determine which side leaves the trimmed
    /// machining face; the configured length is not validated against solids.
    static LeadInSolution computeLeadInSolution(const LaserContour& contour,
                                                double length);

    /// Make a sampled point the real cutting start. Closed contours are
    /// rotated; open contours only accept either endpoint.
    static bool setContourStart(LaserContour& contour,
                                int pointIndex,
                                QString* error = nullptr);

    /// Choose the first sampled start whose topology can unambiguously resolve
    /// a suspended lead-in side. Interior edge samples are preferred to seam
    /// vertices; an existing manually selected start must use setContourStart().
    static bool setAutomaticContourStart(LaserContour& contour,
                                         QString* error = nullptr);

    /// Bind each ordered wire edge to the exact outer and cross-section faces
    /// that share it. This context is transient and must be rebuilt after load
    /// before rediscretising a contour.
    static void bindLeadInSurfaceContext(
        LaserContour& contour,
        const std::vector<TopoDS_Face>& outerFaces,
        const std::vector<TopoDS_Face>& crossFaces);

    /// Find the surface normal at a point on the workpiece.
    /// Iterates all faces and finds the one closest to the query point.
    static gp_Dir findSurfaceNormal(const TopoDS_Shape& workpiece,
                                    const gp_Pnt& pt);

    /// Find the machining normal at a point, prioritising outer-surface faces.
    /// The normal is perpendicular to the outer surface and consistent with
    /// the cross-section direction.  Falls back to findSurfaceNormal() when
    /// face classification data is unavailable.
    static gp_Dir findMachiningNormal(
        const gp_Pnt& pt,
        const std::vector<TopoDS_Face>& outerFaces);

    /// Discretise a contour with face-classification-aware normals.
    static void discretizeContourWithClassification(
        LaserContour& contour,
        const std::vector<TopoDS_Face>& outerFaces,
        const std::vector<TopoDS_Face>& crossFaces,
        double deflection = 0.1);

    /// v5 mode-explicit batch solver. No solver selection is inferred from
    /// missing axes or configuration-name substrings.
    static bool solveToolpathForOrder(
        const std::vector<LaserContour*>& orderedContours,
        MachineKinematics* kinematics,
        const gp_Trsf& wpcTransform,
        const lcnc::MachineModeDefinition& modeDefinition,
        const lcnc::WorkpieceSetupTransform& workpieceSetup,
        const lcnc::HeadToolGeometry& headToolGeometry,
        QString* errorMessage = nullptr,
        const lcnc::SolvedMachinePose* initialPose = nullptr);

    /// Solve a transient, non-persisted motion path using the same kinematic
    /// branch-continuity rules as cutting contours.  Surface-offset rapid
    /// paths use this entry point so their XYZ and rotary values are produced
    /// together instead of interpolating controller axes independently.
    static bool solveTransientMotionPath(
        std::vector<ToolpathPoint>* points,
        MachineKinematics* kinematics,
        const lcnc::MachineModeDefinition& modeDefinition,
        const lcnc::WorkpieceSetupTransform& workpieceSetup,
        const lcnc::HeadToolGeometry& headToolGeometry,
        QString* errorMessage = nullptr,
        const lcnc::SolvedMachinePose* initialPose = nullptr);

    /// Deterministic hash fingerprint of a face (area + centroid + surface type +
    /// outer-wire vertex count). Stable across session boundaries so that manually
    /// picked machining faces can be rebound after project reload.
    static std::uint64_t computeFaceSignature(const TopoDS_Face& face);

private:
    LaserToolpathBuilder() = delete;
};
