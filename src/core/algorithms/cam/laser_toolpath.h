#pragma once

#include <QColor>
#include <QString>
#include <QList>

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
 * @brief Machine-space coordinates for one toolpath point (5-axis: X Y Z R1 R2).
 */
struct MachineCoord
{
    double x{0}, y{0}, z{0};     ///< Linear axis positions (mm)
    double r1{0}, r2{0};         ///< Rotary axis positions (°)
    QString r1Name, r2Name;      ///< Rotary axis names (e.g. "A","C")
    bool   valid{false};         ///< True when IK succeeded
};

/**
 * @brief A single sampled point along a laser cutting contour.
 */
struct ToolpathPoint
{
    gp_Pnt position;      ///< 3D point on the contour
    gp_Dir normal;        ///< Surface normal at this point (laser beam direction)
    gp_Dir tangent;       ///< Tangent direction along the contour (for 5-axis)
    double param{0.0};    ///< Curve parameter on the source edge
    MachineCoord machineCoord; ///< Computed machine coordinates (filled by IK)
};

/**
 * @brief Parameters for a lead-in line (引刀线) on one contour.
 */
struct LeadInParams
{
    double  length{5.0};       ///< Lead-in length in mm
    double  normalAngle{0.0};  ///< Normal angle offset in degrees
    gp_Pnt  entryPoint;        ///< Entry point on the contour where lead-in meets the path
    double  entryParam{0.0};   ///< Curve parameter at the entry point
    int     entryEdgeIndex{-1};///< Index of the edge within the wire (-1 = not set)
    bool    valid{false};      ///< True when the user has picked an entry point
};

/**
 * @brief One closed or open contour extracted from the workpiece.
 */
struct LaserContour
{
    std::uint64_t              contourId{0}; ///< Runtime-stable id, preserved across reordering.
    std::uint64_t              layerId{0};   ///< Runtime-stable layer id used by CAM layer management.
    TopoDS_Wire                wire;     ///< The original topological wire
    TopoDS_Shape               sourceShape; ///< Top-level source shape used for contour extraction/discretisation
    std::vector<ToolpathPoint> points;   ///< Discretised points along the contour
    LeadInParams               leadIn;   ///< Lead-in parameters for this contour
    bool                       enabled{true};
    QString                    name;
    QString                    workpieceEntry; ///< Mounted workpiece entry owning this contour

    // ── Face-classification metadata (set when using face-based extraction) ──
    int  contourType{3};   ///< FaceGroupKind cast to int (3 = Unknown / legacy)
    QString sourceInfo;    ///< Debug info, e.g. "outer ∩ crossSection"
};

struct ToolpathLayer
{
    std::uint64_t layerId{0};
    QString name;
    QColor color{QColor(80, 190, 150)};
    QString toolName;
    bool enabled{true};
    std::vector<std::uint64_t> contourIds;
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
    double globalNormalAngle()   const { return m_globalNormalAngle; }

    void setGlobalLeadInLength(double mm)  { m_globalLeadInLength = mm; }
    void setGlobalNormalAngle(double deg)  { m_globalNormalAngle = deg; }

private:
    std::vector<LaserContour> m_contours;
    std::vector<ToolpathLayer> m_layers;
    double m_globalLeadInLength{5.0};
    double m_globalNormalAngle{0.0};
};

/**
 * @brief Static utility class for laser toolpath computation.
 *
 * Responsibilities:
 *  - Extract contour wires from a workpiece shape.
 *  - Discretise contours into sampled points with surface normals.
 *  - Compute lead-in geometry (approach line that enters from non-vertical direction).
 */
/**
 * @brief Parameters for face-classification-based contour extraction.
 */
struct ContourExtractionParams
{
    double smoothAngleThresholdDeg{5.0}; ///< Angle threshold for smooth face adjacency
    double deflection{0.1};              ///< Chordal deflection for discretisation (mm)
    bool   useFaceClassification{true};  ///< true = face-based, false = legacy OuterWire
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
    /// Direction is ensured to NOT come from directly above the workpiece.
    /// @return A TopoDS_Edge representing the lead-in line, or a null edge if invalid.
    static TopoDS_Edge computeLeadInEdge(const LaserContour& contour,
                                         double length,
                                         double normalAngleDeg);

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
        const std::vector<TopoDS_Face>& outerFaces,
        const std::vector<TopoDS_Face>& crossFaces);

    /// Discretise a contour with face-classification-aware normals.
    static void discretizeContourWithClassification(
        LaserContour& contour,
        const std::vector<TopoDS_Face>& outerFaces,
        const std::vector<TopoDS_Face>& crossFaces,
        double deflection = 0.1);

    /// Adjust an approach direction so it does not come from directly above.
    /// If the angle between the approach direction and +Z is less than the
    /// threshold (default 15°), the direction is rotated away from vertical.
    static gp_Dir ensureNotFromAbove(const gp_Dir& approachDir,
                                     double thresholdDeg = 15.0);

    /// Compute machine coordinates (IK) for all points in a contour.
    /// @param contour       The contour whose points will be updated with machine coords.
    /// @param kinematics    The machine kinematic model (provides config type and axis defs).
    /// @param wpcTransform  World transform of the workpiece (from kin->computeWpcTransform).
    static void computeMachineCoordinates(LaserContour& contour,
                                          MachineKinematics* kinematics,
                                          const gp_Trsf& wpcTransform);

private:
    LaserToolpathBuilder() = delete;
};
