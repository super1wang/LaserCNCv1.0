#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <Bnd_Box.hxx>

#include <vector>

/**
 * @brief Classification tag for a group of smoothly-connected faces.
 */
enum class FaceGroupKind
{
    Outer        = 0,  ///< Largest smooth-connected shell (外表面)
    CrossSection = 1,  ///< Shares non-smooth edges with outer group (截面)
    Inner        = 2,  ///< Neither outer nor cross-section (内表面)
    Unknown      = 3
};

/**
 * @brief A connected group of faces whose pairwise shared edges are smooth.
 */
struct FaceGroup
{
    std::vector<TopoDS_Face> faces;
    Bnd_Box                  bbox;
    FaceGroupKind            kind{FaceGroupKind::Unknown};
};

/**
 * @brief Result of face classification on a workpiece shape.
 */
struct FaceClassification
{
    std::vector<FaceGroup> groups;      ///< All smooth-connected face groups
    int                    outerIdx{-1};///< Index of the outer group in \c groups (-1 = none)

    /// Convenience accessors.
    bool hasOuter()        const { return outerIdx >= 0; }
    bool hasCrossSection() const;

    const FaceGroup* outerGroup()  const;
    std::vector<const FaceGroup*> crossSectionGroups() const;
    std::vector<const FaceGroup*> innerGroups()        const;
};

/**
 * @brief Static utility for classifying faces by smooth connectivity
 *        and extracting machining contours at face-group boundaries.
 *
 * Algorithm (generalised from Gugao pipe-cutting workflow):
 *  1. Extract all faces and edges, build face↔edge mapping.
 *  2. For each edge shared by two faces, compare surface normals at the
 *     edge midpoint — if the angle is below a threshold, the connection
 *     is "smooth".
 *  3. Union-Find merges smoothly connected faces into groups.
 *  4. The group with the largest bounding box diagonal is the outer surface.
 *  5. Groups sharing non-smooth edges with the outer surface are cross-sections.
 *  6. Remaining groups are inner surfaces.
 *
 * Contour extraction:
 *  - Edges shared between the outer group and cross-section groups form
 *    the machining contour boundary.
 *  - Those edges are chained into wires (closed or open).
 */
class FaceClassifier
{
public:
    // ── Face classification ──────────────────────────────────────────────

    /// Classify all faces of \a workpiece into smooth-connected groups.
    /// @param smoothAngleThresholdDeg  Maximum angle (degrees) between
    ///        adjacent surface normals to consider the edge smooth.
    static FaceClassification classifyFaces(
        const TopoDS_Shape& workpiece,
        double smoothAngleThresholdDeg = 5.0);

    // ── Contour extraction ───────────────────────────────────────────────

    /// Collect edges shared between faces of \a outerGroup and \a crossGroups.
    static std::vector<TopoDS_Edge> extractContourEdges(
        const FaceClassification& classification);

    /// Chain a set of loose edges into connected wires.
    /// @param tolerance  Maximum vertex-to-vertex gap to treat as connected.
    /// @return Ordered list of wires (closed contours first).
    static std::vector<TopoDS_Wire> chainEdgesToWires(
        const std::vector<TopoDS_Edge>& edges,
        double tolerance = 0.02);
};
