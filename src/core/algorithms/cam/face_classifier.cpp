#include "core/algorithms/cam/face_classifier.h"
#include "core/math/numeric_constants.h"

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_tool.hxx>
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>

#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>


// =============================================================================
// FaceClassification convenience accessors
// =============================================================================

bool FaceClassification::hasCrossSection() const
{
    for (const auto& g : groups)
        if (g.kind == FaceGroupKind::CrossSection) return true;
    return false;
}

const FaceGroup* FaceClassification::outerGroup() const
{
    return hasOuter() ? &groups[outerIdx] : nullptr;
}

std::vector<const FaceGroup*> FaceClassification::crossSectionGroups() const
{
    std::vector<const FaceGroup*> result;
    for (const auto& g : groups)
        if (g.kind == FaceGroupKind::CrossSection) result.push_back(&g);
    return result;
}

std::vector<const FaceGroup*> FaceClassification::innerGroups() const
{
    std::vector<const FaceGroup*> result;
    for (const auto& g : groups)
        if (g.kind == FaceGroupKind::Inner) result.push_back(&g);
    return result;
}

// =============================================================================
// Union-Find (Disjoint Set Union)
// =============================================================================

namespace {

class UnionFind
{
public:
    explicit UnionFind(int n) : m_parent(n), m_rank(n, 0)
    {
        std::iota(m_parent.begin(), m_parent.end(), 0);
    }

    int find(int x)
    {
        while (m_parent[x] != x) {
            m_parent[x] = m_parent[m_parent[x]]; // path halving
            x = m_parent[x];
        }
        return x;
    }

    void unite(int a, int b)
    {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (m_rank[a] < m_rank[b]) std::swap(a, b);
        m_parent[b] = a;
        if (m_rank[a] == m_rank[b]) ++m_rank[a];
    }

    bool connected(int a, int b) { return find(a) == find(b); }

private:
    std::vector<int> m_parent;
    std::vector<int> m_rank;
};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Compute the outward-pointing surface normal of \a face at UV parameter
/// closest to the midpoint of \a edge on that face's parametric domain.
/// Returns false if the normal cannot be determined.
bool computeNormalAtEdgeMid(const TopoDS_Face& face,
                            const TopoDS_Edge& edge,
                            gp_Dir& outNormal)
{
    Standard_Real umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, edge, umin, umax, vmin, vmax);
    Standard_Real umid = (umin + umax) * 0.5;
    Standard_Real vmid = (vmin + vmax) * 0.5;

    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull())
        return false;

    GeomLProp_SLProps props(surf, umid, vmid, 1, 0.01);
    if (!props.IsNormalDefined())
        return false;

    outNormal = props.Normal();
    if (face.Orientation() == TopAbs_REVERSED)
        outNormal.Reverse();
    return true;
}

/// Check whether two faces are smoothly connected along \a edge.
/// "Smooth" means the angle between outward normals < thresholdRad.
bool isSmoothConnection(const TopoDS_Face& face1,
                        const TopoDS_Face& face2,
                        const TopoDS_Edge& edge,
                        double thresholdRad)
{
    gp_Dir n1, n2;
    if (!computeNormalAtEdgeMid(face1, edge, n1))
        return false;
    if (!computeNormalAtEdgeMid(face2, edge, n2))
        return false;
    return n1.Angle(n2) < thresholdRad;
}

/// Compute the total area of a smooth-connected face group.
double smoothGroupArea(const std::vector<TopoDS_Face>& faces)
{
    double area = 0.0;
    for (const auto& f : faces) {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(f, properties);
        area += properties.Mass();
    }
    return area;
}

std::vector<gp_Pnt> sampleOrderedEdges(const std::vector<TopoDS_Edge>& edges)
{
    std::vector<gp_Pnt> points;
    constexpr int kSamplesPerEdge = 8;

    for (const TopoDS_Edge& edge : edges) {
        if (edge.IsNull() || BRep_Tool::Degenerated(edge))
            continue;

        BRepAdaptor_Curve curve(edge);
        const double first = curve.FirstParameter();
        const double last = curve.LastParameter();
        const bool reversed = edge.Orientation() == TopAbs_REVERSED;

        for (int i = 0; i <= kSamplesPerEdge; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(kSamplesPerEdge);
            const double u = reversed
                ? last + (first - last) * t
                : first + (last - first) * t;
            const gp_Pnt p = curve.Value(u);
            if (!points.empty() && points.back().SquareDistance(p) < 1e-12)
                continue;
            points.push_back(p);
        }
    }

    return points;
}

double dominantProjectedSignedArea(const std::vector<gp_Pnt>& points)
{
    if (points.size() < 3)
        return 0.0;

    gp_Vec newell(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const gp_Pnt& a = points[i];
        const gp_Pnt& b = points[(i + 1) % points.size()];
        newell.SetX(newell.X() + (a.Y() - b.Y()) * (a.Z() + b.Z()));
        newell.SetY(newell.Y() + (a.Z() - b.Z()) * (a.X() + b.X()));
        newell.SetZ(newell.Z() + (a.X() - b.X()) * (a.Y() + b.Y()));
    }

    const double ax = std::abs(newell.X());
    const double ay = std::abs(newell.Y());
    const double az = std::abs(newell.Z());
    if (ax >= ay && ax >= az)
        return newell.X();
    if (ay >= ax && ay >= az)
        return newell.Y();
    return newell.Z();
}

bool shouldReverseForClockwise(const std::vector<TopoDS_Edge>& edges)
{
    const std::vector<gp_Pnt> points = sampleOrderedEdges(edges);
    return dominantProjectedSignedArea(points) > 1e-9;
}

TopoDS_Wire makeWireFromOrderedEdges(const std::vector<TopoDS_Edge>& edges, bool reverseOrder)
{
    BRepBuilderAPI_MakeWire maker;
    if (reverseOrder) {
        for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
            TopoDS_Edge edge = *it;
            edge.Reverse();
            maker.Add(edge);
        }
    } else {
        for (const TopoDS_Edge& edge : edges)
            maker.Add(edge);
    }
    return maker.IsDone() ? maker.Wire() : TopoDS_Wire();
}

} // anonymous namespace

// =============================================================================
// FaceClassifier::classifyFaces
// =============================================================================

FaceClassification FaceClassifier::classifyFaces(const TopoDS_Shape& workpiece,
                                                 double smoothAngleThresholdDeg)
{
    FaceClassification result;

    if (workpiece.IsNull())
        return result;

    const double thresholdRad = lcnc::math::degreesToRadians(smoothAngleThresholdDeg);

    // ── Step 1: extract all faces ────────────────────────────────────────
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(workpiece, TopAbs_FACE, faceMap);
    const int nFaces = faceMap.Extent();
    if (nFaces == 0)
        return result;

    // ── Step 2: build edge-to-face adjacency ─────────────────────────────
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
    TopExp::MapShapesAndAncestors(workpiece, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);

    // ── Step 3: Union-Find — merge smoothly-connected face pairs ─────────
    // face indices in faceMap are 1-based; UF uses 0-based.
    UnionFind uf(nFaces);

    // Also record non-smooth edges for later cross-section detection.
    // Key: unordered pair of UF root indices for the two faces.
    struct EdgeRecord {
        int faceIdx1;  // 0-based
        int faceIdx2;  // 0-based
        TopoDS_Edge edge;
    };
    std::vector<EdgeRecord> nonSmoothEdges;

    for (int ei = 1; ei <= edgeToFaceMap.Extent(); ++ei) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeToFaceMap.FindKey(ei));
        if (BRep_Tool::Degenerated(edge))
            continue;

        const TopTools_ListOfShape& adjFaces = edgeToFaceMap.FindFromIndex(ei);
        if (adjFaces.Extent() != 2)
            continue; // boundary edge or non-manifold

        const TopoDS_Face& f1 = TopoDS::Face(adjFaces.First());
        const TopoDS_Face& f2 = TopoDS::Face(adjFaces.Last());

        int idx1 = faceMap.FindIndex(f1) - 1;
        int idx2 = faceMap.FindIndex(f2) - 1;
        if (idx1 < 0 || idx2 < 0)
            continue;

        if (isSmoothConnection(f1, f2, edge, thresholdRad)) {
            uf.unite(idx1, idx2);
        } else {
            nonSmoothEdges.push_back({idx1, idx2, edge});
        }
    }

    // ── Step 4: collect groups from UF representatives ───────────────────
    std::unordered_map<int, int> rootToGroup; // UF root → group index
    for (int i = 0; i < nFaces; ++i) {
        int root = uf.find(i);
        if (rootToGroup.find(root) == rootToGroup.end()) {
            int gi = static_cast<int>(result.groups.size());
            rootToGroup[root] = gi;
            result.groups.emplace_back();
        }
        int gi = rootToGroup[root];
        result.groups[gi].faces.push_back(
            TopoDS::Face(faceMap.FindKey(i + 1)));
    }

    // Compute bounding boxes
    for (auto& g : result.groups) {
        for (const auto& f : g.faces)
            BRepBndLib::Add(f, g.bbox);
    }

    if (result.groups.empty())
        return result;

    // ── Step 5: identify the outer group (largest smooth area) ───────────
    // Area reflects the actual exterior machining surface better than a
    // bounding-box diagonal for long, thin or folded workpieces.
    double maxArea = -1.0;
    double highestExtent = -std::numeric_limits<double>::max();
    for (int gi = 0; gi < static_cast<int>(result.groups.size()); ++gi) {
        const double area = smoothGroupArea(result.groups[gi].faces);
        Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
        result.groups[gi].bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        // Equal-area top/bottom plate faces are both external candidates. Pick
        // the upper one deterministically so the Z-light strategy starts with
        // an exterior group that can actually be illuminated from +Z.
        if (area > maxArea + 1e-9
            || (std::abs(area - maxArea) <= 1e-9 && zMax > highestExtent + 1e-9)) {
            maxArea = area;
            highestExtent = zMax;
            result.outerIdx = gi;
        }
    }
    result.groups[result.outerIdx].kind = FaceGroupKind::Outer;

    // ── Step 6: cross-section groups share non-smooth edges with outer ───
    int outerRoot = -1;
    // find any face in the outer group to get its UF root
    {
        const TopoDS_Face& firstOuter = result.groups[result.outerIdx].faces.front();
        int idx = faceMap.FindIndex(firstOuter) - 1;
        outerRoot = uf.find(idx);
    }

    std::unordered_set<int> crossGroupIndices;
    for (const auto& rec : nonSmoothEdges) {
        int r1 = uf.find(rec.faceIdx1);
        int r2 = uf.find(rec.faceIdx2);
        int otherRoot = -1;
        if (r1 == outerRoot)
            otherRoot = r2;
        else if (r2 == outerRoot)
            otherRoot = r1;
        else
            continue;

        auto it = rootToGroup.find(otherRoot);
        if (it != rootToGroup.end())
            crossGroupIndices.insert(it->second);
    }

    for (int gi : crossGroupIndices) {
        if (gi != result.outerIdx)
            result.groups[gi].kind = FaceGroupKind::CrossSection;
    }

    // ── Step 7: remaining groups → Inner ─────────────────────────────────
    for (auto& g : result.groups) {
        if (g.kind == FaceGroupKind::Unknown)
            g.kind = FaceGroupKind::Inner;
    }

    return result;
}

// =============================================================================
// FaceClassifier::extractContourEdges
// =============================================================================

std::vector<TopoDS_Edge> FaceClassifier::extractContourEdges(
    const FaceClassification& classification)
{
    std::vector<TopoDS_Edge> result;

    if (!classification.hasOuter() || !classification.hasCrossSection())
        return result;

    // Build a set of all outer faces for quick lookup
    const FaceGroup& outerGrp = *classification.outerGroup();
    TopTools_IndexedMapOfShape outerFaceMap;
    for (const auto& f : outerGrp.faces)
        outerFaceMap.Add(f);

    // Build a set of all cross-section faces
    TopTools_IndexedMapOfShape crossFaceMap;
    for (const auto* cg : classification.crossSectionGroups())
        for (const auto& f : cg->faces)
            crossFaceMap.Add(f);

    // For each outer face, collect edges that are also adjacent to a cross-section face.
    // We iterate edges of outer faces and check if any ancestor face is a cross-section.
    TopTools_IndexedMapOfShape collectedEdges; // dedup
    for (const auto& outerFace : outerGrp.faces) {
        for (TopExp_Explorer exp(outerFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
            if (BRep_Tool::Degenerated(edge))
                continue;
            if (collectedEdges.Contains(edge))
                continue;

            // Check all cross-section faces for this edge
            for (int ci = 1; ci <= crossFaceMap.Extent(); ++ci) {
                const TopoDS_Face& crossFace = TopoDS::Face(crossFaceMap.FindKey(ci));
                for (TopExp_Explorer exp2(crossFace, TopAbs_EDGE); exp2.More(); exp2.Next()) {
                    if (edge.IsSame(exp2.Current())) {
                        collectedEdges.Add(edge);
                        result.push_back(edge);
                        goto next_edge; // found, skip to next outer edge
                    }
                }
            }
            next_edge:;
        }
    }

    return result;
}

// =============================================================================
// FaceClassifier::chainEdgesToWires
// =============================================================================

std::vector<TopoDS_Wire> FaceClassifier::chainEdgesToWires(
    const std::vector<TopoDS_Edge>& edges,
    double tolerance)
{
    std::vector<TopoDS_Wire> result;

    if (edges.empty())
        return result;

    // Build endpoint structures
    struct EdgeEntry {
        TopoDS_Edge edge;
        gp_Pnt      startPt;
        gp_Pnt      endPt;
        bool         used{false};
    };
    std::vector<EdgeEntry> entries;
    entries.reserve(edges.size());

    for (const auto& e : edges) {
        TopoDS_Vertex vFirst, vLast;
        TopExp::Vertices(e, vFirst, vLast, Standard_True);
        if (vFirst.IsNull() || vLast.IsNull())
            continue;
        entries.push_back({e,
                           BRep_Tool::Pnt(vFirst),
                           BRep_Tool::Pnt(vLast),
                           false});
    }

    // Greedy chaining: pick an unused edge, extend the chain as far as possible.
    auto findNext = [&](const gp_Pnt& tail, int exclude) -> int {
        double bestDist = tolerance;
        int bestIdx = -1;
        for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
            if (i == exclude || entries[i].used)
                continue;
            double d1 = tail.Distance(entries[i].startPt);
            double d2 = tail.Distance(entries[i].endPt);
            double d = std::min(d1, d2);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    };

    for (int start = 0; start < static_cast<int>(entries.size()); ++start) {
        if (entries[start].used)
            continue;

        // Start a new chain.
        std::vector<TopoDS_Edge> orderedEdges;
        orderedEdges.push_back(entries[start].edge);
        entries[start].used = true;

        // Determine orientation: keep the original start/end while chaining,
        // then normalise the completed wire to clockwise in its dominant projection.
        gp_Pnt chainTail = entries[start].endPt;

        // Extend from tail
        for (;;) {
            int next = findNext(chainTail, -1);
            if (next < 0)
                break;

            // Check if we need to reverse (tail connects to endPt instead of startPt)
            double dStart = chainTail.Distance(entries[next].startPt);
            double dEnd   = chainTail.Distance(entries[next].endPt);

            entries[next].used = true;
            TopoDS_Edge edgeToAdd = entries[next].edge;

            if (dEnd < dStart) {
                // Reverse the edge orientation so its "start" aligns with chain tail
                edgeToAdd.Reverse();
                chainTail = entries[next].startPt;
            } else {
                chainTail = entries[next].endPt;
            }
            orderedEdges.push_back(edgeToAdd);
        }

        TopoDS_Wire wire = makeWireFromOrderedEdges(
            orderedEdges,
            shouldReverseForClockwise(orderedEdges));
        if (wire.IsNull())
            wire = makeWireFromOrderedEdges(orderedEdges, false);
        if (!wire.IsNull()) {
            result.push_back(wire);
        }
    }

    // Sort: closed wires first (more useful for machining)
    std::sort(result.begin(), result.end(),
              [](const TopoDS_Wire& a, const TopoDS_Wire& b) {
                  return a.Closed() && !b.Closed();
              });

    return result;
}
